from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, replace
from typing import Optional

from .imu_bno055 import IMUReading, parse_serial_imu_line


@dataclass(frozen=True)
class HandForceReading:
    force: float
    voltage: float = 0.0
    raw: Optional[int] = None
    sensor_time_ms: int = 0


@dataclass(frozen=True)
class SensorSnapshot:
    imu: Optional[IMUReading]
    hand_force: Optional[HandForceReading]


class LowPass:
    def __init__(self, alpha: float) -> None:
        self.alpha = max(0.01, min(1.0, alpha))
        self.value: Optional[float] = None

    def update(self, sample: float) -> float:
        self.value = sample if self.value is None else self.value + self.alpha * (sample - self.value)
        return self.value

    def reset(self) -> None:
        self.value = None


def parse_serial_hand_line(
    line: str,
    invert: bool = False,
    zero_raw: int = 0,
    full_raw: int = 4095,
) -> Optional[HandForceReading]:
    fields = [field.strip() for field in line.strip().split(",")]
    if not fields or fields[0] != "H":
        return None

    values = fields[1:]
    sensor_time_ms = 0
    if len(values) == 4:
        try:
            sensor_time_ms = int(values[0])
        except ValueError:
            return None
        values = values[1:]
    if len(values) < 1:
        return None

    try:
        force = max(0.0, min(1.0, float(values[0])))
        voltage = float(values[1]) if len(values) >= 2 else force * 3.3
        raw = int(values[2]) if len(values) >= 3 else None
    except ValueError:
        return None

    if raw is not None:
        span = max(1, full_raw - zero_raw)
        force = max(0.0, min(1.0, (raw - zero_raw) / span))

    if invert:
        force = 1.0 - force

    return HandForceReading(force, voltage, raw, sensor_time_ms)


class RobotSensorHub:
    """Single ESP32 USB sensor stream: Q lines for IMU, H lines for hand force."""

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        timeout_s: float = 0.25,
        use_imu: bool = True,
        use_hand_fsr: bool = False,
        imu_roll_sign: float = 1.0,
        imu_pitch_sign: float = 1.0,
        imu_yaw_sign: float = 1.0,
        imu_vertical_mount: bool = True,
        imu_board_face_sign: float = 1.0,
        hand_fsr_invert: bool = False,
        hand_fsr_filter_alpha: float = 0.18,
        hand_fsr_zero_raw: int = 0,
        hand_fsr_full_raw: int = 4095,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = max(0.05, timeout_s)
        self.use_imu = use_imu
        self.use_hand_fsr = use_hand_fsr
        self.imu_roll_sign = imu_roll_sign
        self.imu_pitch_sign = imu_pitch_sign
        self.imu_yaw_sign = imu_yaw_sign
        self.imu_vertical_mount = imu_vertical_mount
        self.imu_board_face_sign = 1.0 if imu_board_face_sign >= 0.0 else -1.0
        self.hand_fsr_invert = hand_fsr_invert
        self.hand_fsr_zero_raw = hand_fsr_zero_raw
        self.hand_fsr_full_raw = max(hand_fsr_zero_raw + 1, hand_fsr_full_raw)
        self.hand_filter = LowPass(hand_fsr_filter_alpha)

        self._serial = None
        self._serial_factory = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._imu: Optional[IMUReading] = None
        self._imu_at = 0.0
        self._hand_force: Optional[HandForceReading] = None
        self._hand_force_at = 0.0
        self._gravity_basis: Optional[
            tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
        ] = None

    def open(self) -> None:
        try:
            import serial
        except ImportError as exc:
            raise ImportError("ESP32 sensor support requires: pip install pyserial") from exc

        try:
            self._serial_factory = serial.Serial
            self._serial = self._serial_factory(self.port, self.baudrate, timeout=0.05)
        except Exception as exc:
            raise RuntimeError(f"Cannot open ESP32 sensor port {self.port}: {exc}") from exc

        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, name="esp32-sensors", daemon=True)
        self._thread.start()

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            if self._serial is None:
                try:
                    assert self._serial_factory is not None
                    self._serial = self._serial_factory(self.port, self.baudrate, timeout=0.05)
                    print(f"[sensors] Reconnected ESP32 on {self.port}.")
                except Exception:
                    self._stop.wait(0.5)
                    continue
            try:
                raw = self._serial.readline()
            except Exception:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
                with self._lock:
                    self._imu = None
                    self._hand_force = None
                    self.hand_filter.reset()
                if not self._stop.is_set():
                    self._stop.wait(0.25)
                continue
            if not raw:
                continue

            line = raw.decode("ascii", errors="replace")
            now = time.monotonic()

            imu = (
                parse_serial_imu_line(
                    line,
                    roll_sign=self.imu_roll_sign,
                    pitch_sign=self.imu_pitch_sign,
                    yaw_sign=self.imu_yaw_sign,
                )
                if self.use_imu
                else None
            )
            if imu is not None:
                if self.imu_vertical_mount and self._gravity_basis is not None:
                    imu = self._apply_vertical_mount(imu)
                with self._lock:
                    self._imu = imu
                    self._imu_at = now
                continue

            hand_force = (
                parse_serial_hand_line(
                    line,
                    invert=self.hand_fsr_invert,
                    zero_raw=self.hand_fsr_zero_raw,
                    full_raw=self.hand_fsr_full_raw,
                )
                if self.use_hand_fsr
                else None
            )
            if hand_force is not None:
                with self._lock:
                    self._hand_force = HandForceReading(
                        self.hand_filter.update(hand_force.force),
                        hand_force.voltage,
                        hand_force.raw,
                        hand_force.sensor_time_ms,
                    )
                    self._hand_force_at = now

    def read(self) -> SensorSnapshot:
        now = time.monotonic()
        with self._lock:
            imu = self._imu if self._imu is not None and now - self._imu_at <= self.timeout_s else None
            hand_force = (
                self._hand_force
                if self._hand_force is not None and now - self._hand_force_at <= self.timeout_s
                else None
            )
        return SensorSnapshot(imu=imu, hand_force=hand_force)

    def capture_imu_reference(
        self,
        sample_seconds: float = 1.5,
        timeout_s: float = 8.0,
        min_samples: int = 25,
        min_gyro_cal: int = 1,
        min_accel_cal: int = 1,
        max_rms_deg: float = 2.0,
    ) -> Optional[tuple[float, float]]:
        deadline = time.monotonic() + max(sample_seconds, timeout_s)
        started_at: Optional[float] = None
        samples: list[IMUReading] = []
        last_sensor_time: Optional[int] = None
        self._gravity_basis = None

        while not self._stop.is_set() and time.monotonic() < deadline:
            reading = self.read().imu
            if (
                reading is None
                or not reading.balance_ready(min_gyro_cal, min_accel_cal)
                or reading.sensor_time_ms == last_sensor_time
            ):
                time.sleep(0.01)
                continue

            last_sensor_time = reading.sensor_time_ms
            samples.append(reading)
            if started_at is None:
                started_at = time.monotonic()
            if time.monotonic() - started_at >= sample_seconds and len(samples) >= min_samples:
                break
            time.sleep(0.005)

        if len(samples) < min_samples:
            return None

        if self.imu_vertical_mount:
            vertical_reference = self._build_vertical_reference(samples, max_rms_deg)
            if vertical_reference is None:
                return None
            self._gravity_basis = vertical_reference
            return (0.0, 0.0)

        roll = self._circular_mean_deg(value.roll_deg for value in samples)
        pitch = self._circular_mean_deg(value.pitch_deg for value in samples)
        rms = math.sqrt(
            sum(
                self._angle_delta_deg(sample.roll_deg, roll) ** 2
                + self._angle_delta_deg(sample.pitch_deg, pitch) ** 2
                for sample in samples
            )
            / (2.0 * len(samples))
        )
        return (roll, pitch) if rms <= max_rms_deg else None

    def _build_vertical_reference(
        self,
        samples: list[IMUReading],
        max_rms_deg: float,
    ) -> Optional[
        tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    ]:
        gravity = [
            (sample.gravity_x, sample.gravity_y, sample.gravity_z)
            for sample in samples
            if sample.gravity_x is not None and sample.gravity_y is not None and sample.gravity_z is not None
        ]
        if len(gravity) != len(samples):
            return None

        mean = self._normalize_vector(
            tuple(sum(float(vector[axis]) for vector in gravity) for axis in range(3))
        )
        if mean is None:
            return None

        angular_errors = []
        for vector in gravity:
            normalized = self._normalize_vector(tuple(float(value) for value in vector))
            if normalized is None:
                return None
            dot = max(-1.0, min(1.0, self._dot(normalized, mean)))
            angular_errors.append(math.degrees(math.acos(dot)))
        rms = math.sqrt(sum(error * error for error in angular_errors) / len(angular_errors))
        if rms > max_rms_deg:
            return None

        forward_seed = (0.0, 0.0, self.imu_board_face_sign)
        projection = self._dot(forward_seed, mean)
        forward = self._normalize_vector(
            tuple(forward_seed[axis] - projection * mean[axis] for axis in range(3))
        )
        if forward is None:
            return None
        left = self._normalize_vector(self._cross(mean, forward))
        if left is None:
            return None
        return forward, left, mean

    def _apply_vertical_mount(self, reading: IMUReading) -> IMUReading:
        if (
            self._gravity_basis is None
            or reading.gravity_x is None
            or reading.gravity_y is None
            or reading.gravity_z is None
        ):
            return reading
        gravity = self._normalize_vector((reading.gravity_x, reading.gravity_y, reading.gravity_z))
        if gravity is None:
            return reading
        forward, left, up = self._gravity_basis
        up_component = self._dot(gravity, up)
        pitch = -math.degrees(math.atan2(self._dot(gravity, forward), up_component))
        roll = -math.degrees(math.atan2(self._dot(gravity, left), up_component))
        return replace(
            reading,
            roll_deg=roll * self.imu_roll_sign,
            pitch_deg=pitch * self.imu_pitch_sign,
        )

    @staticmethod
    def _normalize_vector(vector: tuple[float, float, float]) -> Optional[tuple[float, float, float]]:
        norm = math.sqrt(sum(value * value for value in vector))
        if norm < 1e-6:
            return None
        return tuple(value / norm for value in vector)

    @staticmethod
    def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
        return sum(a[index] * b[index] for index in range(3))

    @staticmethod
    def _cross(
        a: tuple[float, float, float],
        b: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        return (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )

    @staticmethod
    def _circular_mean_deg(values) -> float:
        radians = [math.radians(value) for value in values]
        return math.degrees(
            math.atan2(
                sum(math.sin(value) for value in radians),
                sum(math.cos(value) for value in radians),
            )
        )

    @staticmethod
    def _angle_delta_deg(value: float, reference: float) -> float:
        return (value - reference + 180.0) % 360.0 - 180.0

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None
