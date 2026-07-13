from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

from .imu_bno055 import IMUReading, parse_serial_imu_line


@dataclass(frozen=True)
class FootLoadReading:
    left: float
    right: float
    left_voltage: float = 0.0
    right_voltage: float = 0.0
    left_raw: Optional[int] = None
    right_raw: Optional[int] = None
    sensor_time_ms: int = 0

    @property
    def total(self) -> float:
        return max(0.0, self.left + self.right)

    @property
    def left_ratio(self) -> float:
        return 0.5 if self.total <= 1e-6 else self.left / self.total

    @property
    def right_ratio(self) -> float:
        return 0.5 if self.total <= 1e-6 else self.right / self.total


@dataclass(frozen=True)
class SensorSnapshot:
    imu: Optional[IMUReading]
    foot_load: Optional[FootLoadReading]


class LowPass:
    def __init__(self, alpha: float) -> None:
        self.alpha = max(0.01, min(1.0, alpha))
        self.value: Optional[float] = None

    def update(self, sample: float) -> float:
        self.value = sample if self.value is None else self.value + self.alpha * (sample - self.value)
        return self.value

    def reset(self) -> None:
        self.value = None


def parse_serial_fsr_line(
    line: str,
    invert: bool = False,
    left_zero_raw: int = 0,
    left_full_raw: int = 4095,
    right_zero_raw: int = 0,
    right_full_raw: int = 4095,
) -> Optional[FootLoadReading]:
    fields = [field.strip() for field in line.strip().split(",")]
    if not fields or fields[0] != "F":
        return None

    values = fields[1:]
    sensor_time_ms = 0
    if len(values) in {3, 5, 7}:
        try:
            sensor_time_ms = int(values[0])
        except ValueError:
            return None
        values = values[1:]
    if len(values) < 2:
        return None

    try:
        left = max(0.0, min(1.0, float(values[0])))
        right = max(0.0, min(1.0, float(values[1])))
        left_voltage = float(values[2]) if len(values) >= 3 else left * 3.3
        right_voltage = float(values[3]) if len(values) >= 4 else right * 3.3
        left_raw = int(values[4]) if len(values) >= 5 else None
        right_raw = int(values[5]) if len(values) >= 6 else None
    except ValueError:
        return None

    if left_raw is not None:
        left_span = max(1, left_full_raw - left_zero_raw)
        left = max(0.0, min(1.0, (left_raw - left_zero_raw) / left_span))
    if right_raw is not None:
        right_span = max(1, right_full_raw - right_zero_raw)
        right = max(0.0, min(1.0, (right_raw - right_zero_raw) / right_span))

    if invert:
        left = 1.0 - left
        right = 1.0 - right

    return FootLoadReading(
        left,
        right,
        left_voltage,
        right_voltage,
        left_raw,
        right_raw,
        sensor_time_ms,
    )


class RobotSensorHub:
    """Single ESP32 USB sensor stream: Q lines for IMU, F lines for FSR."""

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        timeout_s: float = 0.25,
        use_imu: bool = True,
        use_fsr: bool = False,
        imu_roll_sign: float = 1.0,
        imu_pitch_sign: float = 1.0,
        imu_yaw_sign: float = 1.0,
        fsr_invert: bool = False,
        fsr_filter_alpha: float = 0.18,
        fsr_left_zero_raw: int = 0,
        fsr_left_full_raw: int = 4095,
        fsr_right_zero_raw: int = 0,
        fsr_right_full_raw: int = 4095,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = max(0.05, timeout_s)
        self.use_imu = use_imu
        self.use_fsr = use_fsr
        self.imu_roll_sign = imu_roll_sign
        self.imu_pitch_sign = imu_pitch_sign
        self.imu_yaw_sign = imu_yaw_sign
        self.fsr_invert = fsr_invert
        self.fsr_left_zero_raw = fsr_left_zero_raw
        self.fsr_left_full_raw = max(fsr_left_zero_raw + 1, fsr_left_full_raw)
        self.fsr_right_zero_raw = fsr_right_zero_raw
        self.fsr_right_full_raw = max(fsr_right_zero_raw + 1, fsr_right_full_raw)
        self.left_filter = LowPass(fsr_filter_alpha)
        self.right_filter = LowPass(fsr_filter_alpha)

        self._serial = None
        self._serial_factory = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._imu: Optional[IMUReading] = None
        self._imu_at = 0.0
        self._fsr: Optional[FootLoadReading] = None
        self._fsr_at = 0.0

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
                    self._fsr = None
                    self.left_filter.reset()
                    self.right_filter.reset()
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
                with self._lock:
                    self._imu = imu
                    self._imu_at = now
                continue

            fsr = (
                parse_serial_fsr_line(
                    line,
                    invert=self.fsr_invert,
                    left_zero_raw=self.fsr_left_zero_raw,
                    left_full_raw=self.fsr_left_full_raw,
                    right_zero_raw=self.fsr_right_zero_raw,
                    right_full_raw=self.fsr_right_full_raw,
                )
                if self.use_fsr
                else None
            )
            if fsr is not None:
                with self._lock:
                    self._fsr = FootLoadReading(
                        self.left_filter.update(fsr.left),
                        self.right_filter.update(fsr.right),
                        fsr.left_voltage,
                        fsr.right_voltage,
                        fsr.left_raw,
                        fsr.right_raw,
                        fsr.sensor_time_ms,
                    )
                    self._fsr_at = now

    def read(self) -> SensorSnapshot:
        now = time.monotonic()
        with self._lock:
            imu = self._imu if self._imu is not None and now - self._imu_at <= self.timeout_s else None
            fsr = self._fsr if self._fsr is not None and now - self._fsr_at <= self.timeout_s else None
        return SensorSnapshot(imu=imu, foot_load=fsr)

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
        samples: list[tuple[float, float]] = []
        last_sensor_time: Optional[int] = None

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
            samples.append((reading.roll_deg, reading.pitch_deg))
            if started_at is None:
                started_at = time.monotonic()
            if time.monotonic() - started_at >= sample_seconds and len(samples) >= min_samples:
                break
            time.sleep(0.005)

        if len(samples) < min_samples:
            return None

        roll = self._circular_mean_deg(value[0] for value in samples)
        pitch = self._circular_mean_deg(value[1] for value in samples)
        rms = math.sqrt(
            sum(
                self._angle_delta_deg(sample_roll, roll) ** 2
                + self._angle_delta_deg(sample_pitch, pitch) ** 2
                for sample_roll, sample_pitch in samples
            )
            / (2.0 * len(samples))
        )
        return (roll, pitch) if rms <= max_rms_deg else None

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
