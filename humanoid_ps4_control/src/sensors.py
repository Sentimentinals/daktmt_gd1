from __future__ import annotations

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


def parse_serial_fsr_line(line: str, invert: bool = False) -> Optional[FootLoadReading]:
    fields = [field.strip() for field in line.strip().split(",")]
    if not fields or fields[0] != "F":
        return None

    values = fields[1:]
    if len(values) in {3, 5, 7}:
        values = values[1:]
    if len(values) < 2:
        return None

    try:
        left = max(0.0, min(1.0, float(values[0])))
        right = max(0.0, min(1.0, float(values[1])))
        left_voltage = float(values[2]) if len(values) >= 3 else left * 3.3
        right_voltage = float(values[3]) if len(values) >= 4 else right * 3.3
    except ValueError:
        return None

    if invert:
        left = 1.0 - left
        right = 1.0 - right

    return FootLoadReading(left, right, left_voltage, right_voltage)


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
        self.left_filter = LowPass(fsr_filter_alpha)
        self.right_filter = LowPass(fsr_filter_alpha)

        self._serial = None
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
            self._serial = serial.Serial(self.port, self.baudrate, timeout=0.05)
        except Exception as exc:
            raise RuntimeError(f"Cannot open ESP32 sensor port {self.port}: {exc}") from exc

        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, name="esp32-sensors", daemon=True)
        self._thread.start()

    def _read_loop(self) -> None:
        assert self._serial is not None
        while not self._stop.is_set():
            raw = self._serial.readline()
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

            fsr = parse_serial_fsr_line(line, invert=self.fsr_invert) if self.use_fsr else None
            if fsr is not None:
                with self._lock:
                    self._fsr = FootLoadReading(
                        self.left_filter.update(fsr.left),
                        self.right_filter.update(fsr.right),
                        fsr.left_voltage,
                        fsr.right_voltage,
                    )
                    self._fsr_at = now

    def read(self) -> SensorSnapshot:
        now = time.monotonic()
        with self._lock:
            imu = self._imu if self._imu is not None and now - self._imu_at <= self.timeout_s else None
            fsr = self._fsr if self._fsr is not None and now - self._fsr_at <= self.timeout_s else None
        return SensorSnapshot(imu=imu, foot_load=fsr)

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None
        if self._serial is not None:
            self._serial.close()
            self._serial = None
