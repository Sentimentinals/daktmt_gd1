from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional

from .imu_bno055 import BNO055Reader, IMUReading, parse_serial_imu_line


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
        total = self.total
        return 0.5 if total <= 1e-6 else self.left / total

    @property
    def right_ratio(self) -> float:
        total = self.total
        return 0.5 if total <= 1e-6 else self.right / total


@dataclass(frozen=True)
class SensorSnapshot:
    imu: Optional[IMUReading]
    foot_load: Optional[FootLoadReading]


class LowPass:
    def __init__(self, alpha: float) -> None:
        self.alpha = max(0.01, min(1.0, alpha))
        self.value: Optional[float] = None

    def update(self, sample: float) -> float:
        if self.value is None:
            self.value = sample
        else:
            self.value = self.value + self.alpha * (sample - self.value)
        return self.value


class ADS1115FootLoadReader:
    def __init__(
        self,
        address: int = 0x48,
        left_channel: int = 0,
        right_channel: int = 1,
        invert: bool = False,
        filter_alpha: float = 0.18,
    ) -> None:
        self.address = address
        self.left_channel = left_channel
        self.right_channel = right_channel
        self.invert = invert
        self.left_filter = LowPass(filter_alpha)
        self.right_filter = LowPass(filter_alpha)
        self._channels = None

    def open(self) -> None:
        try:
            import board  # type: ignore
            import busio  # type: ignore
            import adafruit_ads1x15.ads1115 as ADS  # type: ignore
            from adafruit_ads1x15.analog_in import AnalogIn  # type: ignore
        except ImportError as exc:
            raise ImportError(
                "FSR support requires: pip install adafruit-circuitpython-ads1x15"
            ) from exc

        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c, address=self.address)
        pins = [ADS.P0, ADS.P1, ADS.P2, ADS.P3]
        self._channels = (
            AnalogIn(ads, pins[self.left_channel]),
            AnalogIn(ads, pins[self.right_channel]),
        )

    def read(self) -> FootLoadReading:
        if self._channels is None:
            raise RuntimeError("ADS1115FootLoadReader.open() must be called before read().")

        left_voltage = self._channels[0].voltage
        right_voltage = self._channels[1].voltage
        left = self._normalize(left_voltage)
        right = self._normalize(right_voltage)
        return FootLoadReading(
            left=self.left_filter.update(left),
            right=self.right_filter.update(right),
            left_voltage=left_voltage,
            right_voltage=right_voltage,
        )

    def _normalize(self, voltage: float) -> float:
        value = max(0.0, min(1.0, float(voltage) / 3.3))
        return 1.0 - value if self.invert else value


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

    return FootLoadReading(
        left=left,
        right=right,
        left_voltage=left_voltage,
        right_voltage=right_voltage,
    )


class ESP32SerialSensorReader:
    """Read interleaved IMU (Q,...) and FSR (F,...) samples from one USB port."""

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        stale_timeout_s: float = 0.25,
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
        self.stale_timeout_s = max(0.05, stale_timeout_s)
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
        self._latest_imu: Optional[IMUReading] = None
        self._latest_imu_at = 0.0
        self._latest_fsr: Optional[FootLoadReading] = None
        self._latest_fsr_at = 0.0
        self._error: Optional[str] = None

    @property
    def error(self) -> Optional[str]:
        with self._lock:
            return self._error

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
        self._thread = threading.Thread(
            target=self._read_loop,
            name="esp32-sensors",
            daemon=True,
        )
        self._thread.start()

    def _read_loop(self) -> None:
        assert self._serial is not None
        while not self._stop.is_set():
            try:
                raw = self._serial.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace")
                now = time.monotonic()

                if self.use_imu:
                    imu = parse_serial_imu_line(
                        line,
                        roll_sign=self.imu_roll_sign,
                        pitch_sign=self.imu_pitch_sign,
                        yaw_sign=self.imu_yaw_sign,
                    )
                    if imu is not None:
                        with self._lock:
                            self._latest_imu = imu
                            self._latest_imu_at = now
                            self._error = None
                        continue

                if self.use_fsr:
                    fsr = parse_serial_fsr_line(line, invert=self.fsr_invert)
                    if fsr is not None:
                        filtered = FootLoadReading(
                            left=self.left_filter.update(fsr.left),
                            right=self.right_filter.update(fsr.right),
                            left_voltage=fsr.left_voltage,
                            right_voltage=fsr.right_voltage,
                        )
                        with self._lock:
                            self._latest_fsr = filtered
                            self._latest_fsr_at = now
                            self._error = None
            except Exception as exc:
                with self._lock:
                    self._error = str(exc)
                self._stop.wait(0.05)

    def read(self) -> SensorSnapshot:
        now = time.monotonic()
        with self._lock:
            imu = (
                self._latest_imu
                if self._latest_imu is not None and now - self._latest_imu_at <= self.stale_timeout_s
                else None
            )
            fsr = (
                self._latest_fsr
                if self._latest_fsr is not None and now - self._latest_fsr_at <= self.stale_timeout_s
                else None
            )
            return SensorSnapshot(imu=imu, foot_load=fsr)

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None
        if self._serial is not None:
            self._serial.close()
            self._serial = None


class RobotSensorHub:
    def __init__(
        self,
        transport: str = "serial",
        serial_port: str = "/dev/ttyUSB0",
        serial_baudrate: int = 115200,
        serial_timeout_s: float = 0.25,
        use_imu: bool = True,
        use_fsr: bool = True,
        imu_roll_sign: float = 1.0,
        imu_pitch_sign: float = 1.0,
        imu_yaw_sign: float = 1.0,
        fsr_ads1115_address: int = 0x48,
        fsr_left_channel: int = 0,
        fsr_right_channel: int = 1,
        fsr_invert: bool = False,
        fsr_filter_alpha: float = 0.18,
    ) -> None:
        transport = transport.lower()
        if transport not in {"serial", "i2c"}:
            raise ValueError("Sensor transport must be 'serial' or 'i2c'.")

        self.serial_reader = None
        if transport == "serial":
            self.serial_reader = ESP32SerialSensorReader(
                port=serial_port,
                baudrate=serial_baudrate,
                stale_timeout_s=serial_timeout_s,
                use_imu=use_imu,
                use_fsr=use_fsr,
                imu_roll_sign=imu_roll_sign,
                imu_pitch_sign=imu_pitch_sign,
                imu_yaw_sign=imu_yaw_sign,
                fsr_invert=fsr_invert,
                fsr_filter_alpha=fsr_filter_alpha,
            )
            self.imu = None
            self.fsr = None
        elif use_imu:
            self.imu = BNO055Reader(
                roll_sign=imu_roll_sign,
                pitch_sign=imu_pitch_sign,
                yaw_sign=imu_yaw_sign,
            )
        else:
            self.imu = None

        self.fsr = (
            ADS1115FootLoadReader(
                address=fsr_ads1115_address,
                left_channel=fsr_left_channel,
                right_channel=fsr_right_channel,
                invert=fsr_invert,
                filter_alpha=fsr_filter_alpha,
            )
            if use_fsr
            else None
        )

    def open(self) -> None:
        if self.serial_reader is not None:
            self.serial_reader.open()
        if self.imu is not None:
            self.imu.open()
        if self.fsr is not None:
            self.fsr.open()

    def read(self) -> SensorSnapshot:
        if self.serial_reader is not None:
            return self.serial_reader.read()
        imu_reading = self.imu.read() if self.imu is not None else None
        foot_load = self.fsr.read() if self.fsr is not None else None
        return SensorSnapshot(imu=imu_reading, foot_load=foot_load)

    def close(self) -> None:
        if self.serial_reader is not None:
            self.serial_reader.close()
        if self.imu is not None:
            self.imu.close()
