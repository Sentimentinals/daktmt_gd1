from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class IMUReading:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


class BNO055Reader:
    """
    Small wrapper around Adafruit's BNO055 CircuitPython driver.

    Dependencies on Raspberry Pi:
      pip install adafruit-circuitpython-bno055

    The axis signs are intentionally configurable because the physical IMU
    mounting direction is robot-specific.
    """

    def __init__(
        self,
        roll_sign: float = 1.0,
        pitch_sign: float = 1.0,
        yaw_sign: float = 1.0,
    ) -> None:
        self.roll_sign = roll_sign
        self.pitch_sign = pitch_sign
        self.yaw_sign = yaw_sign
        self._sensor = None

    def open(self) -> None:
        try:
            import board  # type: ignore
            import busio  # type: ignore
            import adafruit_bno055  # type: ignore
        except ImportError as exc:
            raise ImportError(
                "BNO055 support requires: pip install adafruit-circuitpython-bno055"
            ) from exc

        i2c = busio.I2C(board.SCL, board.SDA)
        self._sensor = adafruit_bno055.BNO055_I2C(i2c)

    def read(self) -> Optional[IMUReading]:
        if self._sensor is None:
            raise RuntimeError("BNO055Reader.open() must be called before read().")

        euler = self._sensor.euler
        if euler is None or any(v is None for v in euler):
            return None

        yaw, roll, pitch = euler
        return IMUReading(
            roll_deg=float(roll) * self.roll_sign,
            pitch_deg=float(pitch) * self.pitch_sign,
            yaw_deg=float(yaw) * self.yaw_sign,
        )

    def close(self) -> None:
        self._sensor = None


def parse_serial_imu_line(
    line: str,
    roll_sign: float = 1.0,
    pitch_sign: float = 1.0,
    yaw_sign: float = 1.0,
) -> Optional[IMUReading]:
    fields = line.strip().split(",")
    if len(fields) != 13 or fields[0] != "Q":
        return None

    try:
        quaternion = [float(value) for value in fields[2:6]]
        if sum(value * value for value in quaternion) < 0.25:
            return None
        return IMUReading(
            roll_deg=float(fields[7]) * roll_sign,
            pitch_deg=float(fields[8]) * pitch_sign,
            yaw_deg=float(fields[6]) * yaw_sign,
        )
    except ValueError:
        return None


class SerialBNO055Reader:
    """Read the latest BNO055 sample produced by the ESP32 USB firmware."""

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        stale_timeout_s: float = 0.25,
        roll_sign: float = 1.0,
        pitch_sign: float = 1.0,
        yaw_sign: float = 1.0,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.stale_timeout_s = max(0.05, stale_timeout_s)
        self.roll_sign = roll_sign
        self.pitch_sign = pitch_sign
        self.yaw_sign = yaw_sign
        self._serial = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: Optional[IMUReading] = None
        self._latest_at = 0.0
        self._error: Optional[str] = None

    @property
    def error(self) -> Optional[str]:
        with self._lock:
            return self._error

    def open(self) -> None:
        try:
            import serial
        except ImportError as exc:
            raise ImportError(
                "ESP32 IMU support requires: pip install pyserial"
            ) from exc

        try:
            self._serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.05,
            )
        except Exception as exc:
            raise RuntimeError(
                f"Cannot open ESP32 sensor port {self.port}: {exc}"
            ) from exc

        self._stop.clear()
        self._thread = threading.Thread(
            target=self._read_loop,
            name="esp32-bno055",
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
                reading = parse_serial_imu_line(
                    raw.decode("ascii", errors="replace"),
                    roll_sign=self.roll_sign,
                    pitch_sign=self.pitch_sign,
                    yaw_sign=self.yaw_sign,
                )
                if reading is not None:
                    with self._lock:
                        self._latest = reading
                        self._latest_at = time.monotonic()
                        self._error = None
            except Exception as exc:
                with self._lock:
                    self._error = str(exc)
                self._stop.wait(0.05)

    def read(self) -> Optional[IMUReading]:
        with self._lock:
            if (
                self._latest is None
                or time.monotonic() - self._latest_at > self.stale_timeout_s
            ):
                return None
            return self._latest

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None
        if self._serial is not None:
            self._serial.close()
            self._serial = None
