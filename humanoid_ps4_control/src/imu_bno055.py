from __future__ import annotations

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
