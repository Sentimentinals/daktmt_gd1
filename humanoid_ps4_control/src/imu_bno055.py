from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class IMUReading:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


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
