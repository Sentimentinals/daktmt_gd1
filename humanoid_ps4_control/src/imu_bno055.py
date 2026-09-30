from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class IMUReading:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    sensor_time_ms: int = 0
    system_cal: int = 0
    gyro_cal: int = 0
    accel_cal: int = 0
    mag_cal: int = 0
    gravity_x: Optional[float] = None
    gravity_y: Optional[float] = None
    gravity_z: Optional[float] = None

    def balance_ready(self, min_gyro_cal: int = 1, min_accel_cal: int = 1) -> bool:
        return self.gyro_cal >= min_gyro_cal and self.accel_cal >= min_accel_cal


def parse_serial_imu_line(
    line: str,
    roll_sign: float = 1.0,
    pitch_sign: float = 1.0,
    yaw_sign: float = 1.0,
) -> Optional[IMUReading]:
    fields = line.strip().split(",")
    if len(fields) not in {13, 16} or fields[0] != "Q":
        return None

    try:
        quaternion = [float(value) for value in fields[2:6]]
        yaw, roll, pitch = (
            float(fields[6]) * yaw_sign,
            float(fields[7]) * roll_sign,
            float(fields[8]) * pitch_sign,
        )
        gravity = [float(value) for value in fields[13:]]
        if not all(math.isfinite(value) for value in (*quaternion, yaw, roll, pitch, *gravity)):
            return None
        quaternion_norm_sq = sum(value * value for value in quaternion)
        calibration = [int(value) for value in fields[9:13]]
        if not 0.25 <= quaternion_norm_sq <= 2.25 or any(value not in range(4) for value in calibration):
            return None
        return IMUReading(
            roll_deg=roll,
            pitch_deg=pitch,
            yaw_deg=yaw,
            sensor_time_ms=int(fields[1]),
            system_cal=calibration[0],
            gyro_cal=calibration[1],
            accel_cal=calibration[2],
            mag_cal=calibration[3],
            gravity_x=gravity[0] if gravity else None,
            gravity_y=gravity[1] if gravity else None,
            gravity_z=gravity[2] if gravity else None,
        )
    except (TypeError, ValueError):
        return None
