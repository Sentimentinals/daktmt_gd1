from __future__ import annotations

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

    def balance_ready(self, min_gyro_cal: int = 1, min_accel_cal: int = 1) -> bool:
        return self.gyro_cal >= min_gyro_cal and self.accel_cal >= min_accel_cal


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
        quaternion_norm_sq = sum(value * value for value in quaternion)
        calibration = [int(value) for value in fields[9:13]]
        if not 0.25 <= quaternion_norm_sq <= 2.25 or any(value not in range(4) for value in calibration):
            return None
        return IMUReading(
            roll_deg=float(fields[7]) * roll_sign,
            pitch_deg=float(fields[8]) * pitch_sign,
            yaw_deg=float(fields[6]) * yaw_sign,
            sensor_time_ms=int(fields[1]),
            system_cal=calibration[0],
            gyro_cal=calibration[1],
            accel_cal=calibration[2],
            mag_cal=calibration[3],
        )
    except (TypeError, ValueError):
        return None
