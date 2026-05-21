from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional


Pose = Dict[int, int]


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    output_limit: float
    integral_limit: float = 0.0

    def __post_init__(self) -> None:
        self.integral = 0.0
        self.prev_error: Optional[float] = None

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = None

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        self.integral += error * dt
        if self.integral_limit > 0.0:
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))

        if self.prev_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.prev_error) / dt
        self.prev_error = error

        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, out))


@dataclass
class BalanceConfig:
    target_roll_deg: float = 0.0
    target_pitch_deg: float = 0.0
    roll_deadband_deg: float = 0.4
    pitch_deadband_deg: float = 0.4
    max_correction_deg: float = 6.0
    pwm_per_deg: float = 2000.0 / 180.0

    pitch_ankle_gain: float = 0.75
    pitch_hip_gain: float = 0.30
    roll_ankle_gain: float = 0.70
    roll_hip_gain: float = 0.25

    swing_leg_gain: float = 0.35
    double_support_gain: float = 0.70


class IMUBalanceController:
    """
    PID stabilizer that adds small closed-loop corrections to ankle and hip servos.

    Input convention:
      +roll_deg  = robot leans left
      +pitch_deg = robot leans forward

    The sign of each axis depends on how the BNO055 is mounted. main.py exposes
    CLI signs so this module can stay deterministic and hardware-independent.
    """

    DIR = {
        1: +1,    # R ankle roll
        2: -1,    # R ankle pitch
        4: -1,    # R hip pitch
        5: +1,    # R hip roll/abduct
        20: -1,   # L hip roll/abduct
        21: +1,   # L hip pitch
        23: +1,   # L ankle pitch
        24: -1,   # L ankle roll
    }

    def __init__(self, config: Optional[BalanceConfig] = None) -> None:
        self.config = config or BalanceConfig()
        limit = self.config.max_correction_deg
        self.roll_pid = PID(kp=0.45, ki=0.0, kd=0.025, output_limit=limit)
        self.pitch_pid = PID(kp=0.45, ki=0.0, kd=0.025, output_limit=limit)

    def reset(self) -> None:
        self.roll_pid.reset()
        self.pitch_pid.reset()

    def apply(
        self,
        pose: Pose,
        roll_deg: float,
        pitch_deg: float,
        dt: float,
        support_leg: str = "double",
    ) -> Pose:
        cfg = self.config
        roll_error = cfg.target_roll_deg - roll_deg
        pitch_error = cfg.target_pitch_deg - pitch_deg

        if abs(roll_error) < cfg.roll_deadband_deg:
            roll_error = 0.0
        if abs(pitch_error) < cfg.pitch_deadband_deg:
            pitch_error = 0.0

        roll_corr = self.roll_pid.update(roll_error, dt)
        pitch_corr = self.pitch_pid.update(pitch_error, dt)

        left_w, right_w = self._support_weights(support_leg)
        corrected = dict(pose)

        self._add_joint_deg(corrected, 2, right_w * cfg.pitch_ankle_gain * pitch_corr)
        self._add_joint_deg(corrected, 23, left_w * cfg.pitch_ankle_gain * pitch_corr)
        self._add_joint_deg(corrected, 4, right_w * cfg.pitch_hip_gain * pitch_corr)
        self._add_joint_deg(corrected, 21, left_w * cfg.pitch_hip_gain * pitch_corr)

        self._add_joint_deg(corrected, 1, right_w * cfg.roll_ankle_gain * roll_corr)
        self._add_joint_deg(corrected, 24, left_w * cfg.roll_ankle_gain * roll_corr)
        self._add_joint_deg(corrected, 5, right_w * cfg.roll_hip_gain * roll_corr)
        self._add_joint_deg(corrected, 20, left_w * cfg.roll_hip_gain * roll_corr)

        return corrected

    def _support_weights(self, support_leg: str) -> tuple[float, float]:
        cfg = self.config
        if support_leg == "left":
            return 1.0, cfg.swing_leg_gain
        if support_leg == "right":
            return cfg.swing_leg_gain, 1.0
        return cfg.double_support_gain, cfg.double_support_gain

    def _add_joint_deg(self, pose: Pose, servo_id: int, delta_deg: float) -> None:
        if servo_id not in pose:
            return
        delta_pwm = round(self.DIR[servo_id] * delta_deg * self.config.pwm_per_deg)
        pose[servo_id] = max(500, min(2500, pose[servo_id] + delta_pwm))
