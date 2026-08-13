from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional


Pose = Dict[int, int]


def angle_error_deg(value: float, target: float) -> float:
    return (target - value + 180.0) % 360.0 - 180.0


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    output_limit: float
    integral_limit: float = 0.0
    derivative_alpha: float = 0.25

    def __post_init__(self) -> None:
        self.integral = 0.0
        self.prev_error: Optional[float] = None
        self.filtered_derivative = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = None
        self.filtered_derivative = 0.0

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        dt = max(0.005, min(0.10, dt))
        self.integral += error * dt
        if self.integral_limit > 0.0:
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))

        if self.prev_error is None:
            derivative = 0.0
        else:
            raw_derivative = (error - self.prev_error) / dt
            alpha = max(0.01, min(1.0, self.derivative_alpha))
            self.filtered_derivative += alpha * (raw_derivative - self.filtered_derivative)
            derivative = self.filtered_derivative
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


class RecoveryState(str, Enum):
    STABLE = "stable"
    ANKLE_HIP = "ankle-hip"
    STOMP = "stomp"
    COUNTER_LEAN = "counter-lean"
    SAFE_LOWER = "safe-lower"


@dataclass
class PushRecoveryConfig:
    warning_tilt_deg: float = 3.0
    recovery_tilt_deg: float = 5.0
    safe_lower_tilt_deg: float = 9.0
    recovery_rate_deg_s: float = 28.0
    settle_tilt_deg: float = 1.4
    recovery_step_forward_cmd: float = 0.10
    recovery_step_side_cmd: float = 0.08
    recovery_step_timeout_s: float = 3.0
    counter_lean_s: float = 0.40
    counter_lean_deg: float = 1.5


@dataclass(frozen=True)
class RecoveryDecision:
    state: RecoveryState
    reason: str
    start_step: bool = False
    forward_cmd: float = 0.0
    side_cmd: float = 0.0
    target_roll_offset_deg: float = 0.0
    target_pitch_offset_deg: float = 0.0

    @property
    def safe_lower(self) -> bool:
        return self.state is RecoveryState.SAFE_LOWER


class PushRecoveryController:
    """Safety state machine around the bounded ankle/hip stabilizer.

    It does not generate servo corrections itself.  The caller keeps the
    existing IMU PID post-IK and runs a short gait step only after this class
    confirms that both foot FSRs are present.  This keeps knees and swing
    trajectories under the walking engine rather than under IMU feedback.
    """

    def __init__(self, config: Optional[PushRecoveryConfig] = None) -> None:
        self.config = config or PushRecoveryConfig()
        self.reset()

    def reset(self) -> None:
        self.state = RecoveryState.STABLE
        self.reason = "stable"
        self._previous_roll: Optional[float] = None
        self._previous_pitch: Optional[float] = None
        self._started_at = 0.0
        self._counter_roll_deg = 0.0
        self._counter_pitch_deg = 0.0
        self._step_forward_cmd = 0.0
        self._step_side_cmd = 0.0

    def force_safe_lower(self, reason: str) -> RecoveryDecision:
        self.state = RecoveryState.SAFE_LOWER
        self.reason = reason
        self._step_forward_cmd = 0.0
        self._step_side_cmd = 0.0
        return RecoveryDecision(self.state, self.reason)

    def complete_step(self, now: float) -> RecoveryDecision:
        if self.state is RecoveryState.STOMP:
            self.state = RecoveryState.COUNTER_LEAN
            self.reason = "stomp landed; counter-lean"
            self._started_at = now
            self._step_forward_cmd = 0.0
            self._step_side_cmd = 0.0
        return self._decision()

    def update(
        self,
        roll_deg: float,
        pitch_deg: float,
        dt: float,
        left_contact: bool,
        right_contact: bool,
        single_support: bool = False,
        walking: bool = False,
        support_leg: str = "double",
        now: float = 0.0,
    ) -> RecoveryDecision:
        cfg = self.config
        dt = max(0.005, min(0.10, dt))
        roll_rate = self._rate(roll_deg, self._previous_roll, dt)
        pitch_rate = self._rate(pitch_deg, self._previous_pitch, dt)
        self._previous_roll = roll_deg
        self._previous_pitch = pitch_deg

        if self.state is RecoveryState.SAFE_LOWER:
            return self._decision()

        max_tilt = max(abs(roll_deg), abs(pitch_deg))
        max_rate = max(abs(roll_rate), abs(pitch_rate))
        support_contact = left_contact if support_leg == "left" else right_contact
        both_contact = left_contact and right_contact

        if max_tilt >= cfg.safe_lower_tilt_deg:
            return self.force_safe_lower("tilt limit")
        if single_support and not support_contact:
            return self.force_safe_lower("support FSR lost")
        if self.state is RecoveryState.STOMP:
            if support_leg in ("left", "right") and not support_contact:
                return self.force_safe_lower("support FSR lost during stomp")
            if support_leg == "double" and not both_contact:
                return self.force_safe_lower("foot FSR unavailable during stomp")
            if now > 0.0 and now - self._started_at > cfg.recovery_step_timeout_s:
                return self.force_safe_lower("stomp timeout")
            return self._decision()
        if self.state is RecoveryState.COUNTER_LEAN:
            if not both_contact:
                return self.force_safe_lower("foot FSR unavailable during counter-lean")
            if now > 0.0 and now - self._started_at >= cfg.counter_lean_s:
                self.state = RecoveryState.ANKLE_HIP
                self.reason = "settling"
                self._counter_roll_deg = 0.0
                self._counter_pitch_deg = 0.0
            return self._decision()

        if single_support and max_tilt >= cfg.recovery_tilt_deg:
            return self.force_safe_lower("single-support tilt")

        step_triggered = max_tilt >= cfg.recovery_tilt_deg and (
            max_rate >= cfg.recovery_rate_deg_s or max_tilt >= cfg.recovery_tilt_deg + 1.0
        )
        if step_triggered and walking:
            self.state = RecoveryState.ANKLE_HIP
            self.reason = "walking ankle/hip correction"
            return self._decision()
        if step_triggered and both_contact and not single_support:
            self.state = RecoveryState.STOMP
            self.reason = "stomp recovery"
            self._started_at = now
            forward_cmd, side_cmd = self._fall_command(roll_deg, pitch_deg)
            self._step_forward_cmd = forward_cmd
            self._step_side_cmd = side_cmd
            counter = abs(cfg.counter_lean_deg)
            self._counter_roll_deg = -counter if roll_deg > 0.0 else counter if roll_deg < 0.0 else 0.0
            self._counter_pitch_deg = -counter if pitch_deg > 0.0 else counter if pitch_deg < 0.0 else 0.0
            return RecoveryDecision(
                self.state,
                self.reason,
                start_step=True,
                forward_cmd=forward_cmd,
                side_cmd=side_cmd,
            )

        if max_tilt >= cfg.warning_tilt_deg:
            self.state = RecoveryState.ANKLE_HIP
            self.reason = "FSR recovery blocked" if not both_contact else "ankle/hip correction"
        elif max_tilt <= cfg.settle_tilt_deg:
            self.state = RecoveryState.STABLE
            self.reason = "stable"
        else:
            self.state = RecoveryState.ANKLE_HIP
            self.reason = "settling"
        return self._decision()

    def _decision(self) -> RecoveryDecision:
        if self.state is RecoveryState.STOMP:
            return RecoveryDecision(
                self.state,
                self.reason,
                forward_cmd=self._step_forward_cmd,
                side_cmd=self._step_side_cmd,
            )
        if self.state is RecoveryState.COUNTER_LEAN:
            return RecoveryDecision(
                self.state,
                self.reason,
                target_roll_offset_deg=self._counter_roll_deg,
                target_pitch_offset_deg=self._counter_pitch_deg,
            )
        return RecoveryDecision(self.state, self.reason)

    @staticmethod
    def _rate(value: float, previous: Optional[float], dt: float) -> float:
        if previous is None:
            return 0.0
        delta = (value - previous + 180.0) % 360.0 - 180.0
        return delta / dt

    def _fall_command(self, roll_deg: float, pitch_deg: float) -> tuple[float, float]:
        cfg = self.config
        if abs(pitch_deg) >= abs(roll_deg):
            return (cfg.recovery_step_forward_cmd if pitch_deg > 0.0 else -cfg.recovery_step_forward_cmd), 0.0
        return 0.0, (cfg.recovery_step_side_cmd if roll_deg > 0.0 else -cfg.recovery_step_side_cmd)


def lower_toward_standing(
    pose: Pose,
    standing: Pose,
    dt: float,
    max_rate_pwm_s: float,
) -> Pose:
    """Return a bounded transition from the current pose back to standing."""
    max_delta = max(1, round(max_rate_pwm_s * max(0.005, min(0.10, dt))))
    return {
        servo_id: current + max(-max_delta, min(max_delta, standing[servo_id] - current))
        for servo_id, current in pose.items()
        if servo_id in standing
    }


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
        17: +1,   # R ankle roll
        18: -1,   # R ankle pitch
        20: -1,   # R hip pitch
        21: +1,   # R hip roll/abduct
        12: -1,   # L hip roll/abduct
        13: +1,   # L hip pitch
        15: +1,   # L ankle pitch
        16: -1,   # L ankle roll
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
        target_roll_offset_deg: float = 0.0,
        target_pitch_offset_deg: float = 0.0,
    ) -> Pose:
        cfg = self.config
        roll_error = angle_error_deg(roll_deg, cfg.target_roll_deg + target_roll_offset_deg)
        pitch_error = angle_error_deg(pitch_deg, cfg.target_pitch_deg + target_pitch_offset_deg)

        if abs(roll_error) < cfg.roll_deadband_deg:
            roll_error = 0.0
            self.roll_pid.reset()
        if abs(pitch_error) < cfg.pitch_deadband_deg:
            pitch_error = 0.0
            self.pitch_pid.reset()

        roll_corr = self.roll_pid.update(roll_error, dt)
        pitch_corr = self.pitch_pid.update(pitch_error, dt)

        left_w, right_w = self._support_weights(support_leg)
        corrected = dict(pose)

        self._add_joint_deg(corrected, 18, right_w * cfg.pitch_ankle_gain * pitch_corr)
        self._add_joint_deg(corrected, 15, left_w * cfg.pitch_ankle_gain * pitch_corr)
        self._add_joint_deg(corrected, 20, right_w * cfg.pitch_hip_gain * pitch_corr)
        self._add_joint_deg(corrected, 13, left_w * cfg.pitch_hip_gain * pitch_corr)

        self._add_joint_deg(corrected, 17, right_w * cfg.roll_ankle_gain * roll_corr)
        self._add_joint_deg(corrected, 16, left_w * cfg.roll_ankle_gain * roll_corr)
        self._add_joint_deg(corrected, 21, right_w * cfg.roll_hip_gain * roll_corr)
        self._add_joint_deg(corrected, 12, left_w * cfg.roll_hip_gain * roll_corr)

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
