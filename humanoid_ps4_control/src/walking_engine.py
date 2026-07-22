from __future__ import annotations

import math
from collections import deque
from typing import Deque

import numpy as np

from .leg_ik import leg_ik
from .zmp_controller import ZMPPreviewController


from .config import (
    DIR,
    GAIT,
    PWM_PER_DEG,
    ROBOT,
    STAND_ANG,
    STANDING,
)


def angle_to_pwm(sid: int, base_ang: float, new_ang: float, base_pwm: int) -> int:
    delta = DIR.get(sid, 1) * (new_ang - base_ang) * PWM_PER_DEG
    return max(500, min(2500, round(base_pwm + delta)))


def clamp_pose_rate(prev: dict[int, int], curr: dict[int, int], max_pwm_per_frame: float) -> dict[int, int]:
    """Limit leg servo pulse changes per frame to reduce shock load."""
    out = dict(curr)
    for sid in DIR:
        if sid in prev and sid in curr:
            delta = curr[sid] - prev[sid]
            if abs(delta) > max_pwm_per_frame:
                out[sid] = prev[sid] + int(math.copysign(max_pwm_per_frame, delta))
    return out


class CubicBSplinePoseFilter:
    """Causal cubic B-spline sampling with one control-frame delay."""

    def __init__(self, initial_pose: dict[int, int], strength: float = 1.0) -> None:
        self.strength = max(0.0, min(1.0, strength))
        self.reset(initial_pose)

    def reset(self, pose: dict[int, int]) -> None:
        start = dict(pose)
        self._controls: Deque[dict[int, int]] = deque((dict(start), dict(start)), maxlen=3)

    def update(self, target: dict[int, int]) -> dict[int, int]:
        current = dict(target)
        self._controls.append(current)
        if self.strength <= 0.0:
            return current

        previous, center, future = self._controls
        out = dict(current)
        for sid, value in current.items():
            if sid not in previous or sid not in center or sid not in future:
                continue
            # Uniform cubic B-spline evaluated at a knot: (P0 + 4*P1 + P2) / 6.
            spline_value = (previous[sid] + 4.0 * center[sid] + future[sid]) / 6.0
            out[sid] = round(value + self.strength * (spline_value - value))
        return out


def blend_pwm(start: int, end: int, t: float) -> int:
    t = max(0.0, min(1.0, t))
    return round(start + (end - start) * t)


def lift_pitch_deltas(lift_height: float, forward_x: float = 0.0) -> tuple[int, int, int]:
    if lift_height <= 0.0:
        return 0, 0, 0

    hip = (0.0, 0.0, ROBOT["com_height"])
    foot_ground = (0.0, 0.0, 0.0)
    foot_lifted = (forward_x, 0.0, lift_height)
    neutral = leg_ik(hip, foot_ground, ROBOT["upper_leg"], ROBOT["lower_leg"])
    lifted = leg_ik(hip, foot_lifted, ROBOT["upper_leg"], ROBOT["lower_leg"])

    raw_thigh_delta = round((lifted["hip_pitch"] - neutral["hip_pitch"]) * PWM_PER_DEG)
    raw_knee_delta = round((lifted["knee"] - neutral["knee"]) * PWM_PER_DEG)
    raw_ankle_delta = round((lifted["ankle_pitch"] - neutral["ankle_pitch"]) * PWM_PER_DEG)
    thigh_scale = 0.68
    lift_scale = 0.60
    thigh_delta = round(max(-300, min(300, raw_thigh_delta * thigh_scale)))
    knee_delta = round(max(0, min(280, raw_knee_delta * lift_scale)))
    ankle_delta = round(max(-120, min(120, raw_ankle_delta * thigh_scale)))
    return thigh_delta, knee_delta, ankle_delta


def compute_pose(
    com_x: float,
    com_y: float,
    foot_L: np.ndarray,
    foot_R: np.ndarray,
    com_z: float | None = None,
    support_leg: str = "double",
    phase_mode: str = "full",
    zmp_support_ratio: float | None = None,
    hip_abduct_gain: float | None = None,
    swing_hip_roll_scale: float | None = None,
    ankle_roll_gain: float | None = None,
    swing_ankle_roll_scale: float | None = None,
) -> dict[int, int]:
    """Convert CoM/foot targets into a full servo pulse pose."""
    body_z = ROBOT["com_height"] if com_z is None else com_z
    roll_height = ROBOT["com_height"]
    hw = ROBOT["half_hip"]
    L1 = ROBOT["upper_leg"]
    L2 = ROBOT["lower_leg"]

    hip_L = (com_x, com_y - hw, body_z)
    hip_R = (com_x, com_y + hw, body_z)

    ik_R = leg_ik(hip_R, tuple(foot_R), L1, L2)
    ik_L = leg_ik(hip_L, tuple(foot_L), L1, L2)

    hip_gain = GAIT["hip_abduct_gain"] if hip_abduct_gain is None else hip_abduct_gain
    swing_hip_scale = GAIT["swing_hip_roll_scale"] if swing_hip_roll_scale is None else swing_hip_roll_scale
    ankle_gain = GAIT["ankle_roll_gain"] if ankle_roll_gain is None else ankle_roll_gain
    swing_roll_scale = GAIT["swing_ankle_roll_scale"] if swing_ankle_roll_scale is None else swing_ankle_roll_scale
    ankle_roll = math.degrees(math.atan2(com_y, roll_height)) * ankle_gain

    if support_leg == "right":
        right_hip_abduct = -abs(ik_R["hip_abduct"]) * hip_gain
        left_hip_abduct = ik_L["hip_abduct"] * hip_gain * swing_hip_scale
        right_ankle_roll = ankle_roll + math.copysign(2.0, ankle_roll) if abs(ankle_roll) > 0.01 else ankle_roll
        left_ankle_roll = ankle_roll * swing_roll_scale
    elif support_leg == "left":
        right_hip_abduct = ik_R["hip_abduct"] * hip_gain * swing_hip_scale
        left_hip_abduct = -abs(ik_L["hip_abduct"]) * hip_gain
        right_ankle_roll = ankle_roll * swing_roll_scale
        left_ankle_roll = ankle_roll + math.copysign(2.0, ankle_roll) if abs(ankle_roll) > 0.01 else ankle_roll
    else:
        right_hip_abduct = ik_R["hip_abduct"] * hip_gain * 0.25
        left_hip_abduct = ik_L["hip_abduct"] * hip_gain * 0.25
        right_ankle_roll = ankle_roll * 0.5
        left_ankle_roll = ankle_roll * 0.5

    pose = dict(STANDING)
    pose[1] = angle_to_pwm(1, STAND_ANG["hip_roll"], right_ankle_roll, STANDING[1])
    pose[2] = angle_to_pwm(2, STAND_ANG["R_ankle"], ik_R["ankle_pitch"], STANDING[2])
    pose[3] = angle_to_pwm(3, STAND_ANG["R_knee"], ik_R["knee"], STANDING[3])
    pose[4] = angle_to_pwm(4, STAND_ANG["R_hip_pitch"], ik_R["hip_pitch"], STANDING[4])
    pose[5] = angle_to_pwm(5, STAND_ANG["R_hip_abduct"], right_hip_abduct, STANDING[5])

    pose[20] = angle_to_pwm(20, STAND_ANG["L_hip_abduct"], left_hip_abduct, STANDING[20])
    pose[21] = angle_to_pwm(21, STAND_ANG["L_hip_pitch"], ik_L["hip_pitch"], STANDING[21])
    pose[22] = angle_to_pwm(22, STAND_ANG["L_knee"], ik_L["knee"], STANDING[22])
    pose[23] = angle_to_pwm(23, STAND_ANG["L_ankle"], ik_L["ankle_pitch"], STANDING[23])
    pose[24] = angle_to_pwm(24, STAND_ANG["hip_roll"], left_ankle_roll, STANDING[24])
    if phase_mode == "shift":
        support_y = hw * (GAIT["zmp_support_ratio"] if zmp_support_ratio is None else zmp_support_ratio)
        signed_support_y = support_y if support_leg == "right" else -support_y
        raw_roll = math.degrees(math.atan2(signed_support_y, roll_height))
        shift_ankle_roll = raw_roll * ankle_gain
        if abs(shift_ankle_roll) > 0.01:
            shift_ankle_roll += math.copysign(2.0, shift_ankle_roll)
        if support_leg == "right":
            pose[1] = angle_to_pwm(1, STAND_ANG["hip_roll"], shift_ankle_roll, STANDING[1])
            pose[5] = STANDING[5]
        elif support_leg == "left":
            pose[24] = angle_to_pwm(24, STAND_ANG["hip_roll"], -shift_ankle_roll, STANDING[24])
            pose[20] = STANDING[20]
    return pose


class SingleSupportTestEngine:
    def __init__(
        self,
        dt: float = 0.04,
        support_leg: str = "right",
        lift_height: float = 28.0,
        zmp_support_ratio: float | None = None,
        hip_abduct_gain: float | None = None,
        ankle_roll_gain: float | None = None,
        arm_pwm: int = 180,
        ramp_s: float = 0.8,
    ) -> None:
        self.dt = dt
        self.support_leg = support_leg
        self.lift_height = lift_height
        self.zmp_support_ratio = GAIT["zmp_support_ratio"] if zmp_support_ratio is None else zmp_support_ratio
        self.hip_abduct_gain = GAIT["hip_abduct_gain"] if hip_abduct_gain is None else hip_abduct_gain
        self.ankle_roll_gain = GAIT["ankle_roll_gain"] if ankle_roll_gain is None else ankle_roll_gain
        self.arm_pwm = arm_pwm
        self.ramp_s = max(dt, ramp_s)
        self.running = False
        self.phase = 0.0
        self.prev_pose = dict(STANDING)

    def start(self, support_leg: str | None = None, current_pose: dict[int, int] | None = None) -> None:
        if support_leg is not None:
            self.support_leg = support_leg
        self.running = True
        self.phase = 0.0
        self.prev_pose = dict(STANDING if current_pose is None else current_pose)

    def stop(self) -> None:
        self.running = False
        self.phase = 0.0

    def update(self) -> dict[int, int]:
        if not self.running:
            self.prev_pose = dict(STANDING)
            return dict(STANDING)

        self.phase = min(1.0, self.phase + self.dt / self.ramp_s)
        blend = self.phase * self.phase * (3.0 - 2.0 * self.phase)
        hw = ROBOT["half_hip"]
        support_y = hw * self.zmp_support_ratio
        com_y = support_y if self.support_leg == "right" else -support_y
        com_x = 8.0 * blend

        foot_l = np.array([0.0, -hw, 0.0])
        foot_r = np.array([0.0, hw, 0.0])
        if self.support_leg == "right":
            foot_l[2] = self.lift_height * blend
        else:
            foot_r[2] = self.lift_height * blend

        target = compute_pose(
            com_x,
            com_y * blend,
            foot_l,
            foot_r,
            support_leg=self.support_leg,
            phase_mode="shift",
            zmp_support_ratio=self.zmp_support_ratio,
            hip_abduct_gain=self.hip_abduct_gain,
            ankle_roll_gain=self.ankle_roll_gain,
        )
        lift_shape_height = min(self.lift_height * blend, ROBOT["step_height"])
        thigh_delta, knee_delta, ankle_delta = lift_pitch_deltas(lift_shape_height)
        if self.support_leg == "right":
            target[21] = max(500, min(2500, STANDING[21] + thigh_delta))
            target[22] = max(500, min(2500, STANDING[22] + knee_delta))
            target[23] = max(500, min(2500, STANDING[23] + ankle_delta))
        else:
            target[4] = max(500, min(2500, STANDING[4] - thigh_delta))
            target[3] = max(500, min(2500, STANDING[3] - knee_delta))
            target[2] = max(500, min(2500, STANDING[2] - ankle_delta))
        arm_delta = round(self.arm_pwm * blend)
        if self.support_leg == "right":
            target[8] = max(500, min(2500, STANDING[8] + arm_delta))
            target[17] = max(500, min(2500, STANDING[17] + arm_delta))
        else:
            target[8] = max(500, min(2500, STANDING[8] - arm_delta))
            target[17] = max(500, min(2500, STANDING[17] - arm_delta))
        pose = {sid: blend_pwm(self.prev_pose.get(sid, STANDING[sid]), target[sid], 0.35) for sid in STANDING}
        self.prev_pose = pose
        return pose


class DynamicWalkingEngine:
    def __init__(
        self,
        dt: float = 0.04,
        t_step: float = 1.55,
        t_dbl: float = 0.04,
        max_step_len: float = 28.0,
        max_turn_step_len: float | None = None,
        max_side_step_len: float | None = None,
        step_height: float | None = None,
        zmp_support_ratio: float | None = None,
        hip_abduct_gain: float | None = None,
        swing_hip_roll_scale: float | None = None,
        ankle_roll_gain: float | None = None,
        swing_ankle_roll_scale: float | None = None,
        step_x_ratio: float | None = None,
        thigh_lift_forward_mm: float | None = None,
        left_swing_x_scale: float | None = None,
        left_step_height_scale: float | None = None,
        landing_gap_mm: float | None = None,
        right_swing_x_scale: float | None = None,
        right_step_height_scale: float | None = None,
        lift_start_phase: float | None = None,
        swing_advance_end_phase: float | None = None,
        lift_end_phase: float | None = None,
        landing_roll_release_start: float | None = None,
        command_deadzone: float | None = None,
        command_rate_limit: float = 16.0,
        arm_swing_pwm: int | None = None,
        arm_right_dir: int | None = None,
        arm_left_dir: int | None = None,
        arm_elbow_ratio: float | None = None,
        arm_lift_ratio: float | None = None,
        arm_smooth_tau: float | None = None,
        arm_min_pwm: int | None = None,
        arm_quantum_pwm: int | None = None,
        max_step_elevation: float = 18.0,
        trajectory_smoothing: float = 1.0,
    ) -> None:
        self.dt = dt
        self.t_step = t_step
        self.t_dbl = t_dbl
        self.t_single = self.t_step - self.t_dbl

        self.n_s = max(1, round(self.t_single / dt))
        self.n_d = max(1, round(self.t_dbl / dt))

        self.zc = ROBOT["com_height"]
        self.hw = ROBOT["half_hip"]
        self.step_height = ROBOT["step_height"] if step_height is None else step_height
        self.zmp_support_ratio = GAIT["zmp_support_ratio"] if zmp_support_ratio is None else zmp_support_ratio
        self.hip_abduct_gain = GAIT["hip_abduct_gain"] if hip_abduct_gain is None else hip_abduct_gain
        self.swing_hip_roll_scale = (
            GAIT["swing_hip_roll_scale"] if swing_hip_roll_scale is None else swing_hip_roll_scale
        )
        self.ankle_roll_gain = GAIT["ankle_roll_gain"] if ankle_roll_gain is None else ankle_roll_gain
        self.swing_ankle_roll_scale = GAIT["swing_ankle_roll_scale"] if swing_ankle_roll_scale is None else swing_ankle_roll_scale
        self.step_x_ratio = GAIT["step_x_ratio"] if step_x_ratio is None else step_x_ratio
        self.thigh_lift_forward_mm = (
            GAIT["thigh_lift_forward_mm"] if thigh_lift_forward_mm is None else thigh_lift_forward_mm
        )
        self.left_swing_x_scale = GAIT["left_swing_x_scale"] if left_swing_x_scale is None else left_swing_x_scale
        self.left_step_height_scale = (
            GAIT["left_step_height_scale"] if left_step_height_scale is None else left_step_height_scale
        )
        self.landing_gap_mm = abs(GAIT["landing_gap_mm"] if landing_gap_mm is None else landing_gap_mm)
        self.right_swing_x_scale = GAIT["right_swing_x_scale"] if right_swing_x_scale is None else right_swing_x_scale
        self.right_step_height_scale = (
            GAIT["right_step_height_scale"] if right_step_height_scale is None else right_step_height_scale
        )
        self.lift_start_phase = GAIT["lift_start_phase"] if lift_start_phase is None else lift_start_phase
        self.swing_advance_end_phase = (
            GAIT["swing_advance_end_phase"] if swing_advance_end_phase is None else swing_advance_end_phase
        )
        self.lift_end_phase = GAIT["lift_end_phase"] if lift_end_phase is None else lift_end_phase
        self.landing_roll_release_start = (
            GAIT["landing_roll_release_start"]
            if landing_roll_release_start is None
            else landing_roll_release_start
        )
        self.lift_start_phase = max(0.0, min(0.30, self.lift_start_phase))
        self.lift_end_phase = max(self.lift_start_phase + 0.20, min(1.0, self.lift_end_phase))
        self.swing_advance_end_phase = max(
            self.lift_start_phase + 0.10,
            min(self.lift_end_phase - 0.05, self.swing_advance_end_phase),
        )
        self.landing_roll_release_start = max(0.0, min(0.95, self.landing_roll_release_start))
        self.command_deadzone = GAIT["command_deadzone"] if command_deadzone is None else command_deadzone
        self.arm_swing_pwm = int(GAIT["arm_swing_pwm"] if arm_swing_pwm is None else arm_swing_pwm)
        self.arm_right_dir = int(GAIT["arm_right_dir"] if arm_right_dir is None else arm_right_dir)
        self.arm_left_dir = int(GAIT["arm_left_dir"] if arm_left_dir is None else arm_left_dir)
        self.arm_elbow_ratio = GAIT["arm_elbow_ratio"] if arm_elbow_ratio is None else arm_elbow_ratio
        self.arm_lift_ratio = GAIT["arm_lift_ratio"] if arm_lift_ratio is None else arm_lift_ratio
        self.arm_smooth_tau = GAIT["arm_smooth_tau"] if arm_smooth_tau is None else arm_smooth_tau
        self.arm_min_pwm = int(GAIT["arm_min_pwm"] if arm_min_pwm is None else arm_min_pwm)
        self.arm_quantum_pwm = max(1, int(GAIT["arm_quantum_pwm"] if arm_quantum_pwm is None else arm_quantum_pwm))
        self.preview_steps = 24

        self.zmp_ctrl = ZMPPreviewController(dt=dt, zc=self.zc, preview_steps=self.preview_steps)
        self.zmp_ctrl_x = ZMPPreviewController(dt=dt, zc=self.zc, preview_steps=self.preview_steps)

        self.max_pwm_per_frame = (1150.0 * dt) * PWM_PER_DEG
        self.max_step_len = max_step_len
        self.max_turn_step_len = GAIT["max_turn_step_len"] if max_turn_step_len is None else max_turn_step_len
        self.max_side_step_len = GAIT["max_side_step_len"] if max_side_step_len is None else max_side_step_len
        self.command_rate_limit = abs(command_rate_limit)
        self.max_step_elevation = max(0.0, abs(max_step_elevation))
        self.trajectory_smoothing = max(0.0, min(1.0, trajectory_smoothing))
        self.stop_extra_steps = max(0, int(GAIT["stop_extra_steps"]))

        self.reset()

    def reset(self) -> None:
        self.zmp_ctrl.reset()
        self.zmp_ctrl_x.reset()
        self.step_count = 0
        self.zmp_y_queue: Deque[float] = deque()
        self.zmp_x_queue: Deque[float] = deque()
        self.zmp_z_queue: Deque[float] = deque()
        self.foot_L_queue: Deque[np.ndarray] = deque()
        self.foot_R_queue: Deque[np.ndarray] = deque()
        self.arm_queue: Deque[tuple[int, int]] = deque()
        self.swing_leg_queue: Deque[str] = deque()
        self.lift_factor_queue: Deque[float] = deque()
        self.landing_progress_queue: Deque[float] = deque()
        self.phase_mode_queue: Deque[str] = deque()
        self.side_len_queue: Deque[float] = deque()
        self.step_elevation_queue: Deque[float] = deque()
        self.support_leg = "double"
        self.commanded_step_len = 0.0
        self.commanded_turn_len = 0.0
        self.commanded_side_len = 0.0
        self.last_foot_L = np.array([0.0, -self.hw, 0.0])
        self.last_foot_R = np.array([0.0, self.hw, 0.0])
        self.last_arm_delta = (0, 0)
        self.last_arm_role = "neutral"
        self.last_swing_leg = "none"
        self.last_lift_factor = 0.0
        self.last_landing_progress = 0.0
        self.last_phase_mode = "idle"
        self.last_step_elevation = 0.0
        self._arm_state = [0.0, 0.0]
        self._com_y = 0.0
        self._com_x = 0.0
        self._com_z = 0.0
        self._last_motion_target = (0.0, 0.0, 0.0)
        self._stop_steps_remaining = 0
        self._stop_decelerating = False
        self.pose_filter = CubicBSplinePoseFilter(STANDING, self.trajectory_smoothing)

        for _ in range(self.n_d):
            self.zmp_y_queue.append(0.0)
            self.zmp_x_queue.append(0.0)
            self.zmp_z_queue.append(0.0)
            self.foot_L_queue.append(np.array([0.0, -self.hw, 0.0]))
            self.foot_R_queue.append(np.array([0.0, self.hw, 0.0]))
            self.arm_queue.append((0, 0))
            self.swing_leg_queue.append("none")
            self.lift_factor_queue.append(0.0)
            self.landing_progress_queue.append(0.0)
            self.phase_mode_queue.append("idle")
            self.side_len_queue.append(0.0)
            self.step_elevation_queue.append(0.0)

        self.prev_pose = dict(STANDING)

    def is_idle_ready(self, tolerance: float = 0.05) -> bool:
        if (
            abs(self.commanded_step_len) > tolerance
            or abs(self.commanded_turn_len) > tolerance
            or abs(self.commanded_side_len) > tolerance
        ):
            return False

        neutral_l = np.array([0.0, -self.hw, 0.0])
        neutral_r = np.array([0.0, self.hw, 0.0])
        if any(abs(zmp_y) > tolerance for zmp_y in self.zmp_y_queue):
            return False
        if any(np.linalg.norm(foot - neutral_l) > tolerance for foot in self.foot_L_queue):
            return False
        if any(np.linalg.norm(foot - neutral_r) > tolerance for foot in self.foot_R_queue):
            return False
        if any(delta != (0, 0) for delta in self.arm_queue):
            return False
        if any(swing_leg != "none" for swing_leg in self.swing_leg_queue):
            return False
        if any(lift_factor > tolerance for lift_factor in self.lift_factor_queue):
            return False
        if any(landing_progress > tolerance for landing_progress in self.landing_progress_queue):
            return False
        if any(phase_mode != "idle" for phase_mode in self.phase_mode_queue):
            return False
        if any(abs(side_len) > tolerance for side_len in self.side_len_queue):
            return False
        arms_settled = abs(self._arm_state[0]) < self.arm_min_pwm and abs(self._arm_state[1]) < self.arm_min_pwm
        return arms_settled and self.zmp_ctrl.is_settled()

    def _enqueue_next_step(
        self,
        step_len: float,
        turn_len: float = 0.0,
        side_len: float = 0.0,
        step_elevation: float = 0.0,
    ) -> None:
        base_L = self.foot_L_queue[-1].copy() if self.foot_L_queue else self.last_foot_L.copy()
        base_R = self.foot_R_queue[-1].copy() if self.foot_R_queue else self.last_foot_R.copy()
        step_elevation = max(-self.max_step_elevation, min(self.max_step_elevation, step_elevation))

        if abs(step_len) < 0.1 and abs(turn_len) < 0.1 and abs(side_len) < 0.1:
            settle_frames = self.n_s + self.n_d
            stance_center_x = (base_L[0] + base_R[0]) / 2.0
            stance_center_z = (base_L[2] + base_R[2]) / 2.0
            for _ in range(settle_frames):
                self.zmp_x_queue.append(stance_center_x)
                self.zmp_y_queue.append(0.0)
                self.zmp_z_queue.append(stance_center_z)
                self.foot_L_queue.append(base_L.copy())
                self.foot_R_queue.append(base_R.copy())
                self.arm_queue.append((0, 0))
                self.swing_leg_queue.append("none")
                self.lift_factor_queue.append(0.0)
                self.landing_progress_queue.append(0.0)
                self.phase_mode_queue.append("idle")
                self.side_len_queue.append(0.0)
                self.step_elevation_queue.append(0.0)
            return

        if self._stop_decelerating and self._stop_steps_remaining > 0:
            raw_scale = self._stop_steps_remaining / max(1, self.stop_extra_steps)
            scale = max(0.12, self._smooth01(raw_scale))
            step_len *= scale
            turn_len *= scale
            side_len *= scale
            self._stop_steps_remaining -= 1

        side_dominant = abs(side_len) > 0.1 and abs(side_len) >= abs(step_len) + abs(turn_len)
        side_step_len = side_len * 1.80 if side_dominant else side_len
        next_step_count = self.step_count + 1
        if side_dominant and side_len > 0.0:
            swing_is_left = next_step_count % 2 == 0
        else:
            swing_is_left = next_step_count % 2 == 1
        planned_swing_leg = "left" if swing_is_left else "right"
        planned_support_leg = "right" if swing_is_left else "left"
        support_z = float(base_R[2] if swing_is_left else base_L[2])
        swing_start_z = float(base_L[2] if swing_is_left else base_R[2])
        swing_target_z = support_z + (0.0 if side_dominant else step_elevation)

        self.step_count = next_step_count
        self.last_swing_leg = planned_swing_leg

        stance_x = base_R[0] if swing_is_left else base_L[0]
        swing_start_y = base_L[1] if swing_is_left else base_R[1]
        swing_target_y = swing_start_y + side_step_len
        support_y_offset = self.hw * self.zmp_support_ratio
        current_center_y = 0.5 * (base_L[1] + base_R[1])
        next_left_y = swing_target_y if swing_is_left else base_L[1]
        next_right_y = base_R[1] if swing_is_left else swing_target_y
        if side_dominant:
            min_side_gap = self.hw * 2.55
            if swing_is_left and next_right_y - next_left_y < min_side_gap:
                swing_target_y = next_right_y - min_side_gap
                next_left_y = swing_target_y
            elif not swing_is_left and next_right_y - next_left_y < min_side_gap:
                swing_target_y = next_left_y + min_side_gap
                next_right_y = swing_target_y
            side_step_len = swing_target_y - swing_start_y
        next_center_y = 0.5 * (next_left_y + next_right_y)
        support_sign = 1.0 if swing_is_left else -1.0
        stance_y = current_center_y + support_sign * support_y_offset
        next_stance_y = next_center_y - support_sign * support_y_offset

        # Positive turn command means turn left: left step shorter, right step longer.
        sagittal_cmd = 0.0 if side_dominant else step_len + (-turn_len if swing_is_left else turn_len)
        effective_step_len = sagittal_cmd * self.step_x_ratio
        thigh_forward_x = self._thigh_forward_bias(sagittal_cmd)

        current_arm_delta = self._side_arm_offsets() if side_dominant else self._arm_offsets(swing_is_left)
        previous_arm_delta = self.arm_queue[-1] if self.arm_queue else self.last_arm_delta
        if side_dominant:
            swing_distance = 0.0
        elif swing_is_left:
            overstep = self._landing_reach(effective_step_len * self.left_swing_x_scale, sagittal_cmd)
            target_x = stance_x + overstep
            swing_distance = target_x - base_L[0]
        else:
            overstep = self._landing_reach(effective_step_len * self.right_swing_x_scale, sagittal_cmd)
            target_x = stance_x + overstep
            swing_distance = target_x - base_R[0]

        step_n_s = self.n_s

        for k in range(step_n_s):
            alpha = k / max(step_n_s - 1, 1)
            swing_t = self._phase_progress(alpha, self.lift_start_phase, self.swing_advance_end_phase)
            self.zmp_x_queue.append(stance_x)

            lift_factor = self._lift_profile(alpha)
            landing_t = self._phase_progress(alpha, self.swing_advance_end_phase, self.lift_end_phase)
            phase_mode = "land" if landing_t > 0.0 else "swing"
            if phase_mode == "land":
                release_t = self._phase_progress(landing_t, self.landing_roll_release_start, 1.0)
                zmp_y = stance_y + (next_stance_y - stance_y) * release_t
            else:
                release_t = 0.0
                zmp_y = stance_y
            self.zmp_y_queue.append(zmp_y)
            self.zmp_z_queue.append(support_z + (swing_target_z - support_z) * release_t)

            swing_x_scale = self.left_swing_x_scale if swing_is_left else self.right_swing_x_scale
            lift_height_scale = self.left_step_height_scale if swing_is_left else self.right_step_height_scale
            swing_base_z = swing_start_z + (swing_target_z - swing_start_z) * swing_t
            z = swing_start_z if side_dominant else swing_base_z + self.step_height * lift_height_scale * lift_factor

            lift_ready = 1.0 if landing_t > 0.0 else self._smooth01(min(1.0, lift_factor / 0.14))
            swing_x_travel = 0.0 if side_dominant else (swing_distance + thigh_forward_x * swing_x_scale) * swing_t * lift_ready
            arm_phase = self._phase_progress(alpha, self.lift_start_phase, min(0.20, self.swing_advance_end_phase))
            arm_delta = (
                round(previous_arm_delta[0] + (current_arm_delta[0] - previous_arm_delta[0]) * arm_phase),
                round(previous_arm_delta[1] + (current_arm_delta[1] - previous_arm_delta[1]) * arm_phase),
            )

            if side_dominant:
                side_ready = self._smooth01(min(1.0, swing_t * 2.45))
                swing_y_travel = side_step_len * side_ready
            else:
                side_ready = self._smooth01(min(1.0, lift_factor / 0.45))
                swing_y_travel = side_len * swing_t * side_ready

            if swing_is_left:
                self.foot_L_queue.append(np.array([base_L[0] + swing_x_travel, base_L[1] + swing_y_travel, z]))
                self.foot_R_queue.append(np.array([base_R[0], base_R[1], base_R[2]]))
            else:
                self.foot_L_queue.append(np.array([base_L[0], base_L[1], base_L[2]]))
                self.foot_R_queue.append(np.array([base_R[0] + swing_x_travel, base_R[1] + swing_y_travel, z]))
            self.arm_queue.append(arm_delta)
            self.swing_leg_queue.append(planned_swing_leg)
            self.lift_factor_queue.append(lift_factor)
            self.landing_progress_queue.append(landing_t if phase_mode == "land" else 0.0)
            self.phase_mode_queue.append(phase_mode)
            self.side_len_queue.append(side_step_len)
            self.step_elevation_queue.append(0.0 if side_dominant else step_elevation)

    def _thigh_forward_bias(self, sagittal_cmd: float) -> float:
        if abs(sagittal_cmd) < 0.1 or abs(self.thigh_lift_forward_mm) < 0.1:
            return 0.0

        full_cmd = max(1.0, self.max_step_len * 0.18)
        scale = min(1.0, max(0.80, abs(sagittal_cmd) / full_cmd))
        return math.copysign(abs(self.thigh_lift_forward_mm) * scale, sagittal_cmd)

    def _landing_reach(self, planned_reach: float, sagittal_cmd: float) -> float:
        if abs(sagittal_cmd) < 0.1:
            return planned_reach

        direction = 1.0 if sagittal_cmd > 0.0 else -1.0
        return direction * max(abs(planned_reach), self.landing_gap_mm)

    def _swing_pitch_deltas(self, lift_factor: float, forward_x: float = 0.0) -> tuple[int, int, int]:
        lift_height = self.step_height * lift_factor
        forward_x = max(-self.landing_gap_mm, min(self.landing_gap_mm, forward_x))
        forward_x *= self._smooth01(min(1.0, lift_factor / 0.25))
        return lift_pitch_deltas(lift_height, forward_x)

    def _phase_progress(self, phase: float, start: float, end: float) -> float:
        if phase <= start:
            return 0.0
        if phase >= end:
            return 1.0
        return self._smooth01((phase - start) / (end - start))

    def _lift_profile(self, phase: float) -> float:
        if phase <= self.lift_start_phase or phase >= self.lift_end_phase:
            return 0.0
        if phase <= self.swing_advance_end_phase:
            lift_t = self._phase_progress(phase, self.lift_start_phase, self.swing_advance_end_phase)
            return self._smooth01(min(1.0, lift_t / 0.55))
        land_t = self._phase_progress(phase, self.swing_advance_end_phase, self.lift_end_phase)
        return 1.0 - self._smooth01(min(1.0, land_t / 0.65))

    def _arm_offsets(self, swing_is_left: bool) -> tuple[int, int]:
        if self.arm_swing_pwm <= 0:
            return 0, 0

        envelope = self._quantize_arm_delta(self.arm_swing_pwm)
        right_arm = envelope if swing_is_left else -envelope
        left_arm = -right_arm
        return int(right_arm), int(left_arm)

    def _side_arm_offsets(self) -> tuple[int, int]:
        if self.arm_swing_pwm <= 0:
            return 0, 0

        front = self._quantize_arm_delta(self.arm_swing_pwm * 0.55)
        return int(front), int(-front)

    @staticmethod
    def _smooth01(t: float) -> float:
        t = max(0.0, min(1.0, t))
        return t * t * (3.0 - 2.0 * t)

    def _quantize_arm_delta(self, value: float) -> int:
        if abs(value) < 0.5:
            return 0
        sign = 1 if value > 0 else -1
        steps = round(abs(value) / self.arm_quantum_pwm)
        return sign * steps * self.arm_quantum_pwm

    def _apply_arm_swing(self, pose: dict[int, int], arm_delta: tuple[int, int]) -> dict[int, int]:
        if self.arm_swing_pwm <= 0:
            return pose

        arm_step = max(30.0, abs(self.arm_swing_pwm) * self.dt / max(self.dt, float(self.arm_smooth_tau)))
        for idx, target in enumerate(arm_delta):
            delta = target - self._arm_state[idx]
            if abs(delta) <= arm_step:
                self._arm_state[idx] = float(target)
            else:
                self._arm_state[idx] += math.copysign(arm_step, delta)

        out = dict(pose)
        right_delta = self._quantize_arm_delta(self._arm_state[0])
        left_delta = self._quantize_arm_delta(self._arm_state[1])
        right_pwm_delta = self.arm_right_dir * right_delta
        left_pwm_delta = self.arm_left_dir * left_delta
        self.last_arm_delta = (right_pwm_delta, left_pwm_delta)
        self.last_arm_role = self._arm_role(right_delta, left_delta)

        right_elbow = abs(right_delta) * self.arm_elbow_ratio
        left_elbow = abs(left_delta) * self.arm_elbow_ratio
        right_lift = abs(right_delta) * self.arm_lift_ratio
        left_lift = abs(left_delta) * self.arm_lift_ratio

        out[6] = max(500, min(2500, round(STANDING[6] + right_elbow)))
        out[7] = max(500, min(2500, round(STANDING[7] + right_lift)))
        out[8] = max(500, min(2500, STANDING[8] + right_pwm_delta))
        out[17] = max(500, min(2500, STANDING[17] + left_pwm_delta))
        out[18] = max(500, min(2500, round(STANDING[18] - left_lift)))
        out[19] = max(500, min(2500, round(STANDING[19] - left_elbow)))
        return out

    @staticmethod
    def _arm_role(right_delta: int, left_delta: int) -> str:
        if right_delta > 0 and left_delta < 0:
            return "right-forward-left-back"
        if right_delta < 0 and left_delta > 0:
            return "left-forward-right-back"
        if right_delta == 0 and left_delta == 0:
            return "neutral"
        return "transfer"

    def update(
        self,
        forward_cmd: float,
        turn_cmd: float = 0.0,
        side_cmd: float = 0.0,
        step_elevation_mm: float = 0.0,
    ) -> dict[int, int]:
        """Advance the walking engine one frame and return servo pulses."""
        forward_cmd = max(-1.0, min(1.0, forward_cmd))
        turn_cmd = max(-1.0, min(1.0, turn_cmd))
        side_cmd = max(-1.0, min(1.0, side_cmd))
        step_elevation_mm = max(
            -self.max_step_elevation,
            min(self.max_step_elevation, step_elevation_mm),
        )

        if abs(forward_cmd) < self.command_deadzone:
            forward_cmd = 0.0
        if abs(turn_cmd) < self.command_deadzone:
            turn_cmd = 0.0
        if abs(side_cmd) < self.command_deadzone:
            side_cmd = 0.0

        requested_step_len = forward_cmd * self.max_step_len
        requested_turn_len = turn_cmd * self.max_turn_step_len
        requested_side_len = -side_cmd * self.max_side_step_len
        input_active = (
            abs(requested_step_len) > 0.1
            or abs(requested_turn_len) > 0.1
            or abs(requested_side_len) > 0.1
        )
        if input_active:
            self._last_motion_target = (requested_step_len, requested_turn_len, requested_side_len)
            self._stop_steps_remaining = self.stop_extra_steps
            self._stop_decelerating = False
            target_step_len, target_turn_len, target_side_len = self._last_motion_target
        elif self._stop_steps_remaining > 0:
            self._stop_decelerating = True
            target_step_len, target_turn_len, target_side_len = self._last_motion_target
        else:
            self._stop_decelerating = False
            target_step_len = 0.0
            target_turn_len = 0.0
            target_side_len = 0.0
        max_delta = self.command_rate_limit * self.dt

        step_delta = target_step_len - self.commanded_step_len
        if abs(step_delta) > max_delta:
            step_delta = math.copysign(max_delta, step_delta)
        self.commanded_step_len += step_delta

        turn_delta = target_turn_len - self.commanded_turn_len
        if abs(turn_delta) > max_delta:
            turn_delta = math.copysign(max_delta, turn_delta)
        self.commanded_turn_len += turn_delta

        side_delta = target_side_len - self.commanded_side_len
        side_rate = max_delta * 10.0 if abs(target_side_len) > abs(target_step_len) + abs(target_turn_len) else max_delta
        if abs(side_delta) > side_rate:
            side_delta = math.copysign(side_rate, side_delta)
        self.commanded_side_len += side_delta

        while len(self.zmp_y_queue) < self.preview_steps + 1:
            self._enqueue_next_step(
                self.commanded_step_len,
                self.commanded_turn_len,
                self.commanded_side_len,
                step_elevation_mm if input_active else 0.0,
            )

        zmp_now = self.zmp_y_queue.popleft()
        zmp_x_now = self.zmp_x_queue.popleft()
        zmp_z_now = self.zmp_z_queue.popleft()
        foot_L_now = self.foot_L_queue.popleft()
        foot_R_now = self.foot_R_queue.popleft()
        arm_delta_now = self.arm_queue.popleft()
        swing_leg_now = self.swing_leg_queue.popleft()
        lift_factor_now = self.lift_factor_queue.popleft()
        landing_t_now = self.landing_progress_queue.popleft()
        phase_mode_now = self.phase_mode_queue.popleft()
        side_len_now = self.side_len_queue.popleft()
        step_elevation_now = self.step_elevation_queue.popleft()
        self.last_foot_L = foot_L_now
        self.last_foot_R = foot_R_now
        self.last_swing_leg = swing_leg_now
        self.last_lift_factor = lift_factor_now
        self.last_landing_progress = landing_t_now
        self.last_phase_mode = phase_mode_now
        self.last_step_elevation = step_elevation_now

        lateral_origin_y = 0.5 * (float(foot_L_now[1]) + float(foot_R_now[1]))
        zmp_rel_y = zmp_now - lateral_origin_y
        if zmp_rel_y > self.hw * 0.5:
            self.support_leg = "right"
        elif zmp_rel_y < -self.hw * 0.5:
            self.support_leg = "left"
        else:
            self.support_leg = "double"
        support_leg_for_pose = self.support_leg
        if swing_leg_now in ("left", "right"):
            old_support_leg = "right" if swing_leg_now == "left" else "left"
            if phase_mode_now in ("swing", "land"):
                support_leg_for_pose = old_support_leg

        com_y_preview = self.zmp_ctrl.step(zmp_now, list(self.zmp_y_queue)[: self.preview_steps])
        com_x_preview = self.zmp_ctrl_x.step(zmp_x_now, list(self.zmp_x_queue)[: self.preview_steps])
        
        self._com_y = com_y_preview
        self._com_x = com_x_preview
        self._com_z = zmp_z_now
        com_y = self._com_y
        com_x = self._com_x
        pose_foot_L = foot_L_now.copy()
        pose_foot_R = foot_R_now.copy()
        pose_foot_L[1] -= lateral_origin_y
        pose_foot_R[1] -= lateral_origin_y
        neutral_l = np.array([0.0, -self.hw, 0.0])
        neutral_r = np.array([0.0, self.hw, 0.0])
        leg_active = (
            abs(self.commanded_step_len) > 0.05
            or abs(self.commanded_turn_len) > 0.05
            or abs(self.commanded_side_len) > 0.05
            or np.linalg.norm(foot_L_now - neutral_l) > 0.05
            or np.linalg.norm(foot_R_now - neutral_r) > 0.05
            or swing_leg_now != "none"
            or lift_factor_now > 0.05
            or not self.zmp_ctrl.is_settled()
            or not self.zmp_ctrl_x.is_settled()
        )
        side_active = (
            (abs(self.commanded_side_len) > 0.1 or abs(side_len_now) > 0.1)
            and swing_leg_now in ("left", "right")
        )
        pose_com_y = zmp_rel_y if side_active else com_y - lateral_origin_y
        side_motion_len = self.commanded_side_len if abs(self.commanded_side_len) > 0.1 else side_len_now
        side_strength = min(1.0, abs(side_motion_len) / max(1.0, self.max_side_step_len * 0.65)) if side_active else 0.0
        side_dir = 1 if side_motion_len > 0.0 else -1
        side_opening_swing = side_active and (
            (side_dir > 0 and swing_leg_now == "right")
            or (side_dir < 0 and swing_leg_now == "left")
        )
        side_swing_scale = 1.0 if side_opening_swing else 0.46
        side_support_roll = round(26.0 * side_strength)
        side_swing_roll = round(max(side_support_roll * 2.15, 155.0 * max(0.82, side_strength)) * side_swing_scale)
        side_hip_roll = round(175.0 * max(0.82, side_strength) * side_swing_scale) if side_active else 0
        side_pitch_gain = 0.0 if side_active else 1.0
        pose_hip_abduct_gain = self.hip_abduct_gain * (1.0 + 0.35 * side_strength)
        pose_swing_hip_roll_scale = 1.0 + 0.75 * side_strength if side_active else self.swing_hip_roll_scale
        pose_swing_ankle_roll_scale = 0.0 if side_active else self.swing_ankle_roll_scale
        if phase_mode_now == "idle":
            pose = {
                sid: blend_pwm(self.prev_pose.get(sid, STANDING[sid]), STANDING[sid], 0.16)
                for sid in STANDING
            }
        elif leg_active:
            compute_phase_mode = "shift" if phase_mode_now == "swing" else phase_mode_now
            pose = compute_pose(
                com_x,
                pose_com_y,
                pose_foot_L,
                pose_foot_R,
                com_z=self.zc + self._com_z,
                support_leg=support_leg_for_pose,
                phase_mode=compute_phase_mode,
                zmp_support_ratio=self.zmp_support_ratio,
                hip_abduct_gain=pose_hip_abduct_gain,
                swing_hip_roll_scale=pose_swing_hip_roll_scale,
                ankle_roll_gain=self.ankle_roll_gain,
                swing_ankle_roll_scale=pose_swing_ankle_roll_scale,
            )
        else:
            pose = dict(STANDING)
        if phase_mode_now == "swing" and support_leg_for_pose == "right":
            # Right leg is stance, Left leg is swing
            support_blend = self._smooth01(min(1.0, lift_factor_now / 0.35))
            target_1 = pose[1]
            target_5 = pose[5]
            if side_active:
                support_blend = 1.0
                target_1 = max(500, min(2500, STANDING[1] - side_dir * side_support_roll))
                target_5 = STANDING[5]
            pose[1] = round(self.prev_pose.get(1, pose[1]) + (target_1 - self.prev_pose.get(1, pose[1])) * support_blend)
            pose[5] = round(self.prev_pose.get(5, pose[5]) + (target_5 - self.prev_pose.get(5, pose[5])) * support_blend)
            for sid in (2, 3, 4):
                if sid in self.prev_pose:
                    pose[sid] = self.prev_pose[sid]
            swing_lift = lift_factor_now
            swing_forward_x = float(foot_L_now[0] - foot_R_now[0])
            if abs(self.commanded_step_len) > 0.1:
                swing_forward_x = math.copysign(abs(swing_forward_x), self.commanded_step_len)
            target_20 = pose[20]
            if abs(step_elevation_now) > 0.05:
                target_21 = pose[21]
                target_22 = pose[22]
                target_23 = pose[23]
            else:
                thigh_delta, knee_delta, ankle_delta = self._swing_pitch_deltas(swing_lift, swing_forward_x)
                target_21 = STANDING[21] + thigh_delta
                target_22 = STANDING[22] + knee_delta
                target_23 = STANDING[23] + ankle_delta
            support_roll_delta = pose[1] - STANDING[1]
            if side_active:
                target_24 = max(500, min(2500, STANDING[24] - side_dir * side_swing_roll))
            else:
                target_24 = max(500, min(2500, STANDING[24] + support_roll_delta))
            if side_active:
                target_21 = round(STANDING[21] + (pose[21] - STANDING[21]) * side_pitch_gain)
                target_22 = round(STANDING[22] + (pose[22] - STANDING[22]) * side_pitch_gain)
                target_23 = round(STANDING[23] + (pose[23] - STANDING[23]) * side_pitch_gain)
                target_20 = max(500, min(2500, STANDING[20] - side_dir * side_hip_roll))
            swing_blend = self._smooth01(min(1.0, swing_lift / 0.45))
            pose[20] = round(self.prev_pose.get(20, pose[20]) + (target_20 - self.prev_pose.get(20, pose[20])) * swing_blend)
            pose[21] = target_21
            pose[22] = target_22
            pose[23] = target_23
            pose[24] = round(self.prev_pose.get(24, pose[24]) + (target_24 - self.prev_pose.get(24, pose[24])) * swing_blend)
        elif phase_mode_now == "swing" and support_leg_for_pose == "left":
            # Left leg is stance, Right leg is swing
            support_blend = self._smooth01(min(1.0, lift_factor_now / 0.35))
            target_24 = pose[24]
            target_20 = pose[20]
            if side_active:
                support_blend = 1.0
                target_24 = max(500, min(2500, STANDING[24] - side_dir * side_support_roll))
                target_20 = STANDING[20]
            pose[24] = round(self.prev_pose.get(24, pose[24]) + (target_24 - self.prev_pose.get(24, pose[24])) * support_blend)
            pose[20] = round(self.prev_pose.get(20, pose[20]) + (target_20 - self.prev_pose.get(20, pose[20])) * support_blend)
            for sid in (21, 22, 23):
                if sid in self.prev_pose:
                    pose[sid] = self.prev_pose[sid]
            swing_lift = lift_factor_now
            swing_forward_x = float(foot_R_now[0] - foot_L_now[0])
            if abs(self.commanded_step_len) > 0.1:
                swing_forward_x = math.copysign(abs(swing_forward_x), self.commanded_step_len)
            if abs(step_elevation_now) > 0.05:
                target_4 = pose[4]
                target_3 = pose[3]
                target_2 = pose[2]
            else:
                thigh_delta, knee_delta, ankle_delta = self._swing_pitch_deltas(swing_lift, swing_forward_x)
                target_4 = STANDING[4] - thigh_delta
                target_3 = STANDING[3] - knee_delta
                target_2 = STANDING[2] - ankle_delta
            support_roll_delta = pose[24] - STANDING[24]
            if side_active:
                target_1 = max(500, min(2500, STANDING[1] - side_dir * side_swing_roll))
            else:
                target_1 = max(500, min(2500, STANDING[1] + support_roll_delta))
            target_5 = pose[5]
            if side_active:
                target_2 = round(STANDING[2] + (pose[2] - STANDING[2]) * side_pitch_gain)
                target_3 = round(STANDING[3] + (pose[3] - STANDING[3]) * side_pitch_gain)
                target_4 = round(STANDING[4] + (pose[4] - STANDING[4]) * side_pitch_gain)
                target_5 = max(500, min(2500, STANDING[5] - side_dir * side_hip_roll))
            swing_blend = self._smooth01(min(1.0, swing_lift / 0.45))
            pose[1] = round(self.prev_pose.get(1, pose[1]) + (target_1 - self.prev_pose.get(1, pose[1])) * swing_blend)
            pose[4] = target_4
            pose[3] = target_3
            pose[2] = target_2
            pose[5] = round(self.prev_pose.get(5, pose[5]) + (target_5 - self.prev_pose.get(5, pose[5])) * swing_blend)
        elif phase_mode_now == "land" and swing_leg_now in ("left", "right"):
            land_blend = self._smooth01(landing_t_now)

            # Touchdown is continuous: keep the landed foot position as the next
            # support, and only blend the joints needed to make that contact.
            next_support_pose = compute_pose(
                com_x,
                pose_com_y,
                pose_foot_L,
                pose_foot_R,
                com_z=self.zc + self._com_z,
                support_leg=swing_leg_now,
                phase_mode="shift",
                zmp_support_ratio=self.zmp_support_ratio,
                hip_abduct_gain=pose_hip_abduct_gain,
                swing_hip_roll_scale=pose_swing_hip_roll_scale,
                ankle_roll_gain=self.ankle_roll_gain,
                swing_ankle_roll_scale=pose_swing_ankle_roll_scale,
            )

            stride_span = abs(float(foot_L_now[0] - foot_R_now[0]))
            stride_scale = self._smooth01(min(1.0, stride_span / max(1.0, self.landing_gap_mm)))
            landing_forward_lean = round(min(10.0, stride_scale * 10.0))
            terrain_landing = abs(step_elevation_now) > 0.05
            if terrain_landing:
                landing_forward_lean = 0
            if side_active:
                landing_forward_lean = 0
                for sid in (2, 3, 4, 21, 22, 23):
                    next_support_pose[sid] = STANDING[sid]
                next_support_pose[5] = STANDING[5]
                next_support_pose[20] = STANDING[20]
                next_support_pose[1] = STANDING[1] - side_dir * side_support_roll if swing_leg_now == "right" else STANDING[1]
                next_support_pose[24] = STANDING[24] - side_dir * side_support_roll if swing_leg_now == "left" else STANDING[24]
            if swing_leg_now == "left":
                if not terrain_landing:
                    next_support_pose[21] = max(500, min(2500, STANDING[21] + landing_forward_lean))
                    next_support_pose[22] = STANDING[22]
                    next_support_pose[23] = STANDING[23]
                old_support_pitch = () if terrain_landing else (2, 3, 4)
            else:
                if not terrain_landing:
                    next_support_pose[4] = max(500, min(2500, STANDING[4] - landing_forward_lean))
                    next_support_pose[3] = STANDING[3]
                    next_support_pose[2] = STANDING[2]
                old_support_pitch = () if terrain_landing else (21, 22, 23)
            for sid in (1, 2, 3, 4, 5, 20, 21, 22, 23, 24):
                if sid in self.prev_pose:
                    if sid in old_support_pitch:
                        pose[sid] = self.prev_pose[sid]
                    else:
                        pose[sid] = blend_pwm(self.prev_pose[sid], next_support_pose[sid], land_blend)
            if not side_active and not terrain_landing:
                if swing_leg_now == "left":
                    swing_forward_x = float(foot_L_now[0] - foot_R_now[0])
                    if abs(self.commanded_step_len) > 0.1:
                        swing_forward_x = math.copysign(abs(swing_forward_x), self.commanded_step_len)
                    thigh_delta, knee_delta, ankle_delta = self._swing_pitch_deltas(lift_factor_now, swing_forward_x)
                    pose[21] = max(500, min(2500, STANDING[21] + landing_forward_lean + thigh_delta))
                    pose[22] = max(500, min(2500, STANDING[22] + knee_delta))
                    pose[23] = max(500, min(2500, STANDING[23] + ankle_delta))
                else:
                    swing_forward_x = float(foot_R_now[0] - foot_L_now[0])
                    if abs(self.commanded_step_len) > 0.1:
                        swing_forward_x = math.copysign(abs(swing_forward_x), self.commanded_step_len)
                    thigh_delta, knee_delta, ankle_delta = self._swing_pitch_deltas(lift_factor_now, swing_forward_x)
                    pose[4] = max(500, min(2500, STANDING[4] - landing_forward_lean - thigh_delta))
                    pose[3] = max(500, min(2500, STANDING[3] - knee_delta))
                    pose[2] = max(500, min(2500, STANDING[2] - ankle_delta))

        pose = self._apply_arm_swing(pose, arm_delta_now)
        pose = self.pose_filter.update(pose)
        max_pwm_per_frame = self.max_pwm_per_frame
        if not input_active and phase_mode_now in ("land", "idle"):
            max_pwm_per_frame = min(max_pwm_per_frame, 70.0)
        pose = clamp_pose_rate(self.prev_pose, pose, max_pwm_per_frame)
        self.prev_pose = pose
        return pose
