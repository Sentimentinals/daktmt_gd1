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
    pitch_ids = (13, 14, 15, 18, 19, 20)
    for group in ((13, 14, 15), (18, 19, 20)):
        max_delta = max(abs(curr[sid] - prev[sid]) for sid in group)
        scale = min(1.0, max_pwm_per_frame / max_delta) if max_delta else 1.0
        for sid in group:
            out[sid] = round(prev[sid] + (curr[sid] - prev[sid]) * scale)
    for sid in DIR:
        if sid in pitch_ids:
            continue
        if sid in prev and sid in curr:
            delta = curr[sid] - prev[sid]
            if abs(delta) > max_pwm_per_frame:
                out[sid] = prev[sid] + int(math.copysign(max_pwm_per_frame, delta))
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
    thigh_scale = 0.68
    lift_scale = 0.60
    thigh_delta = round(raw_thigh_delta * thigh_scale)
    knee_delta = round(raw_knee_delta * lift_scale)
    ankle_delta = knee_delta - thigh_delta
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
    ankle_roll_gain: float | None = None,
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

    ankle_gain = GAIT["ankle_roll_gain"] if ankle_roll_gain is None else ankle_roll_gain
    right_hip_abduct = STAND_ANG["R_hip_abduct"]
    left_hip_abduct = STAND_ANG["L_hip_abduct"]
    if phase_mode == "shift" and support_leg in ("left", "right"):
        support_y = hw * (GAIT["zmp_support_ratio"] if zmp_support_ratio is None else zmp_support_ratio)
        shift_ankle_roll = math.degrees(math.atan2(support_y, roll_height)) * ankle_gain
        right_ankle_roll = STAND_ANG["hip_roll"] + (shift_ankle_roll if support_leg == "right" else 0.0)
        left_ankle_roll = STAND_ANG["hip_roll"] + (shift_ankle_roll if support_leg == "left" else 0.0)
    else:
        ankle_roll = math.degrees(math.atan2(com_y, roll_height)) * ankle_gain
        if support_leg == "right":
            right_ankle_roll = STAND_ANG["hip_roll"] + ankle_roll
            left_ankle_roll = STAND_ANG["hip_roll"]
        elif support_leg == "left":
            right_ankle_roll = STAND_ANG["hip_roll"]
            left_ankle_roll = STAND_ANG["hip_roll"] + ankle_roll
        else:
            right_ankle_roll = STAND_ANG["hip_roll"] + ankle_roll * 0.5
            left_ankle_roll = STAND_ANG["hip_roll"] + ankle_roll * 0.5

    pose = dict(STANDING)
    pose[17] = angle_to_pwm(17, STAND_ANG["hip_roll"], right_ankle_roll, STANDING[17])
    pose[18] = angle_to_pwm(18, STAND_ANG["R_ankle"], ik_R["ankle_pitch"], STANDING[18])
    pose[19] = angle_to_pwm(19, STAND_ANG["R_knee"], ik_R["knee"], STANDING[19])
    pose[20] = angle_to_pwm(20, STAND_ANG["R_hip_pitch"], ik_R["hip_pitch"], STANDING[20])
    pose[21] = angle_to_pwm(21, STAND_ANG["R_hip_abduct"], right_hip_abduct, STANDING[21])

    pose[12] = angle_to_pwm(12, STAND_ANG["L_hip_abduct"], left_hip_abduct, STANDING[12])
    pose[13] = angle_to_pwm(13, STAND_ANG["L_hip_pitch"], ik_L["hip_pitch"], STANDING[13])
    pose[14] = angle_to_pwm(14, STAND_ANG["L_knee"], ik_L["knee"], STANDING[14])
    pose[15] = angle_to_pwm(15, STAND_ANG["L_ankle"], ik_L["ankle_pitch"], STANDING[15])
    pose[16] = angle_to_pwm(16, STAND_ANG["hip_roll"], left_ankle_roll, STANDING[16])
    return pose


class SingleSupportTestEngine:
    def __init__(
        self,
        dt: float = 0.04,
        support_leg: str = "right",
        zmp_support_ratio: float | None = None,
        ankle_roll_gain: float | None = None,
        swing_knee_pwm: int = 260,
        arm_lift_pwm: int = 950,
        ramp_s: float = 1.5,
    ) -> None:
        self.dt = dt
        self.support_leg = support_leg
        self.zmp_support_ratio = GAIT["zmp_support_ratio"] if zmp_support_ratio is None else zmp_support_ratio
        self.ankle_roll_gain = GAIT["ankle_roll_gain"] if ankle_roll_gain is None else ankle_roll_gain
        self.swing_knee_pwm = max(0, int(swing_knee_pwm))
        self.arm_lift_pwm = max(0, int(arm_lift_pwm))
        self.ramp_s = max(dt, ramp_s)
        self.running = False
        self.phase = 0.0
        self.prev_pose = dict(STANDING)

    @property
    def active(self) -> bool:
        return self.running or self.phase > 0.0

    def start(self, support_leg: str | None = None, current_pose: dict[int, int] | None = None) -> None:
        if support_leg is not None:
            self.support_leg = support_leg
        self.running = True
        self.phase = 0.0
        self.prev_pose = dict(STANDING if current_pose is None else current_pose)

    def stop(self) -> None:
        self.running = False

    def reset(self) -> None:
        self.running = False
        self.phase = 0.0
        self.prev_pose = dict(STANDING)

    def update(self) -> dict[int, int]:
        phase_step = self.dt / self.ramp_s
        if self.running:
            self.phase = min(1.0, self.phase + phase_step)
        else:
            self.phase = max(0.0, self.phase - phase_step)
        if self.phase <= 0.0:
            self.prev_pose = dict(STANDING)
            return dict(STANDING)

        blend = self.phase * self.phase * (3.0 - 2.0 * self.phase)
        free_leg_phase = max(0.0, min(1.0, (self.phase - 0.30) / 0.70))
        free_leg_blend = free_leg_phase * free_leg_phase * (3.0 - 2.0 * free_leg_phase)
        hw = ROBOT["half_hip"]
        support_y = hw * self.zmp_support_ratio
        com_y = support_y if self.support_leg == "right" else -support_y

        foot_l = np.array([0.0, -hw, 0.0])
        foot_r = np.array([0.0, hw, 0.0])
        target = compute_pose(
            0.0,
            com_y * blend,
            foot_l,
            foot_r,
            support_leg=self.support_leg,
            phase_mode="full",
            zmp_support_ratio=self.zmp_support_ratio,
            ankle_roll_gain=self.ankle_roll_gain,
        )
        knee_delta = round(self.swing_knee_pwm * free_leg_blend)
        thigh_delta = round(knee_delta * 0.50)
        ankle_delta = knee_delta - thigh_delta
        if self.support_leg == "right":
            target[13] = max(500, min(2500, STANDING[13] + thigh_delta))
            target[14] = max(500, min(2500, STANDING[14] + knee_delta))
            target[15] = max(500, min(2500, STANDING[15] + ankle_delta))
        else:
            target[20] = max(500, min(2500, STANDING[20] - thigh_delta))
            target[19] = max(500, min(2500, STANDING[19] - knee_delta))
            target[18] = max(500, min(2500, STANDING[18] - ankle_delta))
        arm_delta = round(self.arm_lift_pwm * blend)
        target[10] = max(500, min(2500, STANDING[10] - arm_delta))
        target[23] = max(500, min(2500, STANDING[23] + arm_delta))
        pose = {sid: blend_pwm(self.prev_pose.get(sid, STANDING[sid]), target[sid], 0.35) for sid in STANDING}
        self.prev_pose = pose
        return pose


class AdaptiveSquatEngine:
    def __init__(
        self,
        dt: float,
        min_depth_mm: float,
        max_depth_mm: float,
        depth_rate_mm_s: float,
        max_pwm_per_frame: float,
    ) -> None:
        self.dt = max(0.01, dt)
        self.min_depth_mm = max(0.0, min_depth_mm)
        self.max_depth_mm = max(self.min_depth_mm, max_depth_mm)
        self.depth_rate_mm_s = max(1.0, depth_rate_mm_s)
        self.max_pwm_per_frame = max(1.0, max_pwm_per_frame)
        self.depth_mm = 0.0
        self.prev_pose = dict(STANDING)

    def reset(self, pose: dict[int, int] | None = None) -> None:
        self.depth_mm = 0.0
        self.prev_pose = dict(STANDING if pose is None else pose)

    def is_idle(self) -> bool:
        return self.depth_mm <= 0.1 and self.prev_pose == STANDING

    def update(self, depth_ratio: float) -> dict[int, int]:
        ratio = max(0.0, min(1.0, depth_ratio))
        target_depth = 0.0 if ratio <= 0.0 else self.min_depth_mm + ratio * (
            self.max_depth_mm - self.min_depth_mm
        )
        max_delta = self.depth_rate_mm_s * self.dt
        self.depth_mm += max(-max_delta, min(max_delta, target_depth - self.depth_mm))

        half_hip = ROBOT["half_hip"]
        foot_l = np.array([0.0, -half_hip, 0.0])
        foot_r = np.array([0.0, half_hip, 0.0])
        pose = (
            dict(STANDING)
            if self.depth_mm <= 0.1
            else compute_pose(
                0.0,
                0.0,
                foot_l,
                foot_r,
                com_z=ROBOT["com_height"] - self.depth_mm,
                support_leg="double",
            )
        )
        pose = clamp_pose_rate(self.prev_pose, pose, self.max_pwm_per_frame)
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
        step_height: float = 52.0,
        crouch_depth_mm: float = 0.0,
        zmp_support_ratio: float | None = None,
        ankle_roll_gain: float | None = None,
        step_x_ratio: float = 1.0,
        landing_gap_mm: float | None = None,
        lift_start_phase: float = 0.24,
        swing_advance_end_phase: float = 0.60,
        lift_end_phase: float = 1.0,
        landing_roll_release_start: float = 0.42,
        command_deadzone: float | None = None,
        command_rate_limit: float = 16.0,
        max_leg_pwm_per_s: float = 1000.0,
        arm_swing_pwm: int | None = None,
        arm_right_dir: int | None = None,
        arm_left_dir: int | None = None,
        arm_smooth_tau: float | None = None,
        arm_min_pwm: int | None = None,
        arm_quantum_pwm: int | None = None,
        prepare_hold_s: float = 0.0,
        prepare_step_s: float = 0.0,
        prepare_lift_mm: float = 0.0,
    ) -> None:
        self.dt = dt
        self.t_step = t_step
        self.t_dbl = t_dbl
        self.t_single = self.t_step - self.t_dbl

        self.n_s = max(1, round(self.t_single / dt))
        self.n_d = max(1, round(self.t_dbl / dt))

        self.zc = ROBOT["com_height"]
        self.hw = ROBOT["half_hip"]
        self.step_height = max(0.0, step_height)
        self.crouch_depth_mm = max(0.0, float(crouch_depth_mm))
        self.ready_pose = dict(STANDING)
        self.zmp_support_ratio = GAIT["zmp_support_ratio"] if zmp_support_ratio is None else zmp_support_ratio
        self.ankle_roll_gain = GAIT["ankle_roll_gain"] if ankle_roll_gain is None else ankle_roll_gain
        self.step_x_ratio = step_x_ratio
        self.landing_gap_mm = abs(max_step_len if landing_gap_mm is None else landing_gap_mm)
        self.side_lift_scale = 0.55
        self.lift_start_phase = lift_start_phase
        self.swing_advance_end_phase = swing_advance_end_phase
        self.lift_end_phase = lift_end_phase
        self.landing_roll_release_start = landing_roll_release_start
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
        self.arm_smooth_tau = GAIT["arm_smooth_tau"] if arm_smooth_tau is None else arm_smooth_tau
        self.arm_min_pwm = int(GAIT["arm_min_pwm"] if arm_min_pwm is None else arm_min_pwm)
        self.arm_quantum_pwm = max(1, int(GAIT["arm_quantum_pwm"] if arm_quantum_pwm is None else arm_quantum_pwm))
        self.preview_steps = 24

        self.zmp_ctrl = ZMPPreviewController(dt=dt, zc=self.zc, preview_steps=self.preview_steps)
        self.zmp_ctrl_x = ZMPPreviewController(dt=dt, zc=self.zc, preview_steps=self.preview_steps)

        self.max_pwm_per_frame = max(1.0, abs(max_leg_pwm_per_s) * dt)
        self.max_step_len = max_step_len
        self.max_turn_step_len = GAIT["max_turn_step_len"] if max_turn_step_len is None else max_turn_step_len
        self.max_side_step_len = GAIT["max_side_step_len"] if max_side_step_len is None else max_side_step_len
        self.command_rate_limit = abs(command_rate_limit)
        self.prepare_hold_s = max(0.0, prepare_hold_s)
        self.prepare_step_s = max(0.0, prepare_step_s)
        self.prepare_lift_mm = max(0.0, min(abs(prepare_lift_mm), self.step_height))
        self.reset()

    def reset(self) -> None:
        self.zmp_ctrl.reset()
        self.zmp_ctrl_x.reset()
        self.step_count = 0
        self.zmp_y_queue: Deque[float] = deque()
        self.zmp_x_queue: Deque[float] = deque()
        self.body_drop_queue: Deque[float] = deque()
        self.foot_L_queue: Deque[np.ndarray] = deque()
        self.foot_R_queue: Deque[np.ndarray] = deque()
        self.arm_queue: Deque[tuple[int, int]] = deque()
        self.swing_leg_queue: Deque[str] = deque()
        self.lift_factor_queue: Deque[float] = deque()
        self.landing_progress_queue: Deque[float] = deque()
        self.phase_mode_queue: Deque[str] = deque()
        self.side_len_queue: Deque[float] = deque()
        self.support_leg = "double"
        self.commanded_step_len = 0.0
        self.commanded_turn_len = 0.0
        self.commanded_side_len = 0.0
        self.last_foot_L = np.array([0.0, -self.hw, 0.0])
        self.last_foot_R = np.array([0.0, self.hw, 0.0])
        self.last_swing_leg = "none"
        self.last_lift_factor = 0.0
        self.last_landing_progress = 0.0
        self.last_phase_mode = "idle"
        self.last_body_drop = 0.0
        self._arm_state = [0.0, 0.0]
        self._com_y = 0.0
        self._com_x = 0.0
        self._zmp_y = 0.0
        self._zmp_x = 0.0
        self._prepare_pending = self.prepare_step_s > 0.0 and self.prepare_lift_mm > 0.0

        for _ in range(self.n_d):
            self.zmp_y_queue.append(0.0)
            self.zmp_x_queue.append(0.0)
            self.body_drop_queue.append(0.0)
            self.foot_L_queue.append(np.array([0.0, -self.hw, 0.0]))
            self.foot_R_queue.append(np.array([0.0, self.hw, 0.0]))
            self.arm_queue.append((0, 0))
            self.swing_leg_queue.append("none")
            self.lift_factor_queue.append(0.0)
            self.landing_progress_queue.append(0.0)
            self.phase_mode_queue.append("idle")
            self.side_len_queue.append(0.0)

        self.prev_pose = dict(self.ready_pose)

    def _enqueue_preparation(self) -> None:
        base_L = self.last_foot_L.copy()
        base_R = self.last_foot_R.copy()
        hold_frames = round(self.prepare_hold_s / self.dt)
        step_frames = max(2, round(self.prepare_step_s / self.dt))

        for frame in range(hold_frames):
            drop_t = self._smooth01((frame + 1) / max(hold_frames, 1))
            self.zmp_y_queue.append(0.0)
            self.zmp_x_queue.append(0.0)
            self.body_drop_queue.append(self.crouch_depth_mm * drop_t)
            self.foot_L_queue.append(base_L.copy())
            self.foot_R_queue.append(base_R.copy())
            self.arm_queue.append((0, 0))
            self.swing_leg_queue.append("none")
            self.lift_factor_queue.append(0.0)
            self.landing_progress_queue.append(0.0)
            self.phase_mode_queue.append("idle")
            self.side_len_queue.append(0.0)

        support_offset = self.hw * self.zmp_support_ratio
        lift_scale = self.prepare_lift_mm / max(self.step_height, 1.0)
        for swing_is_left in (True, False):
            swing_leg = "left" if swing_is_left else "right"
            stance_y = support_offset if swing_is_left else -support_offset
            next_stance_y = -support_offset if swing_is_left else 0.0
            self.step_count += 1

            for k in range(step_frames):
                alpha = k / max(step_frames - 1, 1)
                lift_t = max(0.0, min(1.0, (alpha - 0.38) / 0.50))
                lift_factor = math.sin(math.pi * lift_t) * lift_scale
                landing_t = self._phase_progress(alpha, 0.78, 1.0)
                zmp_y = stance_y + (next_stance_y - stance_y) * landing_t

                foot_L = base_L.copy()
                foot_R = base_R.copy()
                if swing_is_left:
                    foot_L[2] = self.step_height * lift_factor
                else:
                    foot_R[2] = self.step_height * lift_factor

                self.zmp_y_queue.append(zmp_y)
                self.zmp_x_queue.append(0.0)
                self.body_drop_queue.append(self.crouch_depth_mm)
                self.foot_L_queue.append(foot_L)
                self.foot_R_queue.append(foot_R)
                self.arm_queue.append((0, 0))
                self.swing_leg_queue.append(swing_leg)
                self.lift_factor_queue.append(lift_factor)
                self.landing_progress_queue.append(landing_t)
                self.phase_mode_queue.append("land" if landing_t > 0.0 else "swing")
                self.side_len_queue.append(0.0)

        # The first walking step lifts the left foot, so preparation ends on right support.
        shift_frames = max(2, round(self.prepare_hold_s / self.dt))
        for k in range(shift_frames):
            alpha = self._smooth01((k + 1) / shift_frames)
            self.zmp_y_queue.append(support_offset * alpha)
            self.zmp_x_queue.append(0.0)
            self.body_drop_queue.append(self.crouch_depth_mm)
            self.foot_L_queue.append(base_L.copy())
            self.foot_R_queue.append(base_R.copy())
            self.arm_queue.append((0, 0))
            self.swing_leg_queue.append("none")
            self.lift_factor_queue.append(0.0)
            self.landing_progress_queue.append(0.0)
            self.phase_mode_queue.append("swing")
            self.side_len_queue.append(0.0)

    def is_idle_ready(self, tolerance: float = 0.05) -> bool:
        if (
            abs(self.commanded_step_len) > tolerance
            or abs(self.commanded_turn_len) > tolerance
            or abs(self.commanded_side_len) > tolerance
        ):
            return False

        if any(abs(zmp_y) > tolerance for zmp_y in self.zmp_y_queue):
            return False
        if any(depth > tolerance for depth in self.body_drop_queue) or self.last_body_drop > tolerance:
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
        if any(abs(self.prev_pose.get(sid, self.ready_pose[sid]) - self.ready_pose[sid]) > 3 for sid in DIR):
            return False
        arms_settled = abs(self._arm_state[0]) < self.arm_min_pwm and abs(self._arm_state[1]) < self.arm_min_pwm
        return arms_settled and self.zmp_ctrl.is_settled()

    def _enqueue_next_step(
        self,
        step_len: float,
        turn_len: float = 0.0,
        side_len: float = 0.0,
    ) -> None:
        base_L = self.foot_L_queue[-1].copy() if self.foot_L_queue else self.last_foot_L.copy()
        base_R = self.foot_R_queue[-1].copy() if self.foot_R_queue else self.last_foot_R.copy()

        if abs(step_len) < 0.1 and abs(turn_len) < 0.1 and abs(side_len) < 0.1:
            settle_frames = self.n_s + self.n_d
            stance_center_x = (base_L[0] + base_R[0]) / 2.0
            drop_start = self.body_drop_queue[-1] if self.body_drop_queue else self.last_body_drop
            for frame in range(settle_frames):
                stand_t = self._smooth01((frame + 1) / settle_frames)
                self.zmp_x_queue.append(stance_center_x)
                self.zmp_y_queue.append(0.0)
                self.body_drop_queue.append(drop_start * (1.0 - stand_t))
                self.foot_L_queue.append(base_L.copy())
                self.foot_R_queue.append(base_R.copy())
                self.arm_queue.append((0, 0))
                self.swing_leg_queue.append("none")
                self.lift_factor_queue.append(0.0)
                self.landing_progress_queue.append(0.0)
                self.phase_mode_queue.append("idle")
                self.side_len_queue.append(0.0)
            return

        side_dominant = abs(side_len) > 0.1 and abs(side_len) >= abs(step_len) + abs(turn_len)
        drop_start = self.body_drop_queue[-1] if self.body_drop_queue else self.last_body_drop
        drop_target = self.crouch_depth_mm if abs(step_len) > 0.1 and not side_dominant else 0.0
        side_step_len = side_len * 1.80 if side_dominant else side_len
        next_step_count = self.step_count + 1
        if side_dominant and side_len > 0.0:
            swing_is_left = next_step_count % 2 == 0
        else:
            swing_is_left = next_step_count % 2 == 1
        planned_swing_leg = "left" if swing_is_left else "right"
        support_z = float(base_R[2] if swing_is_left else base_L[2])
        swing_start_z = float(base_L[2] if swing_is_left else base_R[2])
        swing_target_z = support_z

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

        current_arm_delta = self._side_arm_offsets() if side_dominant else self._arm_offsets(swing_is_left)
        if side_dominant:
            swing_distance = 0.0
        elif swing_is_left:
            overstep = self._landing_reach(effective_step_len, sagittal_cmd)
            target_x = stance_x + overstep
            swing_distance = target_x - base_L[0]
        else:
            overstep = self._landing_reach(effective_step_len, sagittal_cmd)
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
                release_start = self.swing_advance_end_phase + (
                    self.lift_end_phase - self.swing_advance_end_phase
                ) * self.landing_roll_release_start
                release_t = self._phase_progress(alpha, release_start, self.lift_end_phase)
                zmp_y = stance_y + (next_stance_y - stance_y) * release_t
            else:
                release_t = 0.0
                zmp_y = stance_y
            self.zmp_y_queue.append(zmp_y)
            drop_t = self._smooth01((k + 1) / step_n_s)
            self.body_drop_queue.append(drop_start + (drop_target - drop_start) * drop_t)

            lift_height_scale = self.side_lift_scale if side_dominant else 1.0
            swing_base_z = swing_start_z + (swing_target_z - swing_start_z) * swing_t
            z = swing_base_z + self.step_height * lift_height_scale * lift_factor

            advance_start = min(self.swing_advance_end_phase - 0.10, self.lift_start_phase + 0.18)
            swing_x_t = self._phase_progress(alpha, advance_start, self.swing_advance_end_phase)
            swing_x_travel = 0.0 if side_dominant else swing_distance * swing_x_t

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
            self.arm_queue.append(current_arm_delta)
            self.swing_leg_queue.append(planned_swing_leg)
            self.lift_factor_queue.append(lift_factor)
            self.landing_progress_queue.append(landing_t if phase_mode == "land" else 0.0)
            self.phase_mode_queue.append(phase_mode)
            self.side_len_queue.append(side_len)

    def _landing_reach(self, planned_reach: float, sagittal_cmd: float) -> float:
        if abs(sagittal_cmd) < 0.1:
            return planned_reach

        direction = 1.0 if sagittal_cmd > 0.0 else -1.0
        return direction * max(abs(planned_reach), self.landing_gap_mm)

    def _side_swing_pitch_deltas(self, lift_factor: float) -> tuple[int, int, int]:
        thigh, knee, ankle = lift_pitch_deltas(self.step_height * lift_factor)
        return (
            round(thigh * self.side_lift_scale),
            round(knee * self.side_lift_scale),
            round(ankle * self.side_lift_scale),
        )

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
            return self._phase_progress(phase, self.lift_start_phase, self.swing_advance_end_phase)
        return 1.0 - self._phase_progress(phase, self.swing_advance_end_phase, self.lift_end_phase)

    def _arm_offsets(self, swing_is_left: bool) -> tuple[int, int]:
        if self.arm_swing_pwm <= 0:
            return 0, 0

        envelope = self.arm_swing_pwm
        right_arm = envelope if swing_is_left else -envelope
        left_arm = -right_arm
        return int(right_arm), int(left_arm)

    def _side_arm_offsets(self) -> tuple[int, int]:
        if self.arm_swing_pwm <= 0:
            return 0, 0

        front = round(self.arm_swing_pwm * 0.55)
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

        out[22] = max(500, min(2500, STANDING[22] + right_pwm_delta))
        out[11] = max(500, min(2500, STANDING[11] + left_pwm_delta))
        return out

    def update(
        self,
        forward_cmd: float,
        turn_cmd: float = 0.0,
        side_cmd: float = 0.0,
    ) -> dict[int, int]:
        """Advance the walking engine one frame and return servo pulses."""
        forward_cmd = max(-1.0, min(1.0, forward_cmd))
        turn_cmd = max(-1.0, min(1.0, turn_cmd))
        side_cmd = max(-1.0, min(1.0, side_cmd))
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
            target_step_len = requested_step_len
            target_turn_len = requested_turn_len
            target_side_len = requested_side_len
        else:
            self.commanded_step_len = 0.0
            self.commanded_turn_len = 0.0
            self.commanded_side_len = 0.0
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

        if not self.zmp_y_queue:
            forward_only = (
                abs(requested_step_len) > 0.1
                and abs(requested_turn_len) <= 0.1
                and abs(requested_side_len) <= 0.1
            )
            if self._prepare_pending:
                self._prepare_pending = False
                if forward_only:
                    self._enqueue_preparation()
            if not self.zmp_y_queue:
                self._enqueue_next_step(
                    self.commanded_step_len,
                    self.commanded_turn_len,
                    self.commanded_side_len,
                )

        zmp_now = self.zmp_y_queue.popleft()
        zmp_x_now = self.zmp_x_queue.popleft()
        body_drop_now = self.body_drop_queue.popleft()
        foot_L_now = self.foot_L_queue.popleft()
        foot_R_now = self.foot_R_queue.popleft()
        arm_delta_now = self.arm_queue.popleft()
        swing_leg_now = self.swing_leg_queue.popleft()
        lift_factor_now = self.lift_factor_queue.popleft()
        landing_t_now = self.landing_progress_queue.popleft()
        phase_mode_now = self.phase_mode_queue.popleft()
        side_len_now = self.side_len_queue.popleft()
        self.last_foot_L = foot_L_now
        self.last_foot_R = foot_R_now
        self.last_swing_leg = swing_leg_now
        self.last_lift_factor = lift_factor_now
        self.last_landing_progress = landing_t_now
        self.last_phase_mode = phase_mode_now
        self.last_body_drop = body_drop_now

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
        self.support_leg = support_leg_for_pose

        zmp_y_preview = list(self.zmp_y_queue)[: self.preview_steps]
        zmp_x_preview = list(self.zmp_x_queue)[: self.preview_steps]
        zmp_y_preview.extend(
            [zmp_y_preview[-1] if zmp_y_preview else zmp_now]
            * (self.preview_steps - len(zmp_y_preview))
        )
        zmp_x_preview.extend(
            [zmp_x_preview[-1] if zmp_x_preview else zmp_x_now]
            * (self.preview_steps - len(zmp_x_preview))
        )
        com_y_preview = self.zmp_ctrl.step(zmp_now, zmp_y_preview)
        com_x_preview = self.zmp_ctrl_x.step(zmp_x_now, zmp_x_preview)
        
        self._com_y = com_y_preview
        self._com_x = com_x_preview
        self._zmp_y = zmp_now
        self._zmp_x = zmp_x_now
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
        side_hip_roll = round(175.0 * max(0.82, side_strength) * side_swing_scale) if side_active else 0
        if phase_mode_now == "idle" and body_drop_now > 0.01:
            pose = compute_pose(
                0.0,
                0.0,
                neutral_l,
                neutral_r,
                com_z=self.zc - body_drop_now,
                support_leg="double",
            )
        elif phase_mode_now == "idle":
            pose = {
                sid: STANDING[sid]
                if abs(self.prev_pose.get(sid, STANDING[sid]) - STANDING[sid]) <= 3
                else blend_pwm(self.prev_pose.get(sid, STANDING[sid]), STANDING[sid], 0.16)
                for sid in STANDING
            }
        elif side_active and phase_mode_now == "swing":
            pose = dict(STANDING)
            thigh_delta, knee_delta, ankle_delta = self._side_swing_pitch_deltas(lift_factor_now)
            swing_blend = self._smooth01(min(1.0, lift_factor_now / 0.45))
            if support_leg_for_pose == "right":
                pose[17] = max(500, min(2500, STANDING[17] - side_dir * side_support_roll))
                pose[18] = self.prev_pose[18]
                pose[19] = self.prev_pose[19]
                pose[20] = self.prev_pose[20]
                pose[12] = blend_pwm(self.prev_pose[12], STANDING[12] - side_dir * side_hip_roll, swing_blend)
                pose[13] = STANDING[13] + thigh_delta
                pose[14] = STANDING[14] + knee_delta
                pose[15] = STANDING[15] + ankle_delta
            else:
                pose[16] = max(500, min(2500, STANDING[16] - side_dir * side_support_roll))
                pose[13] = self.prev_pose[13]
                pose[14] = self.prev_pose[14]
                pose[15] = self.prev_pose[15]
                pose[18] = STANDING[18] - ankle_delta
                pose[19] = STANDING[19] - knee_delta
                pose[20] = STANDING[20] - thigh_delta
                pose[21] = blend_pwm(self.prev_pose[21], STANDING[21] - side_dir * side_hip_roll, swing_blend)
        elif side_active and phase_mode_now == "land":
            target = dict(STANDING)
            support_ankle = 17 if swing_leg_now == "left" else 16
            target[support_ankle] = STANDING[support_ankle] - side_dir * side_support_roll
            land_blend = self._smooth01(landing_t_now)
            pose = {
                sid: blend_pwm(self.prev_pose[sid], target[sid], land_blend)
                for sid in STANDING
            }
        elif leg_active:
            pose = compute_pose(
                com_x,
                pose_com_y,
                pose_foot_L,
                pose_foot_R,
                com_z=self.zc - body_drop_now,
                support_leg=support_leg_for_pose,
                phase_mode="shift",
                zmp_support_ratio=self.zmp_support_ratio,
                ankle_roll_gain=self.ankle_roll_gain,
            )
        else:
            pose = dict(STANDING)
        pose = self._apply_arm_swing(pose, arm_delta_now)
        pose = clamp_pose_rate(self.prev_pose, pose, self.max_pwm_per_frame)
        self.prev_pose = pose
        return pose

    def telemetry_snapshot(self) -> dict[str, object]:
        return {
            "phase": self.last_phase_mode,
            "support_leg": self.support_leg,
            "swing_leg": self.last_swing_leg,
            "step_count": self.step_count,
            "lift_factor": self.last_lift_factor,
            "landing_progress": self.last_landing_progress,
            "crouch_mm": self.last_body_drop,
            "commands": {
                "forward_mm": self.commanded_step_len,
                "turn_mm": self.commanded_turn_len,
                "side_mm": self.commanded_side_len,
            },
            "com_mm": [self._com_x, self._com_y, self.zc - self.last_body_drop],
            "zmp_mm": [self._zmp_x, self._zmp_y, 0.0],
            "feet_mm": {
                "left": self.last_foot_L.tolist(),
                "right": self.last_foot_R.tolist(),
            },
        }
