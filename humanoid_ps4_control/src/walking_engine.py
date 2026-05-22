from __future__ import annotations

import math
from collections import deque
from typing import Dict, Deque

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


def blend_pwm(start: int, end: int, t: float) -> int:
    t = max(0.0, min(1.0, t))
    return round(start + (end - start) * t)


def compute_pose(
    com_x: float,
    com_y: float,
    foot_L: np.ndarray,
    foot_R: np.ndarray,
    support_leg: str = "double",
    phase_mode: str = "full",
    zmp_support_ratio: float | None = None,
    hip_abduct_gain: float | None = None,
    swing_hip_roll_scale: float | None = None,
    ankle_roll_gain: float | None = None,
    swing_ankle_roll_scale: float | None = None,
) -> dict[int, int]:
    """Convert CoM/foot targets into a full servo pulse pose."""
    zc = ROBOT["com_height"]
    hw = ROBOT["half_hip"]
    L1 = ROBOT["upper_leg"]
    L2 = ROBOT["lower_leg"]

    hip_L = (com_x, com_y - hw, zc)
    hip_R = (com_x, com_y + hw, zc)

    ik_R = leg_ik(hip_R, tuple(foot_R), L1, L2)
    ik_L = leg_ik(hip_L, tuple(foot_L), L1, L2)

    hip_gain = GAIT["hip_abduct_gain"] if hip_abduct_gain is None else hip_abduct_gain
    swing_hip_scale = GAIT["swing_hip_roll_scale"] if swing_hip_roll_scale is None else swing_hip_roll_scale
    ankle_gain = GAIT["ankle_roll_gain"] if ankle_roll_gain is None else ankle_roll_gain
    swing_roll_scale = GAIT["swing_ankle_roll_scale"] if swing_ankle_roll_scale is None else swing_ankle_roll_scale
    ankle_roll = math.degrees(math.atan2(com_y, zc)) * ankle_gain

    if support_leg == "right":
        right_hip_abduct = -abs(ik_R["hip_abduct"]) * hip_gain
        left_hip_abduct = abs(ik_L["hip_abduct"]) * hip_gain * swing_hip_scale
        right_ankle_roll = ankle_roll
        left_ankle_roll = ankle_roll * swing_roll_scale
    elif support_leg == "left":
        right_hip_abduct = abs(ik_R["hip_abduct"]) * hip_gain * swing_hip_scale
        left_hip_abduct = -abs(ik_L["hip_abduct"]) * hip_gain
        right_ankle_roll = ankle_roll * swing_roll_scale
        left_ankle_roll = ankle_roll
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
        raw_roll = math.degrees(math.atan2(support_y, zc))
        shift_ankle_roll = raw_roll * ankle_gain
        hip_counter = raw_roll * hip_gain
        if support_leg == "right":
            pose[1] = angle_to_pwm(1, STAND_ANG["hip_roll"], shift_ankle_roll, STANDING[1])
            pose[5] = angle_to_pwm(5, STAND_ANG["R_hip_abduct"], -hip_counter, STANDING[5])
        elif support_leg == "left":
            pose[24] = angle_to_pwm(24, STAND_ANG["hip_roll"], shift_ankle_roll, STANDING[24])
            pose[20] = angle_to_pwm(20, STAND_ANG["L_hip_abduct"], -hip_counter, STANDING[20])
    return pose


class DynamicWalkingEngine:
    def __init__(
        self,
        dt: float = 0.04,
        t_step: float = 1.28,
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
        command_rate_limit: float = 24.0,
        arm_swing_pwm: int | None = None,
        arm_right_dir: int | None = None,
        arm_left_dir: int | None = None,
        arm_elbow_ratio: float | None = None,
        arm_lift_ratio: float | None = None,
        arm_smooth_tau: float | None = None,
        arm_min_pwm: int | None = None,
        arm_quantum_pwm: int | None = None,
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

        self.max_pwm_per_frame = (360.0 * dt) * PWM_PER_DEG
        self.max_step_len = max_step_len
        self.max_turn_step_len = GAIT["max_turn_step_len"] if max_turn_step_len is None else max_turn_step_len
        self.max_side_step_len = GAIT["max_side_step_len"] if max_side_step_len is None else max_side_step_len
        self.command_rate_limit = abs(command_rate_limit)
        self.stop_extra_steps = max(0, int(GAIT["stop_extra_steps"]))

        self.reset()

    def reset(self) -> None:
        self.zmp_ctrl.reset()
        self.zmp_ctrl_x.reset()
        self.step_count = 0
        self.zmp_y_queue: Deque[float] = deque()
        self.zmp_x_queue: Deque[float] = deque()
        self.foot_L_queue: Deque[np.ndarray] = deque()
        self.foot_R_queue: Deque[np.ndarray] = deque()
        self.arm_queue: Deque[tuple[int, int]] = deque()
        self.swing_leg_queue: Deque[str] = deque()
        self.lift_factor_queue: Deque[float] = deque()
        self.landing_progress_queue: Deque[float] = deque()
        self.phase_mode_queue: Deque[str] = deque()
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
        self._support_roll_hold = {
            "right": {1: STANDING[1], 5: STANDING[5]},
            "left": {24: STANDING[24], 20: STANDING[20]},
        }
        self._arm_state = [0.0, 0.0]
        self._com_y = 0.0
        self._com_x = 0.0
        self._last_motion_target = (0.0, 0.0, 0.0)
        self._stop_steps_remaining = 0
        self._stop_decelerating = False

        for _ in range(self.n_d):
            self.zmp_y_queue.append(0.0)
            self.zmp_x_queue.append(0.0)
            self.foot_L_queue.append(np.array([0.0, -self.hw, 0.0]))
            self.foot_R_queue.append(np.array([0.0, self.hw, 0.0]))
            self.arm_queue.append((0, 0))
            self.swing_leg_queue.append("none")
            self.lift_factor_queue.append(0.0)
            self.landing_progress_queue.append(0.0)
            self.phase_mode_queue.append("idle")

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
        arms_settled = abs(self._arm_state[0]) < self.arm_min_pwm and abs(self._arm_state[1]) < self.arm_min_pwm
        return arms_settled and self.zmp_ctrl.is_settled()

    def _enqueue_next_step(self, step_len: float, turn_len: float = 0.0, side_len: float = 0.0) -> None:
        base_L = self.foot_L_queue[-1].copy() if self.foot_L_queue else self.last_foot_L.copy()
        base_R = self.foot_R_queue[-1].copy() if self.foot_R_queue else self.last_foot_R.copy()
        base_L[2] = 0.0
        base_R[2] = 0.0

        if abs(step_len) < 0.1 and abs(turn_len) < 0.1 and abs(side_len) < 0.1:
            settle_frames = self.n_s + self.n_d
            stance_center_x = (base_L[0] + base_R[0]) / 2.0
            for _ in range(settle_frames):
                self.zmp_x_queue.append(stance_center_x)
                self.zmp_y_queue.append(0.0)
                self.foot_L_queue.append(base_L.copy())
                self.foot_R_queue.append(base_R.copy())
                self.arm_queue.append((0, 0))
                self.swing_leg_queue.append("none")
                self.lift_factor_queue.append(0.0)
                self.landing_progress_queue.append(0.0)
                self.phase_mode_queue.append("idle")
            return

        if self._stop_decelerating and self._stop_steps_remaining > 0:
            scale = max(0.45, self._stop_steps_remaining / max(1, self.stop_extra_steps))
            step_len *= scale
            turn_len *= scale
            side_len *= scale
            self._stop_steps_remaining -= 1

        self.step_count += 1
        swing_is_left = self.step_count % 2 == 1
        planned_swing_leg = "left" if swing_is_left else "right"
        self.last_swing_leg = planned_swing_leg

        stance_y = self.hw * self.zmp_support_ratio if swing_is_left else -self.hw * self.zmp_support_ratio
        next_stance_y = -stance_y
        stance_x = base_R[0] if swing_is_left else base_L[0]

        # Positive turn command means turn left: left step shorter, right step longer.
        sagittal_cmd = step_len + (-turn_len if swing_is_left else turn_len)
        effective_step_len = sagittal_cmd * self.step_x_ratio
        thigh_forward_x = self._thigh_forward_bias(sagittal_cmd)

        current_arm_delta = self._arm_offsets(swing_is_left)
        previous_arm_delta = self.arm_queue[-1] if self.arm_queue else (0, 0)

        # Stance-relative targeting: the swing foot lands a fixed overstep
        # distance ahead of the stance foot. This is correct walking mechanics.
        if swing_is_left:
            overstep = self._landing_reach(effective_step_len * self.left_swing_x_scale, sagittal_cmd)
            target_x = stance_x + overstep
            swing_distance = target_x - base_L[0]
        else:
            overstep = self._landing_reach(effective_step_len * self.right_swing_x_scale, sagittal_cmd)
            target_x = stance_x + overstep
            swing_distance = target_x - base_R[0]

        # Scale n_s so servo velocity stays constant regardless of swing distance.
        # Step 1 travels ~overstep, step 2+ travels ~2*overstep. Without scaling,
        # step 2+ servos would move 2x faster, causing jerk and imbalance.
        reference_stride = abs(overstep) if abs(overstep) > 0.5 else max(1.0, self.landing_gap_mm)
        stride_ratio = abs(swing_distance) / reference_stride if reference_stride > 0.5 else 1.0
        step_n_s = max(self.n_s, round(self.n_s * stride_ratio))

        arm_switched = False
        for k in range(step_n_s):
            alpha = k / max(step_n_s - 1, 1)
            smooth_t = alpha * alpha * alpha * (alpha * (alpha * 6.0 - 15.0) + 10.0)
            swing_t = self._phase_progress(alpha, self.lift_start_phase, self.swing_advance_end_phase)
            self.zmp_x_queue.append(stance_x)

            lift_factor = self._lift_profile(alpha)
            landing_t = self._phase_progress(alpha, self.swing_advance_end_phase, self.lift_end_phase)
            if landing_t > 0.0:
                phase_mode = "land"
            elif self.lift_start_phase <= 0.0 and alpha > 0.0:
                phase_mode = "swing"
            elif lift_factor <= 0.02:
                phase_mode = "shift"
            else:
                phase_mode = "swing"
            if phase_mode == "land":
                release_t = self._phase_progress(landing_t, self.landing_roll_release_start, 1.0)
                zmp_y = stance_y + (next_stance_y - stance_y) * release_t
            elif phase_mode in ("shift", "swing"):
                zmp_y = stance_y
            else:
                zmp_y = 0.0
            self.zmp_y_queue.append(zmp_y)

            swing_x_scale = self.left_swing_x_scale if swing_is_left else self.right_swing_x_scale
            lift_height_scale = self.left_step_height_scale if swing_is_left else self.right_step_height_scale
            z = self.step_height * lift_height_scale * lift_factor

            swing_x_travel = (swing_distance + thigh_forward_x * swing_x_scale) * swing_t
            if lift_factor > 0.02:
                arm_switched = True
            arm_delta = current_arm_delta if arm_switched else previous_arm_delta
            if phase_mode == "land":
                boost_factor = lift_factor * (1.0 - landing_t)
            else:
                boost_factor = lift_factor

            stance_y_offset = side_len / 2.0 - side_len * smooth_t
            swing_y_offset = -side_len / 2.0 + side_len * smooth_t

            if swing_is_left:
                self.foot_L_queue.append(np.array([base_L[0] + swing_x_travel, -self.hw + swing_y_offset, z]))
                self.foot_R_queue.append(np.array([base_R[0], self.hw + stance_y_offset, 0.0]))
            else:
                self.foot_L_queue.append(np.array([base_L[0], -self.hw + stance_y_offset, 0.0]))
                self.foot_R_queue.append(np.array([base_R[0] + swing_x_travel, self.hw + swing_y_offset, z]))
            self.arm_queue.append(arm_delta)
            self.swing_leg_queue.append(planned_swing_leg if phase_mode in ("shift", "swing", "land") else "none")
            self.lift_factor_queue.append(boost_factor)
            self.landing_progress_queue.append(landing_t if phase_mode == "land" else 0.0)
            self.phase_mode_queue.append(phase_mode)

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

    def _phase_progress(self, phase: float, start: float, end: float) -> float:
        if phase <= start:
            return 0.0
        if phase >= end:
            return 1.0
        return self._smooth01((phase - start) / (end - start))

    def _lift_profile(self, phase: float) -> float:
        lift_t = self._phase_progress(phase, self.lift_start_phase, self.lift_end_phase)
        ramp = 0.22
        drop = 0.26
        if lift_t <= 0.0 or lift_t >= 1.0:
            return 0.0
        if lift_t < ramp:
            return self._smooth01(lift_t / ramp)
        if lift_t <= 1.0 - drop:
            return 1.0
        return 1.0 - self._smooth01((lift_t - (1.0 - drop)) / drop)

    def _arm_offsets(self, swing_is_left: bool) -> tuple[int, int]:
        if self.arm_swing_pwm <= 0:
            return 0, 0

        envelope = self._quantize_arm_delta(self.arm_swing_pwm)
        if not swing_is_left:
            envelope = -envelope
        # Return one logical shoulder command. Servo direction config maps this
        # into opposite physical motion for servo 8 and servo 17.
        return int(envelope), int(envelope)

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

        tau = max(self.dt, float(self.arm_smooth_tau))
        alpha = 1.0 - math.exp(-self.dt / tau)
        self._arm_state[0] += alpha * (arm_delta[0] - self._arm_state[0])
        self._arm_state[1] += alpha * (arm_delta[1] - self._arm_state[1])

        out = dict(pose)
        right_delta = self._quantize_arm_delta(self._arm_state[0])
        left_delta = self._quantize_arm_delta(self._arm_state[1])
        right_pwm_delta = self.arm_right_dir * right_delta
        left_pwm_delta = self.arm_left_dir * left_delta
        self.last_arm_delta = (right_pwm_delta, left_pwm_delta)
        self.last_arm_role = self._arm_role(right_pwm_delta, left_pwm_delta)

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

    def update(self, joystick_y: float, turn_cmd: float = 0.0, side_cmd: float = 0.0) -> Dict[int, int]:
        """Advance the walking engine one frame and return servo pulses."""
        joystick_y = max(-1.0, min(1.0, joystick_y))
        turn_cmd = max(-1.0, min(1.0, turn_cmd))
        side_cmd = max(-1.0, min(1.0, side_cmd))

        if abs(joystick_y) < self.command_deadzone:
            joystick_y = 0.0
        if abs(turn_cmd) < self.command_deadzone:
            turn_cmd = 0.0
        if abs(side_cmd) < self.command_deadzone:
            side_cmd = 0.0

        requested_step_len = joystick_y * self.max_step_len
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
        if abs(side_delta) > max_delta:
            side_delta = math.copysign(max_delta, side_delta)
        self.commanded_side_len += side_delta

        while len(self.zmp_y_queue) < self.preview_steps + 1:
            self._enqueue_next_step(self.commanded_step_len, self.commanded_turn_len, self.commanded_side_len)

        zmp_now = self.zmp_y_queue.popleft()
        zmp_x_now = self.zmp_x_queue.popleft()
        foot_L_now = self.foot_L_queue.popleft()
        foot_R_now = self.foot_R_queue.popleft()
        arm_delta_now = self.arm_queue.popleft()
        swing_leg_now = self.swing_leg_queue.popleft()
        lift_factor_now = self.lift_factor_queue.popleft()
        landing_t_now = self.landing_progress_queue.popleft()
        phase_mode_now = self.phase_mode_queue.popleft()
        self.last_foot_L = foot_L_now
        self.last_foot_R = foot_R_now
        self.last_swing_leg = swing_leg_now
        self.last_lift_factor = lift_factor_now
        self.last_landing_progress = landing_t_now
        self.last_phase_mode = phase_mode_now

        if zmp_now > self.hw * 0.5:
            self.support_leg = "right"
        elif zmp_now < -self.hw * 0.5:
            self.support_leg = "left"
        else:
            self.support_leg = "double"
        support_leg_for_pose = self.support_leg
        old_support_leg = "none"
        if swing_leg_now in ("left", "right"):
            old_support_leg = "right" if swing_leg_now == "left" else "left"
            if phase_mode_now in ("shift", "swing"):
                support_leg_for_pose = old_support_leg
            elif phase_mode_now == "land":
                support_leg_for_pose = old_support_leg

        com_y_preview = self.zmp_ctrl.step(zmp_now, list(self.zmp_y_queue)[: self.preview_steps])
        com_x_preview = self.zmp_ctrl_x.step(zmp_x_now, list(self.zmp_x_queue)[: self.preview_steps])
        
        self._com_y = com_y_preview
        self._com_x = com_x_preview
        com_y = self._com_y
        com_x = self._com_x
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
        if leg_active:
            compute_phase_mode = "shift" if phase_mode_now == "swing" else phase_mode_now
            pose = compute_pose(
                com_x,
                com_y,
                foot_L_now,
                foot_R_now,
                support_leg=support_leg_for_pose,
                phase_mode=compute_phase_mode,
                zmp_support_ratio=self.zmp_support_ratio,
                hip_abduct_gain=self.hip_abduct_gain,
                swing_hip_roll_scale=self.swing_hip_roll_scale,
                ankle_roll_gain=self.ankle_roll_gain,
                swing_ankle_roll_scale=self.swing_ankle_roll_scale,
            )
        else:
            pose = dict(STANDING)
        if phase_mode_now == "shift":
            # Lock roll servos during shift — only roll should move
            if support_leg_for_pose == "right":
                self._support_roll_hold["right"] = {1: pose[1], 5: pose[5]}
            elif support_leg_for_pose == "left":
                self._support_roll_hold["left"] = {24: pose[24], 20: pose[20]}
        elif phase_mode_now == "swing" and support_leg_for_pose == "right":
            # Right leg is stance, Left leg is swing
            self._support_roll_hold["right"] = {1: pose[1], 5: pose[5]}
            pose[1] = self._support_roll_hold["right"][1]
            pose[5] = self._support_roll_hold["right"][5]
            # Blend Left thigh pitch to IK target
            swing_blend = self._smooth01(lift_factor_now)
            pose[21] = round(self.prev_pose.get(21, pose[21]) + (pose[21] - self.prev_pose.get(21, pose[21])) * swing_blend)
        elif phase_mode_now == "swing" and support_leg_for_pose == "left":
            # Left leg is stance, Right leg is swing
            self._support_roll_hold["left"] = {24: pose[24], 20: pose[20]}
            pose[24] = self._support_roll_hold["left"][24]
            pose[20] = self._support_roll_hold["left"][20]
            # Blend Right thigh pitch to IK target
            swing_blend = self._smooth01(lift_factor_now)
            pose[4] = round(self.prev_pose.get(4, pose[4]) + (pose[4] - self.prev_pose.get(4, pose[4])) * swing_blend)
        elif phase_mode_now == "land" and support_leg_for_pose == "right" and old_support_leg == "right":
            release_t = self._phase_progress(landing_t_now, self.landing_roll_release_start, 1.0)
            hold = self._support_roll_hold["right"]
            pose[1] = blend_pwm(hold[1], STANDING[1], release_t)
            pose[5] = blend_pwm(hold[5], STANDING[5], release_t)
            # Only Left thigh pitch is adjusted during landing
            land_blend = self._smooth01(landing_t_now)
            prev_thigh = self.prev_pose.get(21, pose[21])
            pose[21] = round(prev_thigh + (STANDING[21] - prev_thigh) * land_blend)
        elif phase_mode_now == "land" and support_leg_for_pose == "left" and old_support_leg == "left":
            release_t = self._phase_progress(landing_t_now, self.landing_roll_release_start, 1.0)
            hold = self._support_roll_hold["left"]
            pose[24] = blend_pwm(hold[24], STANDING[24], release_t)
            pose[20] = blend_pwm(hold[20], STANDING[20], release_t)
            # Only Right thigh pitch is adjusted during landing
            land_blend = self._smooth01(landing_t_now)
            prev_thigh = self.prev_pose.get(4, pose[4])
            pose[4] = round(prev_thigh + (STANDING[4] - prev_thigh) * land_blend)


        if phase_mode_now == "land" and swing_leg_now in ("left", "right"):
            release_t = self._phase_progress(landing_t_now, self.landing_roll_release_start, 1.0)
            if release_t > 0.0:
                next_support_pose = compute_pose(
                    com_x,
                    com_y,
                    foot_L_now,
                    foot_R_now,
                    support_leg=swing_leg_now,
                    phase_mode="shift",
                    zmp_support_ratio=self.zmp_support_ratio,
                    hip_abduct_gain=self.hip_abduct_gain,
                    swing_hip_roll_scale=self.swing_hip_roll_scale,
                    ankle_roll_gain=self.ankle_roll_gain,
                    swing_ankle_roll_scale=self.swing_ankle_roll_scale,
                )
                if swing_leg_now == "right":
                    pose[1] = blend_pwm(STANDING[1], next_support_pose[1], release_t)
                    pose[5] = blend_pwm(STANDING[5], next_support_pose[5], release_t)
                    self._support_roll_hold["right"] = {1: pose[1], 5: pose[5]}
                else:
                    pose[24] = blend_pwm(STANDING[24], next_support_pose[24], release_t)
                    pose[20] = blend_pwm(STANDING[20], next_support_pose[20], release_t)
                    self._support_roll_hold["left"] = {24: pose[24], 20: pose[20]}

        if phase_mode_now == "swing" and swing_leg_now == "right" and lift_factor_now > 0.02:
            swing_blend = self._smooth01(lift_factor_now)
            target_hip = max(500, min(2500, STANDING[5] - abs(self._support_roll_hold["left"][20] - STANDING[20])))
            pose[5] = round(pose[5] + (target_hip - pose[5]) * swing_blend)
            pose[1] = round(pose[1] + (STANDING[1] - pose[1]) * swing_blend)
        elif phase_mode_now == "swing" and swing_leg_now == "left" and lift_factor_now > 0.02:
            swing_blend = self._smooth01(lift_factor_now)
            target_hip = max(500, min(2500, STANDING[20] + abs(self._support_roll_hold["right"][5] - STANDING[5])))
            pose[20] = round(pose[20] + (target_hip - pose[20]) * swing_blend)
            pose[24] = round(pose[24] + (STANDING[24] - pose[24]) * swing_blend)

        # Manual swing leg overrides for knee and ankle (active when lift_factor_now > 0.02)
        if lift_factor_now > 0.02 and swing_leg_now in ("left", "right"):
            knee_delta = round(110 * lift_factor_now)
            ankle_delta = round(110 * lift_factor_now)
            swing_blend = self._smooth01(lift_factor_now)
            if swing_leg_now == "left":
                target_22 = STANDING[22] - knee_delta
                target_23 = STANDING[23] + ankle_delta
                pose[22] = round(self.prev_pose.get(22, pose[22]) + (target_22 - self.prev_pose.get(22, pose[22])) * swing_blend)
                pose[23] = round(self.prev_pose.get(23, pose[23]) + (target_23 - self.prev_pose.get(23, pose[23])) * swing_blend)
            elif swing_leg_now == "right":
                target_3 = STANDING[3] + knee_delta
                target_2 = STANDING[2] - ankle_delta
                pose[3] = round(self.prev_pose.get(3, pose[3]) + (target_3 - self.prev_pose.get(3, pose[3])) * swing_blend)
                pose[2] = round(self.prev_pose.get(2, pose[2]) + (target_2 - self.prev_pose.get(2, pose[2])) * swing_blend)

        pose = self._apply_arm_swing(pose, arm_delta_now)
        pose = clamp_pose_rate(self.prev_pose, pose, self.max_pwm_per_frame)
        self.prev_pose = pose
        return pose
