from __future__ import annotations

import math

import numpy as np

from .config import DIR, GAIT, PWM_PER_DEG, ROBOT, STAND_ANG, STANDING
from .leg_ik import leg_ik


def angle_to_pwm(sid: int, base_ang: float, new_ang: float, base_pwm: int) -> int:
    return round(base_pwm + DIR.get(sid, 1) * (new_ang - base_ang) * PWM_PER_DEG)


def compute_pose(
    com_x: float,
    com_y: float,
    foot_L: np.ndarray,
    foot_R: np.ndarray,
    com_z: float | None = None,
    support_leg: str = "double",
    ankle_roll_gain: float | None = None,
) -> dict[int, int]:
    """Terrain IK; flat walking uses its own ground-contact foot targets below."""
    body_z = ROBOT["com_height"] if com_z is None else com_z
    hw = ROBOT["half_hip"]
    gain = GAIT["ankle_roll_gain"] if ankle_roll_gain is None else ankle_roll_gain
    roll = math.degrees(math.atan2(com_y, ROBOT["com_height"])) * gain
    pose = dict(STANDING)
    for side, foot, ids, sign in (
        ("L", foot_L, (12, 13, 14, 15, 16), -1),
        ("R", foot_R, (21, 20, 19, 18, 17), 1),
    ):
        angles = leg_ik((com_x, com_y + sign * hw, body_z), tuple(foot), ROBOT["upper_leg"], ROBOT["lower_leg"])
        for sid, key, ik_key in zip(ids[1:4], ("hip_pitch", "knee", "ankle"), ("hip_pitch", "knee", "ankle_pitch")):
            pose[sid] = angle_to_pwm(sid, STAND_ANG[f"{side}_{key}"], angles[ik_key], STANDING[sid])
        weight = 0.5 if support_leg == "double" else float(support_leg == ("left" if side == "L" else "right"))
        pose[ids[4]] = angle_to_pwm(ids[4], 0.0, roll * weight, STANDING[ids[4]])
    return pose


class DynamicWalkingEngine:
    """Short sliding steps on a flat floor. One foot/body path, one IK pass."""

    def __init__(
        self,
        dt: float = 0.03,
        t_step: float = 1.3,
        max_step_len: float = 24.0,
        max_turn_step_len: float = 6.0,
        max_side_step_len: float = 12.0,
        crouch_depth_mm: float = 8.0,
        weight_shift_mm: float = 4.0,
        command_deadzone: float = GAIT["command_deadzone"],
    ) -> None:
        values = (dt, t_step, max_step_len, max_turn_step_len, max_side_step_len, crouch_depth_mm, weight_shift_mm)
        if not all(math.isfinite(v) and v >= 0 for v in values) or dt <= 0 or t_step < 2 * dt:
            raise ValueError("Invalid sliding gait dimensions or timing")
        self.dt = dt
        self.frames_per_step = max(2, round(t_step / dt))
        self.max_step_len = max_step_len
        self.max_turn_step_len = max_turn_step_len
        self.max_side_step_len = max_side_step_len
        self.crouch_depth_mm = crouch_depth_mm
        self.weight_shift_mm = weight_shift_mm
        self.command_deadzone = command_deadzone
        self.hw = ROBOT["half_hip"]
        self.zc = ROBOT["com_height"]
        self.neutral_angles = leg_ik((0, 0, self.zc), (0, 0, 0), ROBOT["upper_leg"], ROBOT["lower_leg"])
        self.reset()

    def reset(self) -> None:
        self.feet = np.array([[0.0, -self.hw, 0.0], [0.0, self.hw, 0.0]])
        self.body = np.zeros(2)
        self.body_drop = 0.0
        self.phase = "idle"
        self.swing_leg = "none"
        self.step_count = 0
        self._frame = 0
        self._next_left = True
        self.commands = (0.0, 0.0, 0.0)
        self.prev_pose = dict(STANDING)

    def is_idle_ready(self) -> bool:
        return self.phase == "idle"

    @staticmethod
    def _curve(t: float) -> float:
        t = max(0.0, min(1.0, t))
        return t * t * (3.0 - 2.0 * t)

    def _start_step(self, moving: bool) -> None:
        self._frame = 0
        self._feet_start = self.feet.copy()
        self._feet_target = self.feet.copy()
        self._body_start = self.body.copy()
        self._drop_start = self.body_drop
        self._drop_target = self.crouch_depth_mm if moving else 0.0
        if moving:
            forward, turn, side = self.commands
            if self.phase == "idle" and abs(side) > abs(forward) + abs(turn):
                self._next_left = side < 0.0
            index = 0 if self._next_left else 1
            other = 1 - index
            self._next_left = not self._next_left
            self.swing_leg = "left" if index == 0 else "right"
            self._load_sign = 1.0 if index == 0 else -1.0
            reach = forward if abs(forward) > 0.1 else abs(turn)
            self._feet_target[index, 0] = self.feet[other, 0] + reach + (-turn if index == 0 else turn)
            target_y = self.feet[other, 1] + (-2 * self.hw if index == 0 else 2 * self.hw) + side
            # Closing steps return to nominal width; never cross the feet.
            self._feet_target[index, 1] = min(target_y, self.feet[other, 1] - 2 * self.hw) if index == 0 else max(target_y, self.feet[other, 1] + 2 * self.hw)
            self.phase = "slide"
            self.step_count += 1
        else:
            self._feet_target[:, 0] = self.body[0]
            self._feet_target[:, 1] = self.body[1] + np.array([-self.hw, self.hw])
            self.swing_leg = "none"
            self._load_sign = 0.0
            self.phase = "settle"
        self._body_target = self._feet_target[:, :2].mean(axis=0)

    def _pose(self) -> dict[int, int]:
        pose = dict(STANDING)
        for index, ids, sign in ((0, (12, 13, 14, 15, 16), -1), (1, (21, 20, 19, 18, 17), 1)):
            hip = (self.body[0], self.body[1] + sign * self.hw, self.zc - self.body_drop)
            angles = leg_ik(hip, tuple(self.feet[index]), ROBOT["upper_leg"], ROBOT["lower_leg"])
            for sid, key in zip(ids[1:4], ("hip_pitch", "knee", "ankle_pitch")):
                pose[sid] = angle_to_pwm(sid, self.neutral_angles[key], angles[key], STANDING[sid])
            roll_delta = sign * angles["hip_abduct"]
            # Matched hip/ankle roll changes keep the sole at its calibrated angle.
            for sid in (ids[0], ids[4]):
                pose[sid] = angle_to_pwm(sid, 0.0, roll_delta, STANDING[sid])
        if any(not 500 <= pwm <= 2500 for pwm in pose.values()):
            raise ValueError("Sliding gait target outside servo controller range")
        return pose

    def update(self, forward_cmd: float, turn_cmd: float = 0.0, side_cmd: float = 0.0) -> dict[int, int]:
        inputs = (forward_cmd, turn_cmd, side_cmd)
        if not all(math.isfinite(value) for value in inputs):
            raise ValueError("Non-finite walking command")
        forward, turn, side = (0.0 if abs(v) < self.command_deadzone else max(-1.0, min(1.0, v)) for v in inputs)
        self.commands = (forward * self.max_step_len, turn * self.max_turn_step_len, -side * self.max_side_step_len)
        moving = any(abs(value) > 0.1 for value in self.commands)
        if self.phase == "idle" or self._frame >= self.frames_per_step:
            if self.phase == "idle" and not moving:
                return dict(self.prev_pose)
            self._start_step(moving)

        self._frame += 1
        t = self._frame / self.frames_per_step
        blend = self._curve(t)
        slide = self._curve((t - 0.15) / 0.70) if self.phase == "slide" else blend
        self.feet = self._feet_start + (self._feet_target - self._feet_start) * slide
        self.body = self._body_start + (self._body_target - self._body_start) * blend
        if self.phase == "slide":
            load = self._curve(t / 0.20) * (1.0 - self._curve((t - 0.80) / 0.20))
            self.body[1] += self._load_sign * self.weight_shift_mm * load
        self.body_drop = self._drop_start + (self._drop_target - self._drop_start) * blend
        self.prev_pose = self._pose()
        if self.phase == "settle" and self._frame >= self.frames_per_step:
            self.phase = "idle"
            self.prev_pose = dict(STANDING)
        return dict(self.prev_pose)

    def telemetry_snapshot(self) -> dict[str, object]:
        return {
            "phase": self.phase,
            "support_leg": "double",
            "swing_leg": self.swing_leg,
            "step_count": self.step_count,
            "lift_factor": 0.0,
            "crouch_mm": self.body_drop,
            "commands": dict(zip(("forward_mm", "turn_mm", "side_mm"), self.commands)),
            "com_mm": [*self.body.tolist(), self.zc - self.body_drop],
            "feet_mm": {"left": self.feet[0].tolist(), "right": self.feet[1].tolist()},
        }
