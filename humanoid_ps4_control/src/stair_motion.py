from __future__ import annotations

import time

import numpy as np

from .config import ROBOT, STANDING
from .walking_engine import compute_pose


class StairStepEngine:
    PHASES = ("shift", "lead_swing", "transfer", "trail_swing", "settle")

    def __init__(
        self,
        *,
        clearance_mm: float,
        shift_s: float,
        swing_s: float,
        transfer_s: float,
        settle_s: float,
        zmp_support_ratio: float,
        ankle_roll_gain: float,
    ) -> None:
        self.clearance_mm = max(5.0, clearance_mm)
        self.durations = {
            "shift": max(0.25, shift_s),
            "lead_swing": max(0.50, swing_s),
            "transfer": max(0.35, transfer_s),
            "trail_swing": max(0.50, swing_s),
            "settle": max(0.30, settle_s),
        }
        self.zmp_support_ratio = zmp_support_ratio
        self.ankle_roll_gain = ankle_roll_gain
        self.active = False
        self.direction = "up"
        self.lead_leg = "left"
        self.step_height_mm = 0.0
        self.step_depth_mm = 0.0
        self.started_at = 0.0
        self.phase = "idle"
        self.support_leg = "double"
        self.lift_factor = 0.0
        self.landing_progress = 0.0
        self.completed_steps = 0
        self.com_mm = [0.0, 0.0, ROBOT["com_height"]]
        self.zmp_mm = [0.0, 0.0, 0.0]
        self.feet_mm = {
            "left": [0.0, -ROBOT["half_hip"], 0.0],
            "right": [0.0, ROBOT["half_hip"], 0.0],
        }

    @staticmethod
    def _curve(value: float) -> float:
        value = max(0.0, min(1.0, value))
        return value * value * value * (10.0 + value * (-15.0 + 6.0 * value))

    @staticmethod
    def _bump(value: float) -> float:
        value = max(0.0, min(1.0, value))
        return 16.0 * value * value * (1.0 - value) * (1.0 - value)

    def start(
        self,
        direction: str,
        step_height_mm: float,
        step_depth_mm: float,
        lead_leg: str,
        now: float | None = None,
    ) -> None:
        if direction not in ("up", "down"):
            raise ValueError(f"Unsupported stair direction: {direction}")
        if lead_leg not in ("left", "right"):
            raise ValueError(f"Unsupported lead leg: {lead_leg}")
        signed_height = abs(step_height_mm) if direction == "up" else -abs(step_height_mm)
        self.direction = direction
        self.lead_leg = lead_leg
        self.step_height_mm = signed_height
        self.step_depth_mm = abs(step_depth_mm)
        self.started_at = time.monotonic() if now is None else now
        self.active = True
        self.phase = "shift"

    def reset(self) -> None:
        self.active = False
        self.phase = "idle"
        self.support_leg = "double"
        self.lift_factor = 0.0
        self.landing_progress = 0.0
        self.com_mm = [0.0, 0.0, ROBOT["com_height"]]
        self.zmp_mm = [0.0, 0.0, 0.0]
        self.feet_mm = {
            "left": [0.0, -ROBOT["half_hip"], 0.0],
            "right": [0.0, ROBOT["half_hip"], 0.0],
        }

    def update(self, now: float | None = None) -> dict[int, int]:
        if not self.active:
            return dict(STANDING)
        elapsed = (time.monotonic() if now is None else now) - self.started_at
        phase, progress = self._phase_at(elapsed)
        if phase is None:
            self.active = False
            self.phase = "idle"
            self.support_leg = "double"
            self.lift_factor = 0.0
            self.landing_progress = 1.0
            self.completed_steps += 1
            return dict(STANDING)
        self.phase = phase
        return self._phase_pose(phase, progress)

    def _phase_at(self, elapsed: float) -> tuple[str | None, float]:
        cursor = 0.0
        for phase in self.PHASES:
            duration = self.durations[phase]
            if elapsed < cursor + duration:
                return phase, max(0.0, (elapsed - cursor) / duration)
            cursor += duration
        return None, 1.0

    def _phase_pose(self, phase: str, progress: float) -> dict[int, int]:
        s = self._curve(progress)
        half_hip = ROBOT["half_hip"]
        lead_left = self.lead_leg == "left"
        lead_support_y = -half_hip * self.zmp_support_ratio if lead_left else half_hip * self.zmp_support_ratio
        trail_support_y = -lead_support_y
        signed_height = self.step_height_mm
        depth = self.step_depth_mm
        lead_foot = np.array([0.0, -half_hip if lead_left else half_hip, 0.0])
        trail_foot = np.array([0.0, half_hip if lead_left else -half_hip, 0.0])
        body_x = 0.0
        body_y = 0.0
        body_z = ROBOT["com_height"]
        if phase == "shift":
            body_y = trail_support_y * s
            self.support_leg = "right" if lead_left else "left"
            self.lift_factor = 0.0
            self.landing_progress = 0.0
        elif phase == "lead_swing":
            body_y = trail_support_y
            body_z += min(0.0, signed_height) * s
            lead_foot[0] = depth * s
            lead_foot[2] = signed_height * s + self.clearance_mm * self._bump(progress)
            self.support_leg = "right" if lead_left else "left"
            self.lift_factor = self._bump(progress)
            self.landing_progress = max(0.0, (progress - 0.55) / 0.45)
        elif phase == "transfer":
            lead_foot[0] = depth
            lead_foot[2] = signed_height
            body_x = depth * 0.70 * s
            weight_shift = self._curve(min(1.0, progress * 2.0))
            body_y = trail_support_y + (lead_support_y - trail_support_y) * weight_shift
            if signed_height > 0.0:
                rise = signed_height * s
                body_z += rise
                trail_foot[2] = rise
            else:
                body_z += signed_height
            self.support_leg = "double"
            self.lift_factor = 0.0
            self.landing_progress = 1.0
        elif phase == "trail_swing":
            lead_foot[0] = depth
            lead_foot[2] = signed_height
            trail_foot[0] = depth * s
            if signed_height > 0.0:
                trail_foot[2] = signed_height + self.clearance_mm * self._bump(progress)
            else:
                trail_foot[2] = signed_height * s + self.clearance_mm * self._bump(progress)
            body_x = depth * 0.70
            body_y = lead_support_y
            body_z += signed_height
            self.support_leg = self.lead_leg
            self.lift_factor = self._bump(progress)
            self.landing_progress = max(0.0, (progress - 0.55) / 0.45)
        else:
            lead_foot[0] = depth
            lead_foot[2] = signed_height
            trail_foot[0] = depth
            trail_foot[2] = signed_height
            body_x = depth * (0.70 + 0.30 * s)
            body_y = lead_support_y * (1.0 - s)
            body_z += signed_height
            self.support_leg = "double"
            self.lift_factor = 0.0
            self.landing_progress = 1.0

        foot_left = lead_foot if lead_left else trail_foot
        foot_right = trail_foot if lead_left else lead_foot
        self.com_mm = [body_x, body_y, body_z]
        self.feet_mm = {"left": foot_left.tolist(), "right": foot_right.tolist()}
        if self.support_leg == "left":
            self.zmp_mm = foot_left.tolist()
        elif self.support_leg == "right":
            self.zmp_mm = foot_right.tolist()
        else:
            self.zmp_mm = ((foot_left + foot_right) * 0.5).tolist()
        return compute_pose(
            body_x,
            body_y,
            foot_left,
            foot_right,
            com_z=body_z,
            support_leg=self.support_leg,
            phase_mode="full",
            zmp_support_ratio=self.zmp_support_ratio,
            ankle_roll_gain=self.ankle_roll_gain,
        )

    def telemetry_snapshot(self) -> dict[str, object]:
        return {
            "phase": self.phase,
            "support_leg": self.support_leg,
            "swing_leg": self.lead_leg if self.phase == "lead_swing" else (
                "right" if self.lead_leg == "left" else "left"
            ) if self.phase == "trail_swing" else None,
            "step_count": self.completed_steps,
            "lift_factor": self.lift_factor,
            "landing_progress": self.landing_progress,
            "crouch_mm": 0.0,
            "commands": {
                "forward_mm": self.step_depth_mm,
                "turn_mm": 0.0,
                "side_mm": 0.0,
            },
            "com_mm": self.com_mm,
            "zmp_mm": self.zmp_mm,
            "feet_mm": self.feet_mm,
            "stair": {
                "direction": self.direction,
                "riser_mm": abs(self.step_height_mm),
                "tread_mm": self.step_depth_mm,
            },
        }
