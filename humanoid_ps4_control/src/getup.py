from __future__ import annotations

import math
from dataclasses import dataclass

from .config import STANDING, STAND_ANG
from .walking_engine import angle_to_pwm


@dataclass(frozen=True)
class GetupStep:
    label: str
    pose: dict[int, int]
    duration_s: float


def build_getup_sequence(initial: dict[int, int], speed: float = 1.0) -> list[GetupStep]:
    # Hip, knee, ankle pitch in degrees; all other joints keep their starting pose.
    stages = (
        ("plant-feet", 0.9, 30, 55, 90),
        ("tuck-knees", 1.6, 108, 110, 62),
        ("shift-over-feet", 1.4, 100, 126, 61),
        ("upright-crouch", 1.6, 70, 120, 50),
        ("extend-legs", 1.6, None, None, None),
        ("hold-standing", 0.6, None, None, None),
        ("release-arms", 1.0, None, None, None),
    )
    steps = []
    for label, duration, hip, knee, ankle in stages:
        pose = dict(initial)
        for side, ids in (("L", (13, 14, 15)), ("R", (20, 19, 18))):
            for sid, name, value in zip(ids, ("hip_pitch", "knee", "ankle"), (hip, knee, ankle)):
                base = STAND_ANG[f"{side}_{name}"]
                pose[sid] = angle_to_pwm(sid, base, base if value is None else value, STANDING[sid])
        if label == "release-arms":
            pose = dict(STANDING)
        steps.append(GetupStep(label, pose, duration / speed))
    return steps


class GetupEngine:
    def __init__(self, dt: float = 0.03, speed: float = 1.0) -> None:
        if not math.isfinite(dt) or dt <= 0 or not math.isfinite(speed) or speed <= 0:
            raise ValueError("Get-up dt and speed must be finite and positive")
        self.dt = dt
        self.speed = speed
        self.reset()

    @property
    def running(self) -> bool:
        return self._running

    @property
    def label(self) -> str:
        if self.blocked:
            return "support-not-ready"
        if not self.running:
            return "off"
        if self._wait_s > 0:
            return "wait-upright"
        return self.steps[self.step_index].label

    def reset(self) -> None:
        self._running = False
        self.blocked = False
        self.steps: list[GetupStep] = []
        self.step_index = 0
        self._tick = 0
        self._upright_s = 0.0
        self._wait_s = 0.0
        self._previous_tilt: tuple[float, float] | None = None
        self.current_pose = dict(STANDING)
        self.step_start_pose = dict(STANDING)

    def start(self, current_pose: dict[int, int] | None = None) -> str:
        initial = {**STANDING, **(current_pose or self.current_pose)}
        if set(initial) != set(STANDING) or any(not 500 <= value <= 2500 for value in initial.values()):
            raise ValueError("Get-up requires servo IDs 9..25 and PWM 500..2500")
        self.reset()
        self.current_pose = dict(initial)
        self.step_start_pose = dict(initial)
        self.steps = build_getup_sequence(initial, self.speed)
        self._running = True
        return self.label

    def update(self, tilt: tuple[float, float] | None = None) -> dict[int, int]:
        """Tilt is fresh IMU roll/pitch relative to standing, never a commanded angle."""
        if not self.running or self.blocked:
            return dict(self.current_pose)

        valid = tilt is not None and all(math.isfinite(value) for value in tilt)
        steady = valid and self._previous_tilt is not None and max(abs(value) for value in tilt) <= 8.0
        if steady:
            rate = max(abs((a - b + 180.0) % 360.0 - 180.0) for a, b in zip(tilt, self._previous_tilt)) / self.dt
            steady = rate <= 20.0
        self._upright_s = self._upright_s + self.dt if steady else 0.0
        self._previous_tilt = tilt if valid else None

        step = self.steps[self.step_index]
        if step.label == "release-arms" and self._tick > 0 and not steady:
            self.blocked = True
            return dict(self.current_pose)
        if step.label == "release-arms" and self._tick == 0 and self._upright_s < 0.3:
            self._wait_s += self.dt
            self.blocked = self._wait_s >= 3.0
            return dict(self.current_pose)
        self._wait_s = 0.0
        self._tick += 1
        ticks = max(1, round(step.duration_s / self.dt))
        progress = min(1.0, self._tick / ticks)
        alpha = progress * progress * (3.0 - 2.0 * progress)
        self.current_pose = {
            sid: round(start + alpha * (step.pose[sid] - start))
            for sid, start in self.step_start_pose.items()
        }
        if self._tick >= ticks:
            self.step_index += 1
            self._tick = 0
            self.step_start_pose = dict(self.current_pose)
            if self.step_index == len(self.steps):
                self._running = False
        return dict(self.current_pose)
