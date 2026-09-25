from __future__ import annotations

import math
from dataclasses import dataclass

from .config import PWM_PER_DEG, STANDING, STAND_ANG
from .walking_engine import angle_to_pwm


@dataclass(frozen=True)
class GetupStep:
    label: str
    pose: dict[int, int]
    duration_s: float


def build_getup_sequence(initial: dict[int, int], speed: float = 1.0) -> list[GetupStep]:
    # Hip/knee/ankle, arm forward, elbow bend and arm spread, in degrees.
    stages = (
        ("brace-hands", 0.7, 30, 55, 90, 65, 30, 10),
        ("push-chest", 0.8, 30, 55, 90, 75, 0, 10),
        ("tuck-knees", 1.2, 106, 110, 62, 75, 0, 10),
        ("shift-over-feet", 1.0, 106, 126, 61, 50, 10, 10),
        ("upright-crouch", 0.8, 82, 120, 50, 20, 0, 25),
        ("extend-legs", 0.9, None, None, None, 0, 0, 70),
        ("hold-standing", 0.6, None, None, None, 0, 0, 70),
        ("release-arms", 1.0, None, None, None, 0, 0, 0),
    )
    steps = []
    for label, duration, hip, knee, ankle, forward, bend, spread in stages:
        pose = dict(initial)
        for sid in (12, 16, 17, 21):
            pose[sid] = STANDING[sid]
        for side, ids in (("L", (13, 14, 15)), ("R", (20, 19, 18))):
            for sid, name, value in zip(ids, ("hip_pitch", "knee", "ankle"), (hip, knee, ankle)):
                base = STAND_ANG[f"{side}_{name}"]
                pose[sid] = angle_to_pwm(sid, base, base if value is None else value, STANDING[sid])
        for left, right, degrees in ((11, 22, forward), (10, 23, spread), (9, 24, -bend)):
            pose[left] = round(STANDING[left] - degrees * PWM_PER_DEG)
            pose[right] = round(STANDING[right] + degrees * PWM_PER_DEG)
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
        # Join the two push phases without stopping, with monotone Hermite tangents.
        join = self.step_index if step.label == "upright-crouch" else self.step_index - 1
        pushing = step.label in ("upright-crouch", "extend-legs")
        if pushing:
            before, middle, after = self.steps[join - 1:join + 2]
            t1 = max(1, round(middle.duration_s / self.dt)) * self.dt
            t2 = max(1, round(after.duration_s / self.dt)) * self.dt
        for sid, start in self.step_start_pose.items():
            value = start + alpha * (step.pose[sid] - start)
            if pushing:
                v1 = (middle.pose[sid] - before.pose[sid]) / t1
                v2 = (after.pose[sid] - middle.pose[sid]) / t2
                velocity = 2 * v1 * v2 / (v1 + v2) if v1 * v2 > 0 else 0.0
                tangent = (
                    progress ** 2 * (progress - 1)
                    if step.label == "upright-crouch"
                    else progress * (1 - progress) ** 2
                )
                value += tangent * ticks * self.dt * velocity
            self.current_pose[sid] = round(value)
        if self._tick >= ticks:
            self.step_index += 1
            self._tick = 0
            self.step_start_pose = dict(self.current_pose)
            if self.step_index == len(self.steps):
                self._running = False
        return dict(self.current_pose)
