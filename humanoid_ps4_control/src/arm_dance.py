from __future__ import annotations

import math

from .walking_engine import STANDING


ARM_DANCE_IDS = (6, 7, 8, 16, 17, 18, 19)


def _clamp_pwm(value: float) -> int:
    return max(500, min(2500, round(value)))


def _smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def _blend_pose(a: dict[int, int], b: dict[int, int], t: float) -> dict[int, int]:
    alpha = _smoothstep(t)
    ids = set(a) | set(b)
    return {
        sid: _clamp_pwm(a.get(sid, STANDING.get(sid, 1500)) * (1.0 - alpha) + b.get(sid, STANDING.get(sid, 1500)) * alpha)
        for sid in ids
    }


class ArmDanceEngine:
    """
    Standing arm-wave loop.

    It only drives arm/head channels and keeps the legs at STANDING. L1 toggles
    between running the loop and returning smoothly to STANDING.
    """

    def __init__(
        self,
        dt: float = 0.04,
        period_s: float = 2.4,
        transition_s: float = 0.45,
        shoulder_pwm: int = 420,
        elbow_pwm: int = 260,
        lift_pwm: int = 820,
        head_pwm: int = 180,
        head_speed: float = 1.0,
        smooth_tau: float = 0.08,
        max_pwm_per_sec: float = 2200.0,
        min_step_pwm: int = 18,
    ) -> None:
        self.dt = dt
        self.period_s = max(0.8, period_s)
        self.transition_s = max(dt, transition_s)
        self.shoulder_pwm = abs(shoulder_pwm)
        self.elbow_pwm = abs(elbow_pwm)
        self.lift_pwm = abs(lift_pwm)
        self.head_pwm = abs(head_pwm)
        self.head_speed = max(0.25, abs(head_speed))
        self.smooth_tau = max(dt, smooth_tau)
        self.max_pwm_per_frame = max(1.0, abs(max_pwm_per_sec) * dt)
        self.min_step_pwm = max(0, int(min_step_pwm))
        self.mode = "off"
        self.phase_t = 0.0
        self.transition_t = 0.0
        self.start_pose = dict(STANDING)
        self.current_pose = dict(STANDING)

    @property
    def running(self) -> bool:
        return self.mode in {"starting", "loop", "returning"}

    @property
    def active(self) -> bool:
        return self.mode in {"starting", "loop"}

    def toggle(self) -> bool:
        if self.active:
            self.stop()
            return False
        self.start()
        return True

    def start(self) -> None:
        self.mode = "starting"
        self.transition_t = 0.0
        self.start_pose = dict(self.current_pose)

    def stop(self) -> None:
        self.mode = "returning"
        self.transition_t = 0.0
        self.start_pose = dict(self.current_pose)

    def reset(self) -> None:
        self.mode = "off"
        self.phase_t = 0.0
        self.transition_t = 0.0
        self.start_pose = dict(STANDING)
        self.current_pose = dict(STANDING)

    def _wave_keyframes(self) -> list[dict[int, int]]:
        lift = self.lift_pwm
        pose = dict(STANDING)
        pose[7] = _clamp_pwm(STANDING[7] + lift)
        pose[18] = _clamp_pwm(STANDING[18] - lift)

        left = dict(pose)
        left[8] = _clamp_pwm(STANDING[8] - self.shoulder_pwm)
        left[17] = _clamp_pwm(STANDING[17] - self.shoulder_pwm)
        left[6] = _clamp_pwm(STANDING[6] + self.elbow_pwm)
        left[19] = _clamp_pwm(STANDING[19] - self.elbow_pwm * 0.45)
        left[16] = _clamp_pwm(STANDING[16] - self.head_pwm)

        left_snap = dict(left)
        left_snap[6] = _clamp_pwm(STANDING[6] + self.elbow_pwm * 0.35)
        left_snap[19] = _clamp_pwm(STANDING[19] - self.elbow_pwm)

        right = dict(pose)
        right[8] = _clamp_pwm(STANDING[8] + self.shoulder_pwm)
        right[17] = _clamp_pwm(STANDING[17] + self.shoulder_pwm)
        right[6] = _clamp_pwm(STANDING[6] + self.elbow_pwm * 0.45)
        right[19] = _clamp_pwm(STANDING[19] - self.elbow_pwm)
        right[16] = _clamp_pwm(STANDING[16] + self.head_pwm)

        right_snap = dict(right)
        right_snap[6] = _clamp_pwm(STANDING[6] + self.elbow_pwm)
        right_snap[19] = _clamp_pwm(STANDING[19] - self.elbow_pwm * 0.35)
        return [left, left_snap, right, right_snap]

    def _loop_pose(self) -> dict[int, int]:
        frames = self._wave_keyframes()
        n = len(frames)
        phase = (self.phase_t / self.period_s) * n
        idx = int(phase) % n
        local_t = phase - int(phase)

        hold = 0.38
        if local_t < hold:
            return frames[idx]

        blend_t = (local_t - hold) / (1.0 - hold)
        return _blend_pose(frames[idx], frames[(idx + 1) % n], blend_t)

    def _filter_pose(self, target: dict[int, int]) -> dict[int, int]:
        alpha = 1.0 - math.exp(-self.dt / self.smooth_tau)
        out = dict(STANDING)

        for sid in ARM_DANCE_IDS:
            current = float(self.current_pose.get(sid, STANDING[sid]))
            desired = float(target.get(sid, STANDING[sid]))
            raw_delta = (desired - current) * alpha
            if abs(raw_delta) > self.max_pwm_per_frame:
                raw_delta = math.copysign(self.max_pwm_per_frame, raw_delta)

            remaining = desired - current
            if self.min_step_pwm > 0 and 0.0 < abs(raw_delta) < self.min_step_pwm:
                if abs(remaining) <= self.min_step_pwm:
                    raw_delta = remaining
                else:
                    raw_delta = 0.0

            out[sid] = _clamp_pwm(current + raw_delta)

        return out

    def update(self) -> dict[int, int]:
        if self.mode == "off":
            self.current_pose = dict(STANDING)
            return self.current_pose

        self.phase_t = (self.phase_t + self.dt) % self.period_s
        loop_pose = self._loop_pose()

        if self.mode == "starting":
            self.transition_t += self.dt
            target_pose = _blend_pose(self.start_pose, loop_pose, self.transition_t / self.transition_s)
            self.current_pose = self._filter_pose(target_pose)
            if self.transition_t >= self.transition_s:
                self.mode = "loop"
            return self.current_pose

        if self.mode == "returning":
            self.transition_t += self.dt
            target_pose = _blend_pose(self.start_pose, STANDING, self.transition_t / self.transition_s)
            self.current_pose = self._filter_pose(target_pose)
            if self.transition_t >= self.transition_s:
                self.reset()
            return self.current_pose

        self.current_pose = self._filter_pose(loop_pose)
        return self.current_pose
