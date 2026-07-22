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
    Standing arm keyframe loop.

    It only drives arm/head channels and keeps the legs at STANDING. L/M toggles
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

    def _arm_pose(
        self,
        right_lift: float,
        left_lift: float,
        right_shoulder: float,
        left_shoulder: float,
        right_elbow: float = 0.0,
        left_elbow: float = 0.0,
        head: float = 0.0,
    ) -> dict[int, int]:
        pose = dict(STANDING)
        pose[7] = _clamp_pwm(STANDING[7] + right_lift)
        pose[18] = _clamp_pwm(STANDING[18] - left_lift)
        pose[8] = _clamp_pwm(STANDING[8] + right_shoulder)
        pose[17] = _clamp_pwm(STANDING[17] + left_shoulder)
        pose[6] = _clamp_pwm(STANDING[6] + right_elbow)
        pose[19] = _clamp_pwm(STANDING[19] - left_elbow)
        pose[16] = _clamp_pwm(STANDING[16] + head)
        return pose

    def _dance_keyframes(self) -> list[dict[int, int]]:
        lift = self.lift_pwm
        shoulder = self.shoulder_pwm
        elbow = self.elbow_pwm
        head = self.head_pwm

        return [
            self._arm_pose(lift * 0.72, lift * 0.72, shoulder * 0.65, shoulder * 0.65, elbow * 0.55, elbow * 0.55, 0),
            self._arm_pose(lift * 0.82, lift * 0.55, shoulder * 1.00, shoulder * 0.30, elbow * 0.15, elbow * 0.85, -head),
            self._arm_pose(lift * 0.55, lift * 0.82, shoulder * 0.30, shoulder * 1.00, elbow * 0.85, elbow * 0.15, head),
            self._arm_pose(lift * 0.96, lift * 0.96, shoulder * 0.45, shoulder * 0.45, elbow * 0.20, elbow * 0.20, 0),
            self._arm_pose(lift * 0.90, lift * 0.90, -shoulder * 0.75, -shoulder * 0.75, elbow * 0.90, elbow * 0.25, -head),
            self._arm_pose(lift * 0.90, lift * 0.90, shoulder * 0.75, shoulder * 0.75, elbow * 0.25, elbow * 0.90, head),
        ]

    def _loop_pose(self) -> dict[int, int]:
        frames = self._dance_keyframes()
        n = len(frames)
        phase = (self.phase_t / self.period_s) * n
        idx = int(phase) % n
        local_t = phase - int(phase)

        hold = 0.56
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
                    raw_delta = math.copysign(self.min_step_pwm, remaining)

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


class HandshakeEngine:
    """Offer the right hand, detect a grip, shake, then return to standing."""

    def __init__(
        self,
        dt: float = 0.04,
        offer_s: float = 0.75,
        contact_timeout_s: float = 8.0,
        release_timeout_s: float = 3.0,
        frequency_hz: float = 2.2,
        cycles: int = 4,
        lift_pwm: int = 500,
        shoulder_pwm: int = 260,
        elbow_pwm: int = 260,
        shake_pwm: int = 75,
        contact_threshold: float = 0.12,
        release_threshold: float = 0.06,
        stable_frames: int = 3,
    ) -> None:
        self.dt = max(0.01, dt)
        self.offer_s = max(self.dt, offer_s)
        self.contact_timeout_s = max(self.dt, contact_timeout_s)
        self.release_timeout_s = max(self.dt, release_timeout_s)
        self.frequency_hz = max(0.5, frequency_hz)
        self.cycles = max(1, cycles)
        self.lift_pwm = abs(lift_pwm)
        self.shoulder_pwm = shoulder_pwm
        self.elbow_pwm = elbow_pwm
        self.shake_pwm = abs(shake_pwm)
        self.contact_threshold = max(0.0, min(1.0, contact_threshold))
        self.release_threshold = max(0.0, min(self.contact_threshold, release_threshold))
        self.stable_frames = max(1, stable_frames)
        self.reset()

    @property
    def running(self) -> bool:
        return self.mode != "off"

    @property
    def status(self) -> str:
        return {
            "off": "OFF",
            "offering": "OFFERING HAND",
            "waiting": "WAITING FOR GRIP",
            "shaking": "SHAKING",
            "releasing": "WAITING FOR RELEASE",
            "returning": "RETURNING",
        }[self.mode]

    def reset(self) -> None:
        self.mode = "off"
        self.elapsed_s = 0.0
        self.contact_frames = 0
        self.release_frames = 0
        self.start_pose = dict(STANDING)
        self.current_pose = dict(STANDING)

    def start(self, current_pose: dict[int, int] | None = None) -> None:
        self.mode = "offering"
        self.elapsed_s = 0.0
        self.contact_frames = 0
        self.release_frames = 0
        self.start_pose = dict(STANDING if current_pose is None else current_pose)
        self.current_pose = dict(self.start_pose)

    def cancel(self) -> None:
        if self.running:
            self._enter_returning()

    def _offer_pose(self, shake_offset: float = 0.0) -> dict[int, int]:
        pose = dict(STANDING)
        pose[7] = _clamp_pwm(STANDING[7] + self.lift_pwm + shake_offset)
        pose[8] = _clamp_pwm(STANDING[8] + self.shoulder_pwm)
        pose[6] = _clamp_pwm(STANDING[6] + self.elbow_pwm - shake_offset * 0.25)
        return pose

    def _enter_returning(self) -> None:
        self.mode = "returning"
        self.elapsed_s = 0.0
        self.start_pose = dict(self.current_pose)

    def update(self, hand_force: float | None) -> dict[int, int]:
        if self.mode == "off":
            return dict(STANDING)

        self.elapsed_s += self.dt
        offer_pose = self._offer_pose()

        if self.mode == "offering":
            self.current_pose = _blend_pose(self.start_pose, offer_pose, self.elapsed_s / self.offer_s)
            if self.elapsed_s >= self.offer_s:
                self.mode = "waiting"
                self.elapsed_s = 0.0
            return dict(self.current_pose)

        if self.mode == "waiting":
            self.current_pose = offer_pose
            self.contact_frames = self.contact_frames + 1 if (
                hand_force is not None and hand_force >= self.contact_threshold
            ) else 0
            if self.contact_frames >= self.stable_frames:
                self.mode = "shaking"
                self.elapsed_s = 0.0
            elif self.elapsed_s >= self.contact_timeout_s:
                self._enter_returning()
            return dict(self.current_pose)

        if self.mode == "shaking":
            shake_duration_s = self.cycles / self.frequency_hz
            phase = 2.0 * math.pi * self.frequency_hz * min(self.elapsed_s, shake_duration_s)
            self.current_pose = self._offer_pose(self.shake_pwm * math.sin(phase))
            if self.elapsed_s >= shake_duration_s:
                self.mode = "releasing"
                self.elapsed_s = 0.0
            return dict(self.current_pose)

        if self.mode == "releasing":
            self.current_pose = offer_pose
            self.release_frames = self.release_frames + 1 if (
                hand_force is not None and hand_force <= self.release_threshold
            ) else 0
            if self.release_frames >= self.stable_frames or self.elapsed_s >= self.release_timeout_s:
                self._enter_returning()
            return dict(self.current_pose)

        self.current_pose = _blend_pose(self.start_pose, STANDING, self.elapsed_s / self.offer_s)
        if self.elapsed_s >= self.offer_s:
            self.reset()
        return dict(self.current_pose)
