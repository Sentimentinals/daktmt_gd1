from __future__ import annotations

from .walking_engine import STANDING


def _blend_pose(a: dict[int, int], b: dict[int, int], t: float) -> dict[int, int]:
    alpha = max(0.0, min(1.0, t))
    ids = set(a) | set(b)
    return {
        sid: round(a.get(sid, STANDING.get(sid, 1500)) * (1.0 - alpha) + b.get(sid, STANDING.get(sid, 1500)) * alpha)
        for sid in ids
    }


class ArmDanceEngine:
    """
    Standing arm keyframe loop.

    It only drives arm/head channels and keeps the legs at STANDING. L/M toggles
    between running the loop and returning to STANDING.
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
    ) -> None:
        self.dt = dt
        self.period_s = max(0.8, period_s)
        self.transition_s = max(dt, transition_s)
        self.shoulder_pwm = abs(shoulder_pwm)
        self.elbow_pwm = abs(elbow_pwm)
        self.lift_pwm = abs(lift_pwm)
        self.head_pwm = abs(head_pwm)
        self.reset()

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
        pose[23] = round(STANDING[23] + right_lift)
        pose[10] = round(STANDING[10] - left_lift)
        pose[22] = round(STANDING[22] + right_shoulder)
        pose[11] = round(STANDING[11] + left_shoulder)
        pose[24] = round(STANDING[24] + right_elbow)
        pose[9] = round(STANDING[9] - left_elbow)
        pose[25] = round(STANDING[25] + head)
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

    def update(self) -> dict[int, int]:
        if self.mode == "off":
            self.current_pose = dict(STANDING)
            return self.current_pose

        self.phase_t = (self.phase_t + self.dt) % self.period_s
        loop_pose = self._loop_pose()

        if self.mode == "starting":
            self.transition_t += self.dt
            self.current_pose = _blend_pose(self.start_pose, loop_pose, self.transition_t / self.transition_s)
            if self.transition_t >= self.transition_s:
                self.mode = "loop"
            return self.current_pose

        if self.mode == "returning":
            self.transition_t += self.dt
            self.current_pose = _blend_pose(self.start_pose, STANDING, self.transition_t / self.transition_s)
            if self.transition_t >= self.transition_s:
                self.reset()
            return self.current_pose

        self.current_pose = loop_pose
        return self.current_pose
