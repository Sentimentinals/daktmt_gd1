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
    Standing arm show: legacy dance, Tarzan chest beats, then victory poses.

    It only drives arm/head channels and keeps the legs at STANDING. L/M toggles
    between running the loop and returning to STANDING.
    """

    def __init__(
        self,
        dt: float = 0.04,
        transition_s: float = 0.45,
        shoulder_pwm: int = 420,
        elbow_pwm: int = 260,
        lift_pwm: int = 820,
        head_pwm: int = 180,
    ) -> None:
        self.dt = dt
        self.transition_s = max(dt, transition_s)
        self.shoulder_pwm = abs(shoulder_pwm)
        self.elbow_pwm = abs(elbow_pwm)
        self.lift_pwm = abs(lift_pwm)
        self.head_pwm = abs(head_pwm)
        self.sequence = self._dance_sequence()
        self.loop_duration = sum(duration for _, duration, _ in self.sequence)
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
        self.section = "legacy"
        self.phase_t = 0.0
        self.transition_t = 0.0
        self.start_pose = dict(self.current_pose)

    def stop(self) -> None:
        self.mode = "returning"
        self.section = "return"
        self.transition_t = 0.0
        self.start_pose = dict(self.current_pose)

    def reset(self) -> None:
        self.mode = "off"
        self.section = "off"
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

    def _dance_sequence(self) -> list[tuple[str, float, dict[int, int]]]:
        lift = self.lift_pwm
        shoulder = self.shoulder_pwm
        elbow = self.elbow_pwm
        head = self.head_pwm

        return [
            ("legacy", 0.70, self._arm_pose(lift * 0.72, lift * 0.72, shoulder * 0.65, shoulder * 0.65, elbow * 0.55, elbow * 0.55)),
            ("legacy", 0.70, self._arm_pose(lift * 0.82, lift * 0.55, shoulder, shoulder * 0.30, elbow * 0.15, elbow * 0.85, -head)),
            ("legacy", 0.70, self._arm_pose(lift * 0.55, lift * 0.82, shoulder * 0.30, shoulder, elbow * 0.85, elbow * 0.15, head)),
            ("legacy", 0.75, self._arm_pose(lift * 0.96, lift * 0.96, shoulder * 0.45, shoulder * 0.45, elbow * 0.20, elbow * 0.20)),
            ("legacy", 0.65, self._arm_pose(lift * 0.90, lift * 0.90, -shoulder * 0.75, -shoulder * 0.75, elbow * 0.90, elbow * 0.25, -head)),
            ("legacy", 0.65, self._arm_pose(lift * 0.90, lift * 0.90, shoulder * 0.75, shoulder * 0.75, elbow * 0.25, elbow * 0.90, head)),
            ("tarzan", 0.65, self._arm_pose(lift, lift, shoulder * 0.50, shoulder * 0.50, elbow * 0.10, elbow * 0.10)),
            ("tarzan", 0.40, self._arm_pose(lift * 0.84, lift * 0.84, shoulder * 0.12, shoulder * 0.12, elbow, elbow * 0.45, -head * 0.45)),
            ("tarzan", 0.30, self._arm_pose(lift * 0.90, lift * 0.90, shoulder * 0.30, shoulder * 0.30, elbow * 0.15, elbow * 0.15)),
            ("tarzan", 0.40, self._arm_pose(lift * 0.84, lift * 0.84, shoulder * 0.12, shoulder * 0.12, elbow * 0.45, elbow, head * 0.45)),
            ("tarzan", 0.30, self._arm_pose(lift * 0.90, lift * 0.90, shoulder * 0.30, shoulder * 0.30, elbow * 0.15, elbow * 0.15)),
            ("tarzan", 0.40, self._arm_pose(lift * 0.84, lift * 0.84, shoulder * 0.12, shoulder * 0.12, elbow, elbow * 0.45, -head * 0.45)),
            ("tarzan", 0.30, self._arm_pose(lift * 0.90, lift * 0.90, shoulder * 0.30, shoulder * 0.30, elbow * 0.15, elbow * 0.15)),
            ("tarzan", 0.40, self._arm_pose(lift * 0.84, lift * 0.84, shoulder * 0.12, shoulder * 0.12, elbow * 0.45, elbow, head * 0.45)),
            ("victory", 0.70, self._arm_pose(lift, lift, shoulder * 0.55, shoulder * 0.55, elbow * 0.12, elbow * 0.12)),
            ("victory", 0.75, self._arm_pose(lift, lift * 0.68, shoulder * 0.70, shoulder * 0.10, elbow * 0.18, elbow * 0.65, -head)),
            ("victory", 0.45, self._arm_pose(lift * 0.88, lift * 0.88, shoulder * 0.35, shoulder * 0.35, elbow * 0.10, elbow * 0.10)),
            ("victory", 0.75, self._arm_pose(lift * 0.68, lift, shoulder * 0.10, shoulder * 0.70, elbow * 0.65, elbow * 0.18, head)),
            ("victory", 0.45, self._arm_pose(lift * 0.88, lift * 0.88, shoulder * 0.35, shoulder * 0.35, elbow * 0.10, elbow * 0.10)),
            ("victory", 0.85, self._arm_pose(lift, lift, shoulder * 0.55, shoulder * 0.55, elbow * 0.12, elbow * 0.12)),
        ]

    def _loop_pose(self) -> dict[int, int]:
        elapsed = 0.0
        for index, (section, duration, pose) in enumerate(self.sequence):
            if self.phase_t < elapsed + duration:
                self.section = section
                local_t = (self.phase_t - elapsed) / duration
                if local_t <= 0.35:
                    return pose
                blend_t = (local_t - 0.35) / 0.65
                blend_t = blend_t * blend_t * (3.0 - 2.0 * blend_t)
                next_pose = self.sequence[(index + 1) % len(self.sequence)][2]
                return _blend_pose(pose, next_pose, blend_t)
            elapsed += duration
        return self.sequence[-1][2]

    def update(self) -> dict[int, int]:
        if self.mode == "off":
            self.current_pose = dict(STANDING)
            return self.current_pose

        if self.mode == "returning":
            self.transition_t += self.dt
            self.current_pose = _blend_pose(self.start_pose, STANDING, self.transition_t / self.transition_s)
            if self.transition_t >= self.transition_s:
                self.reset()
            return self.current_pose

        loop_pose = self._loop_pose()
        self.phase_t = (self.phase_t + self.dt) % self.loop_duration
        if self.mode == "starting":
            self.transition_t += self.dt
            self.current_pose = _blend_pose(self.start_pose, loop_pose, self.transition_t / self.transition_s)
            if self.transition_t >= self.transition_s:
                self.mode = "loop"
            return self.current_pose

        self.current_pose = loop_pose
        return self.current_pose
