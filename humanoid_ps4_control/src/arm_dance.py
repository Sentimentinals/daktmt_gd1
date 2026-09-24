from __future__ import annotations

from .config import STANDING


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

    It only drives arm/head channels and keeps the legs at STANDING. L toggles
    between running the loop and returning to STANDING.
    """

    def __init__(
        self,
        dt: float = 0.04,
        transition_s: float = 0.45,
        shoulder_pwm: int = 420,
        elbow_pwm: int = 420,
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
        pose[11] = round(STANDING[11] - left_shoulder)
        pose[24] = round(STANDING[24] + right_elbow)
        pose[9] = round(STANDING[9] - left_elbow)
        pose[25] = round(STANDING[25] + head)
        return pose

    def _dance_sequence(self) -> list[tuple[str, float, dict[int, int]]]:
        lift = self.lift_pwm
        shoulder = self.shoulder_pwm
        elbow = self.elbow_pwm
        head = self.head_pwm

        # Repeat the original six-pose dance before returning both arms to rest.
        legacy = [
            ("legacy", 0.85, self._arm_pose(lift * 0.72, lift * 0.72, shoulder * 0.65, shoulder * 0.65, elbow * 0.55, elbow * 0.55)),
            ("legacy", 0.85, self._arm_pose(lift * 0.82, lift * 0.55, shoulder, shoulder * 0.30, elbow * 0.15, elbow * 0.85, -head)),
            ("legacy", 0.85, self._arm_pose(lift * 0.55, lift * 0.82, shoulder * 0.30, shoulder, elbow * 0.85, elbow * 0.15, head)),
            ("legacy", 0.85, self._arm_pose(lift * 0.96, lift * 0.96, shoulder * 0.45, shoulder * 0.45, elbow * 0.20, elbow * 0.20)),
            ("legacy", 0.85, self._arm_pose(lift * 0.90, lift * 0.90, -shoulder * 0.75, -shoulder * 0.75, elbow * 0.90, elbow * 0.25, -head)),
            ("legacy", 0.85, self._arm_pose(lift * 0.90, lift * 0.90, shoulder * 0.75, shoulder * 0.75, elbow * 0.25, elbow * 0.90, head)),
        ] * 2 + [("legacy", 1.0, dict(STANDING))]

        reach = shoulder * 2.0
        curl = elbow * 1.7
        chest = self._arm_pose(lift * 0.15, lift * 0.15, reach, reach)
        tarzan = [("tarzan", 1.0, chest)] + [
            ("tarzan", 0.65, self._arm_pose(lift * 0.15, lift * 0.15, reach, reach, -curl, 0.0, -head * 0.25)),
            ("tarzan", 0.55, chest),
            ("tarzan", 0.65, self._arm_pose(lift * 0.15, lift * 0.15, reach, reach, 0.0, -curl, head * 0.25)),
            ("tarzan", 0.55, chest),
        ] * 3 + [("tarzan", 1.0, dict(STANDING))]

        # A high V needs more than 90 degrees of upper-arm travel from arms down.
        high_v = self._arm_pose(lift * 1.55, lift * 1.55, 0.0, 0.0)
        victory = [
            ("victory", 1.0, self._arm_pose(lift * 0.70, lift * 0.70, 0.0, 0.0)),
            ("victory", 1.6, high_v),
        ] + [
            ("victory", 0.8, self._arm_pose(lift * 1.10, lift * 1.10, 0.0, 0.0, elbow * 0.20, elbow * 0.20)),
            ("victory", 1.2, high_v),
        ] * 2 + [("victory", 1.6, dict(STANDING))]
        return legacy + tarzan + victory

    def _loop_pose(self) -> dict[int, int]:
        elapsed = 0.0
        for index, (section, duration, pose) in enumerate(self.sequence):
            if self.phase_t < elapsed + duration:
                self.section = section
                local_t = (self.phase_t - elapsed) / duration
                # Arrive, then hold; each section ends at rest before the next.
                blend_t = min(1.0, local_t / 0.60)
                blend_t = blend_t * blend_t * (3.0 - 2.0 * blend_t)
                previous_pose = self.sequence[index - 1][2]
                return _blend_pose(previous_pose, pose, blend_t)
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
