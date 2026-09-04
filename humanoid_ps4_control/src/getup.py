from __future__ import annotations

from dataclasses import dataclass, field

from .walking_engine import STAND_ANG, STANDING, angle_to_pwm


JOINT_TO_SERVO = {
    "R_ankle_roll": (17, "hip_roll"),
    "R_ankle_pitch": (18, "R_ankle"),
    "R_knee": (19, "R_knee"),
    "R_hip_pitch": (20, "R_hip_pitch"),
    "R_hip_roll": (21, "R_hip_abduct"),
    "L_hip_roll": (12, "L_hip_abduct"),
    "L_hip_pitch": (13, "L_hip_pitch"),
    "L_knee": (14, "L_knee"),
    "L_ankle_pitch": (15, "L_ankle"),
    "L_ankle_roll": (16, "hip_roll"),
}


def _merge(*parts: dict[int, int]) -> dict[int, int]:
    out: dict[int, int] = {}
    for part in parts:
        out.update(part)
    return out


ARM_AUX_DOWN = {
    24: STANDING[24],
    23: STANDING[23],
    10: STANDING[10],
    9: STANDING[9],
}

ARM_STANDING = {
    9: STANDING[9],
    10: STANDING[10],
    11: STANDING[11],
    22: STANDING[22],
    23: STANDING[23],
    24: STANDING[24],
}

ARM_SHOULDER_POSES = {
    "front_reach": {22: 2100, 11: 900},
    "front_push": {22: 2500, 11: 500},
    "front_hold": {22: 2200, 11: 800},
}


def _arm_pose(name: str, extra: dict[int, int] | None = None) -> dict[int, int]:
    if name == "standing":
        base = dict(ARM_STANDING)
    else:
        base = _merge(ARM_SHOULDER_POSES[name], ARM_AUX_DOWN)
    if extra:
        base.update(extra)
    return base


def _blend_pose(a: dict[int, int], b: dict[int, int], t: float) -> dict[int, int]:
    alpha = max(0.0, min(1.0, t))
    ids = set(STANDING) | set(a) | set(b)
    return {
        sid: round(a.get(sid, STANDING.get(sid, 1500)) * (1.0 - alpha) + b.get(sid, STANDING.get(sid, 1500)) * alpha)
        for sid in ids
    }


@dataclass(frozen=True)
class GetupStep:
    label: str
    pose: dict[int, int]
    duration_s: float


@dataclass(frozen=True)
class GetupPoseState:
    label: str
    joint_angles: dict[str, float] = field(default_factory=dict)
    pwm_overrides: dict[int, int] = field(default_factory=dict)


def _scaled(duration_s: float, speed: float) -> float:
    return max(0.16, duration_s / max(0.2, speed))


def _leg_angles(
    *,
    r_ankle_roll: float = STAND_ANG["hip_roll"],
    r_ankle_pitch: float = STAND_ANG["R_ankle"],
    r_knee: float = STAND_ANG["R_knee"],
    r_hip_pitch: float = STAND_ANG["R_hip_pitch"],
    r_hip_roll: float = STAND_ANG["R_hip_abduct"],
    l_hip_roll: float = STAND_ANG["L_hip_abduct"],
    l_hip_pitch: float = STAND_ANG["L_hip_pitch"],
    l_knee: float = STAND_ANG["L_knee"],
    l_ankle_pitch: float = STAND_ANG["L_ankle"],
    l_ankle_roll: float = STAND_ANG["hip_roll"],
) -> dict[str, float]:
    return {
        "R_ankle_roll": r_ankle_roll,
        "R_ankle_pitch": r_ankle_pitch,
        "R_knee": r_knee,
        "R_hip_pitch": r_hip_pitch,
        "R_hip_roll": r_hip_roll,
        "L_hip_roll": l_hip_roll,
        "L_hip_pitch": l_hip_pitch,
        "L_knee": l_knee,
        "L_ankle_pitch": l_ankle_pitch,
        "L_ankle_roll": l_ankle_roll,
    }


def _symmetric_leg_angles(
    *,
    ankle_pitch: float,
    knee: float,
    hip_pitch: float,
    ankle_roll: float = STAND_ANG["hip_roll"],
    hip_roll: float = STAND_ANG["R_hip_abduct"],
) -> dict[str, float]:
    return _leg_angles(
        r_ankle_roll=ankle_roll,
        r_ankle_pitch=ankle_pitch,
        r_knee=knee,
        r_hip_pitch=hip_pitch,
        r_hip_roll=hip_roll,
        l_hip_roll=hip_roll,
        l_hip_pitch=hip_pitch,
        l_knee=knee,
        l_ankle_pitch=ankle_pitch,
        l_ankle_roll=ankle_roll,
    )


def _pose_from_state(state: GetupPoseState) -> dict[int, int]:
    pose = dict(STANDING)
    for joint_name, angle_deg in state.joint_angles.items():
        sid, base_key = JOINT_TO_SERVO[joint_name]
        pose[sid] = angle_to_pwm(sid, STAND_ANG[base_key], angle_deg, STANDING[sid])
    for sid, pwm in state.pwm_overrides.items():
        pose[sid] = round(pwm)
    return pose


def _step(state: GetupPoseState, duration_s: float, speed: float) -> GetupStep:
    return GetupStep(
        label=state.label,
        pose=_pose_from_state(state),
        duration_s=_scaled(duration_s, speed),
    )


def build_getup_sequence(speed: float = 0.7) -> list[GetupStep]:
    """Return the face-down stand-up sequence."""
    standing_angles = _leg_angles()
    front_tuck_angles = _symmetric_leg_angles(
        ankle_pitch=50.0,
        knee=-38.0,
        hip_pitch=-60.0,
    )
    plant_angles = _symmetric_leg_angles(
        ankle_pitch=12.0,
        knee=20.0,
        hip_pitch=8.0,
    )
    kneel_low_angles = _symmetric_leg_angles(
        ankle_pitch=2.0,
        knee=-8.0,
        hip_pitch=-10.0,
    )
    squat_deep_angles = _symmetric_leg_angles(
        ankle_pitch=4.0,
        knee=8.0,
        hip_pitch=4.0,
    )
    squat_high_angles = _symmetric_leg_angles(
        ankle_pitch=12.0,
        knee=24.0,
        hip_pitch=12.0,
    )

    states = {
        "front_arms_forward": GetupPoseState(
            "arms-forward",
            front_tuck_angles,
            _arm_pose("front_reach"),
        ),
        "front_push_floor": GetupPoseState(
            "push-floor",
            front_tuck_angles,
            _arm_pose("front_push"),
        ),
        "front_plant_knees": GetupPoseState(
            "plant-knees",
            plant_angles,
            _arm_pose("front_push"),
        ),
        "front_kneel_low": GetupPoseState(
            "kneel-low",
            kneel_low_angles,
            _arm_pose("front_push"),
        ),
        "front_squat_deep": GetupPoseState(
            "squat-deep",
            squat_deep_angles,
            _arm_pose("front_hold", {24: 1520, 23: 800, 10: 2160, 9: 1480}),
        ),
        "front_squat_high": GetupPoseState(
            "squat-high",
            squat_high_angles,
            _arm_pose("front_hold", {24: 1520, 23: 660, 10: 2300, 9: 1480}),
        ),
        "front_arms_down": GetupPoseState(
            "arms-down",
            standing_angles,
            _arm_pose("standing"),
        ),
        "front_stand": GetupPoseState(
            "stand",
            standing_angles,
            _arm_pose("front_hold", {24: 1520, 23: 660, 10: 2300, 9: 1480}),
        ),
    }

    plan = [
        (states["front_arms_forward"], 0.95),
        (states["front_push_floor"], 0.55),
        (states["front_plant_knees"], 0.55),
        (states["front_kneel_low"], 0.65),
        (states["front_squat_deep"], 0.38),
        (states["front_squat_high"], 0.30),
        (states["front_stand"], 0.38),
        (states["front_arms_down"], 0.45),
    ]

    return [_step(state, duration, speed) for state, duration in plan]


class GetupEngine:
    def __init__(self, dt: float = 0.04, speed: float = 0.7) -> None:
        self.dt = dt
        self.speed = speed
        self.steps = build_getup_sequence(speed)
        self.reset()

    @property
    def running(self) -> bool:
        return self._running

    @property
    def label(self) -> str:
        if not self._running:
            return "off"
        return self.steps[self.step_index].label

    def reset(self) -> None:
        self._running = False
        self.step_index = 0
        self.step_t = 0.0
        self.step_start_pose = dict(STANDING)
        self.current_pose = dict(STANDING)

    def start(self, current_pose: dict[int, int] | None = None) -> str:
        self._running = True
        self.step_index = 0
        self.step_t = 0.0
        self.step_start_pose = dict(current_pose or self.current_pose or STANDING)
        self.current_pose = dict(self.step_start_pose)
        return self.label

    def update(self) -> dict[int, int]:
        if not self._running:
            self.current_pose = dict(STANDING)
            return self.current_pose

        step = self.steps[self.step_index]
        self.step_t += self.dt
        t = self.step_t / step.duration_s
        self.current_pose = _blend_pose(self.step_start_pose, step.pose, t)

        if t >= 1.0:
            self.step_index += 1
            self.step_t = 0.0
            self.step_start_pose = dict(self.current_pose)
            if self.step_index >= len(self.steps):
                self.reset()
                return dict(STANDING)

        return self.current_pose
