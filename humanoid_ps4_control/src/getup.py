from __future__ import annotations

from dataclasses import dataclass, field

from .walking_engine import STAND_ANG, STANDING, angle_to_pwm


JOINT_TO_SERVO = {
    "R_ankle_roll": (1, "hip_roll"),
    "R_ankle_pitch": (2, "R_ankle"),
    "R_knee": (3, "R_knee"),
    "R_hip_pitch": (4, "R_hip_pitch"),
    "R_hip_roll": (5, "R_hip_abduct"),
    "L_hip_roll": (20, "L_hip_abduct"),
    "L_hip_pitch": (21, "L_hip_pitch"),
    "L_knee": (22, "L_knee"),
    "L_ankle_pitch": (23, "L_ankle"),
    "L_ankle_roll": (24, "hip_roll"),
}


def _clamp_pwm(value: float) -> int:
    return max(500, min(2500, round(value)))


def _smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def _ease(t: float, curve: str) -> float:
    t = max(0.0, min(1.0, t))
    if curve == "linear":
        return t
    if curve == "snap":
        return 1.0 - (1.0 - t) ** 3
    return _smoothstep(t)


def _merge(*parts: dict[int, int]) -> dict[int, int]:
    out: dict[int, int] = {}
    for part in parts:
        out.update(part)
    return out


ARM_AUX_DOWN = {
    6: STANDING[6],
    7: STANDING[7],
    18: STANDING[18],
    19: STANDING[19],
}

ARM_STANDING = {
    6: STANDING[6],
    7: STANDING[7],
    8: STANDING[8],
    17: STANDING[17],
    18: STANDING[18],
    19: STANDING[19],
}

ARM_SHOULDER_POSES = {
    "back_reach": {8: 900, 17: 2100},
    "back_push": {8: 500, 17: 2500},
    "back_hold": {8: 800, 17: 2200},
    "front_reach": {8: 2100, 17: 900},
    "front_push": {8: 2500, 17: 500},
    "front_hold": {8: 2200, 17: 800},
}


def _arm_pose(name: str, extra: dict[int, int] | None = None) -> dict[int, int]:
    if name == "standing":
        base = dict(ARM_STANDING)
    else:
        base = _merge(ARM_SHOULDER_POSES[name], ARM_AUX_DOWN)
    if extra:
        base.update(extra)
    return base


def _blend_pose(a: dict[int, int], b: dict[int, int], t: float, curve: str = "smooth") -> dict[int, int]:
    alpha = _ease(t, curve)
    ids = set(STANDING) | set(a) | set(b)
    return {
        sid: _clamp_pwm(a.get(sid, STANDING.get(sid, 1500)) * (1.0 - alpha) + b.get(sid, STANDING.get(sid, 1500)) * alpha)
        for sid in ids
    }


@dataclass(frozen=True)
class GetupStep:
    label: str
    pose: dict[int, int]
    duration_s: float
    curve: str = "smooth"
    contacts: tuple[str, ...] = ()


@dataclass(frozen=True)
class GetupPoseState:
    label: str
    contacts: tuple[str, ...]
    joint_angles: dict[str, float] = field(default_factory=dict)
    pwm_overrides: dict[int, int] = field(default_factory=dict)


def _scaled(duration_s: float, speed: float) -> float:
    return max(0.16, duration_s / max(0.2, speed))


def _leg_angles(
    *,
    r_ankle_roll: float = 0.0,
    r_ankle_pitch: float = STAND_ANG["R_ankle"],
    r_knee: float = STAND_ANG["R_knee"],
    r_hip_pitch: float = STAND_ANG["R_hip_pitch"],
    r_hip_roll: float = STAND_ANG["R_hip_abduct"],
    l_hip_roll: float = STAND_ANG["L_hip_abduct"],
    l_hip_pitch: float = STAND_ANG["L_hip_pitch"],
    l_knee: float = STAND_ANG["L_knee"],
    l_ankle_pitch: float = STAND_ANG["L_ankle"],
    l_ankle_roll: float = 0.0,
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
    ankle_roll: float = 0.0,
    hip_roll: float = 0.0,
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
        pose[sid] = _clamp_pwm(pwm)
    return pose


def _step(state: GetupPoseState, duration_s: float, curve: str, speed: float) -> GetupStep:
    return GetupStep(
        label=state.label,
        pose=_pose_from_state(state),
        duration_s=_scaled(duration_s, speed),
        curve=curve,
        contacts=state.contacts,
    )


def build_getup_sequence(mode: str = "back", speed: float = 0.7) -> list[GetupStep]:
    """Return an open-loop recovery sequence.

    `back` assumes the robot is lying face-up. `front` assumes face-down.
    The back sequence uses a short power-rise phase because the upper body is
    heavy; standing up too slowly loses momentum and stability.
    """
    mode = mode.lower().strip()
    if mode not in {"back", "front"}:
        raise ValueError("getup mode must be 'back' or 'front'")

    standing_angles = _leg_angles()
    box_clear_angles = _symmetric_leg_angles(
        ankle_pitch=30.6,
        knee=-2.7,
        hip_pitch=-23.4,
    )
    tuck_angles = _symmetric_leg_angles(
        ankle_pitch=41.4,
        knee=-27.0,
        hip_pitch=-39.6,
    )
    snap_angles = _symmetric_leg_angles(
        ankle_pitch=23.4,
        knee=10.8,
        hip_pitch=0.0,
    )
    plant_angles = _symmetric_leg_angles(
        ankle_pitch=34.2,
        knee=-15.3,
        hip_pitch=-19.8,
    )
    kneel_low_angles = _symmetric_leg_angles(
        ankle_pitch=30.6,
        knee=-10.8,
        hip_pitch=-10.8,
    )
    squat_deep_angles = _symmetric_leg_angles(
        ankle_pitch=27.0,
        knee=1.8,
        hip_pitch=0.0,
    )
    squat_high_angles = _symmetric_leg_angles(
        ankle_pitch=21.6,
        knee=19.8,
        hip_pitch=9.0,
    )

    states = {
        "back_box_clear": GetupPoseState(
            "box-clear",
            ("back_case", "feet_light"),
            box_clear_angles,
            _arm_pose("back_reach"),
        ),
        "back_load_knees": GetupPoseState(
            "load-knees",
            ("back_case", "hands", "feet"),
            tuck_angles,
            _arm_pose("back_push"),
        ),
        "back_leg_snap": GetupPoseState(
            "leg-snap",
            ("hands", "feet"),
            snap_angles,
            _arm_pose("back_push"),
        ),
        "back_plant_feet": GetupPoseState(
            "plant-feet",
            ("hands", "feet"),
            plant_angles,
            _arm_pose("back_push"),
        ),
        "back_power_stand": GetupPoseState(
            "power-stand",
            ("feet",),
            standing_angles,
            _arm_pose("standing"),
        ),
        "back_stand_hold": GetupPoseState(
            "stand-hold",
            ("feet",),
            standing_angles,
            {},
        ),
        "front_arms_forward": GetupPoseState(
            "arms-forward",
            ("chest", "hands", "knees"),
            tuck_angles,
            _arm_pose("front_reach"),
        ),
        "front_push_floor": GetupPoseState(
            "push-floor",
            ("hands", "knees", "feet"),
            tuck_angles,
            _arm_pose("front_push"),
        ),
        "front_plant_knees": GetupPoseState(
            "plant-knees",
            ("hands", "knees", "feet"),
            plant_angles,
            _arm_pose("front_push"),
        ),
        "front_kneel_low": GetupPoseState(
            "kneel-low",
            ("hands", "knees", "feet"),
            kneel_low_angles,
            _arm_pose("front_hold", {7: 980, 18: 1970}),
        ),
        "front_squat_deep": GetupPoseState(
            "squat-deep",
            ("hands_light", "feet"),
            squat_deep_angles,
            _arm_pose("front_hold", {6: 1520, 7: 800, 18: 2160, 19: 1480}),
        ),
        "front_squat_high": GetupPoseState(
            "squat-high",
            ("feet",),
            squat_high_angles,
            _arm_pose("front_hold", {6: 1520, 7: 660, 18: 2300, 19: 1480}),
        ),
        "front_arms_down": GetupPoseState(
            "arms-down",
            ("feet",),
            squat_high_angles,
            _arm_pose("standing"),
        ),
        "front_stand": GetupPoseState(
            "stand",
            ("feet",),
            standing_angles,
            {},
        ),
    }

    if mode == "back":
        plan = [
            (states["back_box_clear"], 0.55, "smooth"),
            (states["back_load_knees"], 0.45, "smooth"),
            (states["back_leg_snap"], 0.26, "linear"),
            (states["back_plant_feet"], 0.18, "linear"),
            (states["back_power_stand"], 0.22, "snap"),
            (states["back_stand_hold"], 0.40, "linear"),
        ]
    else:
        plan = [
            (states["front_arms_forward"], 1.0, "smooth"),
            (states["front_push_floor"], 1.2, "smooth"),
            (states["front_plant_knees"], 0.8, "smooth"),
            (states["front_kneel_low"], 1.3, "smooth"),
            (states["front_squat_deep"], 1.1, "smooth"),
            (states["front_squat_high"], 1.0, "smooth"),
            (states["front_arms_down"], 0.45, "smooth"),
            (states["front_stand"], 1.35, "smooth"),
        ]

    return [_step(state, duration, curve, speed) for state, duration, curve in plan]


class GetupEngine:
    def __init__(self, dt: float = 0.04, mode: str = "back", speed: float = 0.7) -> None:
        self.dt = dt
        self.mode_name = mode
        self.speed = speed
        self.steps = build_getup_sequence(mode, speed)
        self.reset()

    @property
    def running(self) -> bool:
        return self._running

    @property
    def label(self) -> str:
        if not self._running:
            return "off"
        return self.steps[self.step_index].label

    @property
    def contacts(self) -> tuple[str, ...]:
        if not self._running:
            return ()
        return self.steps[self.step_index].contacts

    def reset(self) -> None:
        self._running = False
        self.step_index = 0
        self.step_t = 0.0
        self.step_start_pose = dict(STANDING)
        self.current_pose = dict(STANDING)

    def start(self, current_pose: dict[int, int] | None = None, mode: str | None = None) -> str:
        if mode is not None and mode != self.mode_name:
            self.mode_name = mode
            self.steps = build_getup_sequence(self.mode_name, self.speed)
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
        self.current_pose = _blend_pose(self.step_start_pose, step.pose, t, step.curve)

        if t >= 1.0:
            self.step_index += 1
            self.step_t = 0.0
            self.step_start_pose = dict(self.current_pose)
            if self.step_index >= len(self.steps):
                self.reset()
                return dict(STANDING)

        return self.current_pose
