from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.getup import build_getup_sequence  # noqa: E402
from src.leg_fk import leg_fk  # noqa: E402
from src.leg_ik import leg_ik  # noqa: E402
from src.walking_engine import (  # noqa: E402
    DynamicWalkingEngine,
    ROBOT,
    STAND_ANG,
    STANDING,
    angle_to_pwm,
    compute_pose,
)
from src.zmp_controller import ZMPPreviewController  # noqa: E402


def assert_close(name: str, actual: float, limit: float) -> None:
    if abs(actual) > limit:
        raise AssertionError(f"{name}: error={actual:.6f}, limit={limit:.6f}")


def validate_leg_kinematics() -> None:
    l1, l2 = 80.0, 75.0
    cases = [
        ((0.0, 0.0, 147.4), (0.0, 0.0, 0.0)),
        ((0.0, 28.0, 147.4), (5.0, 28.0, 8.0)),
        ((0.0, -28.0, 147.4), (-8.0, -22.0, 4.0)),
        ((0.0, 28.0, 147.4), (12.0, 20.0, 0.0)),
    ]

    for idx, (hip, foot) in enumerate(cases):
        angles = leg_ik(hip, foot, l1, l2)
        fk_pos = leg_fk(angles, l1, l2)
        target = np.array(foot) - np.array(hip)
        err = float(np.linalg.norm(fk_pos - target))
        assert_close(f"IK/FK case {idx}", err, 1e-6)

    hw = ROBOT["half_hip"]
    zc = ROBOT["com_height"]
    right_stand = leg_ik((0.0, hw, zc), (0.0, hw, 0.0), l1, l2)
    left_stand = leg_ik((0.0, -hw, zc), (0.0, -hw, 0.0), l1, l2)
    standing_checks = [
        ("R hip pitch baseline", right_stand["hip_pitch"] - STAND_ANG["R_hip_pitch"]),
        ("R knee baseline", right_stand["knee"] - STAND_ANG["R_knee"]),
        ("R ankle baseline", right_stand["ankle_pitch"] - STAND_ANG["R_ankle"]),
        ("L hip pitch baseline", left_stand["hip_pitch"] - STAND_ANG["L_hip_pitch"]),
        ("L knee baseline", left_stand["knee"] - STAND_ANG["L_knee"]),
        ("L ankle baseline", left_stand["ankle_pitch"] - STAND_ANG["L_ankle"]),
    ]
    for name, error in standing_checks:
        assert_close(name, error, 1.0)


def validate_zmp_preview() -> None:
    ctrl = ZMPPreviewController(dt=0.04, zc=147.4, preview_steps=24)
    values = []
    for i in range(140):
        ref = 15.0 if i > 10 else 0.0
        values.append(ctrl.step(ref, [ref] * 24))

    steady_error = values[-1] - 15.0
    assert_close("ZMP preview steady-state", steady_error, 1e-3)
    if not all(math.isfinite(v) for v in values):
        raise AssertionError("ZMP preview produced non-finite CoM values")
    if max(abs(v) for v in values) > 40.0:
        raise AssertionError("ZMP preview response exceeds conservative bound")


def validate_getup_sequences() -> None:
    for mode in ("front", "back"):
        seq = build_getup_sequence(mode, speed=0.6)
        if not seq:
            raise AssertionError(f"{mode} get-up sequence is empty")
        if seq[-1].pose != STANDING:
            raise AssertionError(f"{mode} get-up sequence does not end at STANDING")
        for step in seq:
            if not step.contacts:
                raise AssertionError(f"{mode}/{step.label}: missing contact state")
            for sid, pwm in step.pose.items():
                if not 500 <= pwm <= 2500:
                    raise AssertionError(f"{mode}/{step.label}: servo {sid} out of bounds: {pwm}")
            if step.duration_s <= 0.0:
                raise AssertionError(f"{mode}/{step.label}: invalid duration {step.duration_s}")


def validate_walking_settle() -> None:
    engine = DynamicWalkingEngine(dt=0.04, arm_swing_pwm=240)
    if not engine.is_idle_ready():
        raise AssertionError("Walking engine is not idle-ready after reset")

    if abs(engine._lift_profile(0.0)) > 1e-9:
        raise AssertionError("Swing foot lifts before the support-shift phase")
    if abs(engine._lift_profile(engine.lift_start_phase * 0.9)) > 1e-9:
        raise AssertionError("Swing foot lifts too early")
    if engine._lift_profile(0.5) < 0.9:
        raise AssertionError("Swing foot peak lift profile is too low")
    if abs(engine._lift_profile(1.0)) > 1e-9:
        raise AssertionError("Swing foot does not land by the end of the phase")

    engine_lift = DynamicWalkingEngine(dt=0.04, step_x_ratio=0.0, step_height=20.0, thigh_lift_forward_mm=18.0)
    lift_pose = None
    for _ in range(40):
        lift_pose = engine_lift.update(0.16)
        if engine_lift.last_swing_leg == "left" and engine_lift.last_lift_factor > 0.95:
            break
    if lift_pose is None:
        raise AssertionError("Walking engine did not produce lift poses")
    if engine_lift.last_foot_L[2] < 19.0:
        raise AssertionError("Swing foot target does not reach the commanded lift height")
    if engine_lift.last_foot_L[0] < 14.0:
        raise AssertionError("Swing foot forward bias does not engage thigh-lift trajectory")
    if abs(lift_pose[21] - STANDING[21]) < 80 or abs(lift_pose[22] - STANDING[22]) < 80:
        raise AssertionError("IK foot lift does not visibly engage hip and knee servos")
    if abs(engine_lift.last_foot_R[0]) > 1e-9:
        raise AssertionError("Support foot should not slide in lift-first gait")

    low_speed_engine = DynamicWalkingEngine(dt=0.04, t_step=0.95, t_dbl=0.16, arm_swing_pwm=0)
    decisive_pose = None
    shift_pose = None
    for _ in range(40):
        decisive_pose = low_speed_engine.update(0.06)
        if (
            low_speed_engine.last_swing_leg == "left"
            and low_speed_engine.support_leg == "right"
            and low_speed_engine.last_lift_factor <= 0.02
        ):
            shift_score = (
                abs(decisive_pose[1] - STANDING[1])
                + abs(decisive_pose[5] - STANDING[5])
            )
            if shift_pose is None or shift_score > shift_pose[0]:
                shift_pose = (shift_score, dict(decisive_pose))
        if low_speed_engine.last_swing_leg == "left" and low_speed_engine.last_lift_factor > 0.95:
            break
    if shift_pose is None:
        raise AssertionError("Low-speed gait does not produce a pre-lift support shift")
    right_support_hip = abs(shift_pose[1][5] - STANDING[5])
    right_support_ankle = abs(shift_pose[1][1] - STANDING[1])
    if right_support_hip < 40:
        raise AssertionError("Pre-lift support hip counter-roll is too small")
    if right_support_ankle < 90:
        raise AssertionError("Pre-lift support ankle roll is too small to unload the swing foot")
    if right_support_ankle <= right_support_hip:
        raise AssertionError("Pre-lift support shift should be ankle-led, not hip-led")
    if abs(shift_pose[1][20] - STANDING[20]) > 15:
        raise AssertionError("Pre-lift swing hip roll should stay near neutral")
    if abs(shift_pose[1][24] - STANDING[24]) > 15:
        raise AssertionError("Pre-lift left swing ankle roll should stay neutral")
    if shift_pose[1][5] <= STANDING[5]:
        raise AssertionError("Pre-lift hip roll direction is reversed for right-support shift")
    if shift_pose[1][4] < STANDING[4] + 40 or shift_pose[1][21] < STANDING[21] + 40:
        raise AssertionError("Pre-lift support shift should preserve balanced forward lean on both thighs")
    if decisive_pose is None or low_speed_engine.last_lift_factor <= 0.95:
        raise AssertionError("Low-speed gait does not reach a clear swing-lift phase")
    if low_speed_engine.last_foot_L[2] < 27.0:
        raise AssertionError("Low-speed gait swing target is too low and will look like foot dragging")
    if low_speed_engine.last_foot_L[0] < 10.0:
        raise AssertionError("Low-speed gait swing target is too short before touchdown")
    if abs(decisive_pose[21] - STANDING[21]) < 180 or abs(decisive_pose[22] - STANDING[22]) < 320:
        raise AssertionError("Low-speed gait still produces tiny thigh/knee PWM changes")
    if decisive_pose[4] < STANDING[4] + 40:
        raise AssertionError("Left swing lift should include forward lean on the stance thigh")
    if decisive_pose[22] <= STANDING[22]:
        raise AssertionError("Left knee servo 22 direction is reversed during left swing lift")
    if decisive_pose[21] <= STANDING[21] or decisive_pose[23] >= STANDING[23]:
        raise AssertionError("Left swing ankle pitch servo 23 must counter the lifted thigh to keep the foot level")
    if decisive_pose[5] <= STANDING[5] or abs(decisive_pose[20] - STANDING[20]) > 15:
        raise AssertionError("Support hip roll should stay one-sided through left swing")
    if abs(decisive_pose[24] - STANDING[24]) > 15:
        raise AssertionError("Left swing foot roll should stay neutral during phase 2")

    right_speed_engine = DynamicWalkingEngine(dt=0.04, t_step=0.95, t_dbl=0.16, arm_swing_pwm=0)
    right_pose = None
    right_shift_pose = None
    for _ in range(90):
        right_pose = right_speed_engine.update(0.06)
        if (
            right_speed_engine.last_swing_leg == "right"
            and right_speed_engine.support_leg == "left"
            and right_speed_engine.last_lift_factor <= 0.02
        ):
            shift_score = (
                abs(right_pose[24] - STANDING[24])
                + abs(right_pose[20] - STANDING[20])
            )
            if right_shift_pose is None or shift_score > right_shift_pose[0]:
                right_shift_pose = (shift_score, dict(right_pose))
        if right_speed_engine.last_swing_leg == "right" and right_speed_engine.last_lift_factor > 0.95:
            break
    if right_shift_pose is None:
        raise AssertionError("Right-swing gait does not produce a left-support shift")
    left_support_hip = abs(right_shift_pose[1][20] - STANDING[20])
    left_support_ankle = abs(right_shift_pose[1][24] - STANDING[24])
    if left_support_hip < 40:
        raise AssertionError("Right-swing support hip counter-roll is too small")
    if left_support_ankle < 90:
        raise AssertionError("Right-swing support ankle roll is too small to unload the right foot")
    if left_support_ankle <= left_support_hip:
        raise AssertionError("Right-swing support shift should be ankle-led, not hip-led")
    if abs(right_shift_pose[1][5] - STANDING[5]) > 15:
        raise AssertionError("Right-swing hip roll should keep the swing hip near neutral")
    if abs(right_shift_pose[1][1] - STANDING[1]) > 15:
        raise AssertionError("Pre-lift right swing ankle roll should stay neutral")
    if right_shift_pose[1][4] < STANDING[4] + 40 or right_shift_pose[1][21] < STANDING[21] + 40:
        raise AssertionError("Right-swing support shift should preserve balanced forward lean on both thighs")
    if right_pose is None or right_speed_engine.last_lift_factor <= 0.95:
        raise AssertionError("Right-swing gait does not reach a clear swing-lift phase")
    if right_speed_engine.last_foot_R[2] < 27.0:
        raise AssertionError("Right swing target is too low and will look like foot dragging")
    if abs(right_pose[4] - STANDING[4]) < 130 or abs(right_pose[3] - STANDING[3]) < 280:
        raise AssertionError("Right-swing gait still produces tiny thigh/knee PWM changes")
    if right_pose[21] < STANDING[21] + 40:
        raise AssertionError("Right swing lift should include forward lean on the stance thigh")
    if right_pose[3] <= STANDING[3]:
        raise AssertionError("Right knee servo 3 direction is reversed during right swing lift")
    if right_pose[4] <= STANDING[4] or right_pose[2] >= STANDING[2]:
        raise AssertionError("Right swing ankle pitch servo 2 must counter the lifted thigh to keep the foot level")
    if abs(right_pose[5] - STANDING[5]) > 15 or right_pose[20] >= STANDING[20]:
        raise AssertionError("Support hip roll should stay one-sided through right swing")
    if abs(right_pose[1] - STANDING[1]) > 15:
        raise AssertionError("Right swing foot roll should stay neutral during phase 2")

    for _ in range(90):
        engine.update(0.16)
    if engine.is_idle_ready():
        raise AssertionError("Walking engine reports idle while commanded to walk")

    for _ in range(220):
        pose = engine.update(0.0)
    if not engine.is_idle_ready():
        raise AssertionError("Walking engine does not settle back to neutral queues")
    if pose != STANDING:
        raise AssertionError("Walking engine settled pose does not match exact STANDING")


def validate_landing_transfer_continuity() -> None:
    engine = DynamicWalkingEngine(
        dt=0.04,
        t_step=0.95,
        t_dbl=0.16,
        step_x_ratio=0.52,
        thigh_lift_forward_mm=18.0,
    )
    engine._enqueue_next_step(step_len=12.0, turn_len=0.0, side_len=0.0)

    first_step_idx = engine.n_d + engine.n_d
    last_single_L = engine.foot_L_queue[first_step_idx + engine.n_s - 1]
    first_transfer_L = engine.foot_L_queue[first_step_idx + engine.n_s]
    last_transfer_L = engine.foot_L_queue[first_step_idx + engine.n_s + engine.n_d - 1]
    last_single_zmp = engine.zmp_y_queue[first_step_idx + engine.n_s - 1]
    first_transfer_zmp = engine.zmp_y_queue[first_step_idx + engine.n_s]
    last_transfer_zmp = engine.zmp_y_queue[first_step_idx + engine.n_s + engine.n_d - 1]
    support_zmp = engine.zmp_y_queue[first_step_idx]
    engine._enqueue_next_step(step_len=12.0, turn_len=0.0, side_len=0.0)
    first_right_support_L = engine.foot_L_queue[first_step_idx + engine.n_s + engine.n_d]

    if abs(last_single_L[2]) > 1e-9:
        raise AssertionError("Swing foot should be landed before transfer starts")
    if abs(first_transfer_L[0] - last_single_L[0]) > 1e-6:
        raise AssertionError("Landing transfer snaps swing foot X instead of holding touchdown pose")
    if abs(last_transfer_L[0] - last_single_L[0]) > 1e-6:
        raise AssertionError("Landing transfer should hold touchdown X spacing for the next step preload")
    if abs(first_right_support_L[0] - last_single_L[0]) > 1e-6:
        raise AssertionError("New support foot snaps back to old X position after touchdown")
    if abs(last_single_zmp) > 1e-6 or abs(first_transfer_zmp) > 1e-6:
        raise AssertionError("Landing should release ankle/hip roll to zero before next-support preload")
    if last_transfer_zmp >= -1e-6:
        raise AssertionError("Landing transfer should blend into next-support preload during touchdown")
    if abs(last_transfer_zmp) >= abs(support_zmp):
        raise AssertionError("Landing transfer preload should be smaller than the original support shift")


def validate_single_leg_lift_pose() -> None:
    hw = ROBOT["half_hip"]
    left_lift_pose = compute_pose(
        0.0,
        hw * 0.58,
        np.array([35.0, -hw, 45.0]),
        np.array([0.0, hw, 0.0]),
        support_leg="right",
    )
    right_lift_pose = compute_pose(
        0.0,
        -hw * 0.58,
        np.array([0.0, -hw, 0.0]),
        np.array([35.0, hw, 45.0]),
        support_leg="left",
    )

    for name, pose, hip_sid, knee_sid in (
        ("left lift", left_lift_pose, 21, 22),
        ("right lift", right_lift_pose, 4, 3),
    ):
        for sid, pwm in pose.items():
            if not 500 <= pwm <= 2500:
                raise AssertionError(f"{name}: servo {sid} out of bounds: {pwm}")
        if abs(pose[hip_sid] - STANDING[hip_sid]) < 80:
            raise AssertionError(f"{name}: swing hip pitch barely changed")
        if abs(pose[knee_sid] - STANDING[knee_sid]) < 80:
            raise AssertionError(f"{name}: swing knee barely changed")


def validate_three_servo_leg_pose() -> None:
    for name, hip_sid, thigh_sid, knee_sid, hip_key, thigh_key, knee_key, hip_delta in (
        ("left three-servo lift", 20, 21, 22, "L_hip_abduct", "L_hip_pitch", "L_knee", -7.0),
        ("right three-servo lift", 5, 4, 3, "R_hip_abduct", "R_hip_pitch", "R_knee", 7.0),
    ):
        pose = dict(STANDING)
        pose[hip_sid] = angle_to_pwm(hip_sid, STAND_ANG[hip_key], STAND_ANG[hip_key] + hip_delta, STANDING[hip_sid])
        pose[thigh_sid] = angle_to_pwm(thigh_sid, STAND_ANG[thigh_key], STAND_ANG[thigh_key] + 28.0, STANDING[thigh_sid])
        pose[knee_sid] = angle_to_pwm(knee_sid, STAND_ANG[knee_key], STAND_ANG[knee_key] - 20.0, STANDING[knee_sid])

        for sid in (hip_sid, thigh_sid, knee_sid):
            if not 500 <= pose[sid] <= 2500:
                raise AssertionError(f"{name}: servo {sid} out of bounds: {pose[sid]}")
        if abs(pose[hip_sid] - STANDING[hip_sid]) < 40:
            raise AssertionError(f"{name}: hip roll servo barely changed")
        if abs(pose[thigh_sid] - STANDING[thigh_sid]) < 250:
            raise AssertionError(f"{name}: thigh servo barely changed")
        if abs(pose[knee_sid] - STANDING[knee_sid]) < 180:
            raise AssertionError(f"{name}: knee servo barely changed")


def main() -> None:
    validate_leg_kinematics()
    validate_zmp_preview()
    validate_getup_sequences()
    validate_walking_settle()
    validate_landing_transfer_continuity()
    validate_single_leg_lift_pose()
    validate_three_servo_leg_pose()
    print("algorithm validation passed: IK/FK endpoint, ZMP preview, get-up bounds, walking settle, landing transfer, single-leg lift, three-servo lift")


if __name__ == "__main__":
    main()
