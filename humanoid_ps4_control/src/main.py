from __future__ import annotations

import argparse
from dataclasses import dataclass
import sys
import threading
import time
from typing import Dict, Optional

from .rtrobot_xml import ActionGroup, load_xml
from .backends import make_backend
from .motion import playback


def _play_group(
    group: ActionGroup,
    backend,
    loop: bool = False,
    update_ms: int = 20,
    stop_flag: Optional[threading.Event] = None,
    start_pose: Optional[Dict[int, int]] = None,
    pose_log: Optional[Dict[int, int]] = None,
) -> None:
    print(
        f"[main] Playing Group {group.group_id} alias={group.alias!r} "
        f"frames={len(group.frames)} loop={loop}"
    )

    for pose, _ in playback(group, loop=loop, update_ms=update_ms, start_pose=start_pose):
        if stop_flag and stop_flag.is_set():
            print("[main] Stop requested - halting playback.")
            break
        if pose_log is not None:
            pose_log.update(pose)
        backend.send(pose, duration_ms=update_ms)


def run_direct(args: argparse.Namespace) -> None:
    groups = load_xml(args.xml)
    if args.group not in groups:
        print(f"[ERROR] Group {args.group} not found. Available: {sorted(groups.keys())}")
        sys.exit(1)

    with make_backend(mode=args.backend, port=args.port, baudrate=args.baudrate, csv_path=args.csv) as backend:
        _play_group(groups[args.group], backend, loop=args.loop, update_ms=args.update_ms)
    print("[main] Done.")


def run_getup(args: argparse.Namespace) -> None:
    """Run one get-up sequence directly, without PS4/keyboard input."""
    from .getup import GetupEngine
    from .walking_engine import STANDING

    dt = args.update_ms / 1000.0
    engine = GetupEngine(dt=dt, mode=args.getup_mode, speed=args.getup_speed)
    debug_ids = [1, 2, 3, 4, 5, 6, 7, 8, 16, 17, 18, 19, 20, 21, 22, 23, 24]

    print(
        f"[main] Direct get-up: mode={args.getup_mode}, backend={args.backend}, "
        f"port={args.port}, update_ms={args.update_ms}, speed={args.getup_speed}"
    )

    with make_backend(mode=args.backend, port=args.port, baudrate=args.baudrate, csv_path=args.csv) as backend:
        if args.pre_stand_ms > 0:
            print(f"[main] Sending pre-stand for {args.pre_stand_ms} ms.")
            backend.send(STANDING, duration_ms=args.pre_stand_ms, force=True)
            time.sleep(args.pre_stand_ms / 1000.0)

        label = engine.start(current_pose=STANDING, mode=args.getup_mode)
        print(f"[main] Starting get-up from step {label}.")

        frame = 0
        last_label = None
        try:
            while engine.running:
                pose = engine.update()
                backend.send(pose, duration_ms=args.update_ms, force=True)

                if engine.label != last_label:
                    contacts = ",".join(engine.contacts) if engine.contacts else "-"
                    print(f"[getup] step={engine.label} contacts={contacts}")
                    last_label = engine.label

                if args.getup_print_every > 0 and frame % args.getup_print_every == 0:
                    values = " ".join(f"{sid}:{pose[sid]}" for sid in debug_ids)
                    print(f"[getup] frame={frame:04d} {values}")

                frame += 1
                time.sleep(dt)
        except KeyboardInterrupt:
            print("\n[main] Ctrl+C received. Returning to STANDING.")
        finally:
            print(f"[main] Sending final standing for {args.final_stand_ms} ms.")
            backend.send(STANDING, duration_ms=args.final_stand_ms, force=True)
            time.sleep(args.final_stand_ms / 1000.0)

    print("[main] Direct get-up done.")


def run_leg_lift_test(args: argparse.Namespace) -> None:
    """Run a direct single-leg vertical lift test using the leg IK solver."""
    import numpy as np

    from .walking_engine import ROBOT, STANDING, compute_pose

    def smooth01(value: float) -> float:
        value = max(0.0, min(1.0, value))
        return value * value * (3.0 - 2.0 * value)

    hw = ROBOT["half_hip"]
    height_mm = max(0.0, min(80.0, float(args.lift_height_mm)))
    forward_mm = max(0.0, min(70.0, float(args.lift_forward_mm)))
    support_leg = "right" if args.lift_leg == "left" else "left"
    support_y = hw * args.lift_zmp_ratio if support_leg == "right" else -hw * args.lift_zmp_ratio
    left_neutral = np.array([0.0, -hw, 0.0])
    right_neutral = np.array([0.0, hw, 0.0])
    debug_ids = [1, 2, 3, 4, 5, 20, 21, 22, 23, 24]

    ramp_frames = max(1, round(args.lift_ramp_ms / args.update_ms))
    hold_frames = max(1, round(args.lift_hold_ms / args.update_ms))
    dt = args.update_ms / 1000.0
    pre_stand_ms = args.pre_stand_ms if args.pre_stand_ms > 0 else 1200

    def make_pose(com_y: float, lift_z: float) -> dict[int, int]:
        foot_l = left_neutral.copy()
        foot_r = right_neutral.copy()
        lift_ratio = 0.0 if height_mm <= 1e-6 else max(0.0, min(1.0, lift_z / height_mm))
        lift_x = forward_mm * lift_ratio
        if args.lift_leg == "left":
            foot_l[0] = lift_x
            foot_l[2] = lift_z
        else:
            foot_r[0] = lift_x
            foot_r[2] = lift_z
        if abs(com_y) < 1e-6 and lift_z <= 1e-6:
            return dict(STANDING)
        return compute_pose(0.0, com_y, foot_l, foot_r, support_leg=support_leg)

    def send_phase(backend, phase: str, frames: int, com_start: float, com_end: float, z_start: float, z_end: float) -> None:
        for frame in range(frames):
            alpha = smooth01((frame + 1) / frames)
            com_y = com_start + (com_end - com_start) * alpha
            lift_z = z_start + (z_end - z_start) * alpha
            pose = make_pose(com_y, lift_z)
            if args.lift_print_every > 0 and frame % args.lift_print_every == 0:
                values = " ".join(f"{sid}:{pose[sid]}" for sid in debug_ids)
                print(
                    f"[leg-lift] phase={phase} leg={args.lift_leg} support={support_leg} "
                    f"com_y={com_y:.1f}mm lift_x={forward_mm * (0.0 if height_mm <= 1e-6 else lift_z / height_mm):.1f}mm "
                    f"lift_z={lift_z:.1f}mm {values}"
                )
            backend.send(pose, duration_ms=args.update_ms, force=True)
            time.sleep(dt)

    print(
        f"[main] Direct leg lift: leg={args.lift_leg}, support={support_leg}, "
        f"height={height_mm:.1f}mm, forward={forward_mm:.1f}mm, "
        f"backend={args.backend}, port={args.port}, update_ms={args.update_ms}"
    )

    with make_backend(mode=args.backend, port=args.port, baudrate=args.baudrate, csv_path=args.csv) as backend:
        print(f"[main] Sending STANDING for {pre_stand_ms} ms.")
        backend.send(STANDING, duration_ms=pre_stand_ms, force=True)
        time.sleep(pre_stand_ms / 1000.0)

        try:
            send_phase(backend, "shift-support", ramp_frames, 0.0, support_y, 0.0, 0.0)
            send_phase(backend, "lift", ramp_frames, support_y, support_y, 0.0, height_mm)
            send_phase(backend, "hold", hold_frames, support_y, support_y, height_mm, height_mm)
            send_phase(backend, "lower", ramp_frames, support_y, support_y, height_mm, 0.0)
            send_phase(backend, "return-center", ramp_frames, support_y, 0.0, 0.0, 0.0)
        except KeyboardInterrupt:
            print("\n[main] Ctrl+C received. Returning to STANDING.")
        finally:
            print(f"[main] Sending final STANDING for {args.final_stand_ms} ms.")
            backend.send(STANDING, duration_ms=args.final_stand_ms, force=True)
            time.sleep(args.final_stand_ms / 1000.0)

    print("[main] Direct leg lift done.")


def run_knee_lift_test(args: argparse.Namespace) -> None:
    """Run a decisive three-servo leg test: hip roll, thigh pitch, knee."""
    from .walking_engine import STAND_ANG, STANDING, angle_to_pwm

    def clamp_pwm(value: float) -> int:
        return max(500, min(2500, round(value)))

    leg = args.lift_leg
    hip_roll_mag = max(0.0, min(25.0, float(args.three_hip_roll_deg)))
    thigh_delta = max(-60.0, min(70.0, float(args.three_thigh_deg)))
    knee_delta = max(-70.0, min(70.0, float(args.three_knee_delta_deg)))
    move_ms = max(40, int(args.three_move_ms))
    hold_ms = max(0, int(args.three_hold_ms))
    pre_stand_ms = args.pre_stand_ms if args.pre_stand_ms > 0 else 700
    arm_amp = max(0, int(args.arm_swing_pwm))

    if leg == "left":
        hip_sid, thigh_sid, knee_sid = 20, 21, 22
        hip_key, thigh_key, knee_key = "L_hip_abduct", "L_hip_pitch", "L_knee"
        hip_roll_delta = -hip_roll_mag
        right_arm_delta = arm_amp
        left_arm_delta = -arm_amp
    else:
        hip_sid, thigh_sid, knee_sid = 5, 4, 3
        hip_key, thigh_key, knee_key = "R_hip_abduct", "R_hip_pitch", "R_knee"
        hip_roll_delta = hip_roll_mag
        right_arm_delta = -arm_amp
        left_arm_delta = arm_amp

    pose = dict(STANDING)
    pose[hip_sid] = angle_to_pwm(hip_sid, STAND_ANG[hip_key], STAND_ANG[hip_key] + hip_roll_delta, STANDING[hip_sid])
    pose[thigh_sid] = angle_to_pwm(thigh_sid, STAND_ANG[thigh_key], STAND_ANG[thigh_key] + thigh_delta, STANDING[thigh_sid])
    pose[knee_sid] = angle_to_pwm(knee_sid, STAND_ANG[knee_key], STAND_ANG[knee_key] + knee_delta, STANDING[knee_sid])
    if arm_amp > 0:
        pose[8] = clamp_pwm(STANDING[8] + args.arm_right_dir * right_arm_delta)
        pose[17] = clamp_pwm(STANDING[17] + args.arm_left_dir * left_arm_delta)
        pose[6] = clamp_pwm(STANDING[6] + abs(right_arm_delta) * args.arm_elbow_ratio)
        pose[19] = clamp_pwm(STANDING[19] - abs(left_arm_delta) * args.arm_elbow_ratio)
        pose[7] = clamp_pwm(STANDING[7] + abs(right_arm_delta) * args.arm_lift_ratio)
        pose[18] = clamp_pwm(STANDING[18] - abs(left_arm_delta) * args.arm_lift_ratio)

    print(
        f"[main] Three-servo leg test: leg={leg}, backend={args.backend}, port={args.port}, "
        f"move_ms={move_ms}, hold_ms={hold_ms}, arm_swing_pwm={arm_amp}"
    )
    print(
        f"[three-servo] hip {hip_sid}: {STANDING[hip_sid]} -> {pose[hip_sid]} "
        f"({hip_roll_delta:+.1f}deg), thigh {thigh_sid}: {STANDING[thigh_sid]} -> {pose[thigh_sid]} "
        f"({thigh_delta:+.1f}deg), knee {knee_sid}: {STANDING[knee_sid]} -> {pose[knee_sid]} "
        f"({knee_delta:+.1f}deg)"
    )
    if arm_amp > 0:
        print(
            f"[three-servo] arms 8:{STANDING[8]}->{pose[8]} 17:{STANDING[17]}->{pose[17]} "
            f"6:{STANDING[6]}->{pose[6]} 19:{STANDING[19]}->{pose[19]} "
            f"7:{STANDING[7]}->{pose[7]} 18:{STANDING[18]}->{pose[18]}"
        )

    with make_backend(mode=args.backend, port=args.port, baudrate=args.baudrate, csv_path=args.csv) as backend:
        print(f"[main] Sending full STANDING pose for {pre_stand_ms} ms.")
        backend.send(STANDING, duration_ms=pre_stand_ms, force=True)
        time.sleep(pre_stand_ms / 1000.0)

        try:
            for rep in range(max(1, int(args.three_repeat))):
                print(f"[three-servo] rep={rep + 1} lift")
                backend.send(pose, duration_ms=move_ms, force=True)
                time.sleep(move_ms / 1000.0 + hold_ms / 1000.0)

                print(f"[three-servo] rep={rep + 1} standing")
                backend.send(STANDING, duration_ms=move_ms, force=True)
                time.sleep(move_ms / 1000.0)
        except KeyboardInterrupt:
            print("\n[main] Ctrl+C received. Returning to STANDING.")
        finally:
            print(f"[main] Sending final full STANDING pose for {args.final_stand_ms} ms.")
            backend.send(STANDING, duration_ms=args.final_stand_ms, force=True)
            time.sleep(args.final_stand_ms / 1000.0)

    print("[main] Three-servo leg test done.")


def run_ps4(args: argparse.Namespace) -> None:
    """
    Real-time walking mode.

    D-pad/keyboard:
      Up/U       : walk forward
      Down/D     : walk backward
      Left/A     : side walk left
      Right      : side walk right
      Stick X    : turn left/right in auto/stick mode
      J/K        : side walk left/right
      L1/L       : toggle standing arm dance
      R1/G       : run get-up sequence
      B          : run back get-up sequence
      Circle/C   : stop and hold standing
      Triangle/E : reset walking engine
      Q          : quit
    """
    from .ps4_pygame import PS4Reader
    from .walking_engine import DynamicWalkingEngine, STANDING
    from .arm_dance import ArmDanceEngine
    from .getup import GetupEngine
    from .balance import BalanceConfig, IMUBalanceController
    from .imu_bno055 import BNO055Reader

    backend = make_backend(mode=args.backend, port=args.port, baudrate=args.baudrate, csv_path=args.csv)

    poll_hz = int(1000 / args.update_ms)
    reader = PS4Reader(
        joystick_index=args.joystick_index,
        fallback_keys=True,
        debug=args.input_debug,
        poll_rate_hz=poll_hz,
        deadzone=args.input_deadzone,
    )
    reader.init()

    engine = DynamicWalkingEngine(
        dt=args.update_ms / 1000.0,
        t_step=args.t_step,
        t_dbl=args.t_dbl,
        max_step_len=args.max_step_len,
        max_turn_step_len=args.max_turn_step_len,
        max_side_step_len=args.max_side_step_len,
        step_height=args.step_height,
        zmp_support_ratio=args.zmp_support_ratio,
        hip_abduct_gain=args.hip_abduct_gain,
        swing_hip_roll_scale=args.swing_hip_roll_scale,
        ankle_roll_gain=args.ankle_roll_gain,
        swing_ankle_roll_scale=args.swing_ankle_roll_scale,
        step_x_ratio=args.step_x_ratio,
        thigh_lift_forward_mm=args.thigh_lift_forward_mm,
        left_swing_x_scale=args.left_swing_x_scale,
        left_step_height_scale=args.left_step_height_scale,
        landing_gap_mm=args.landing_gap_mm,
        right_swing_x_scale=args.right_swing_x_scale,
        right_step_height_scale=args.right_step_height_scale,
        lift_start_phase=args.lift_start_phase,
        swing_advance_end_phase=args.swing_advance_end_phase,
        lift_end_phase=args.lift_end_phase,
        landing_roll_release_start=args.landing_roll_release_start,
        command_rate_limit=args.command_rate_limit,
        arm_swing_pwm=args.arm_swing_pwm,
        arm_right_dir=args.arm_right_dir,
        arm_left_dir=args.arm_left_dir,
        arm_elbow_ratio=args.arm_elbow_ratio,
        arm_lift_ratio=args.arm_lift_ratio,
        arm_smooth_tau=args.arm_smooth_tau,
        arm_min_pwm=args.arm_min_pwm,
        arm_quantum_pwm=args.arm_quantum_pwm,
    )
    arm_dance = ArmDanceEngine(
        dt=args.update_ms / 1000.0,
        period_s=args.dance_period,
        transition_s=args.dance_transition,
        shoulder_pwm=args.dance_shoulder_pwm,
        elbow_pwm=args.dance_elbow_pwm,
        lift_pwm=args.dance_lift_pwm,
        head_pwm=args.dance_head_pwm,
        head_speed=args.dance_head_speed,
        smooth_tau=args.dance_smooth_tau,
        max_pwm_per_sec=args.dance_max_pwm_per_sec,
        min_step_pwm=args.dance_min_step_pwm,
    )
    getup = GetupEngine(
        dt=args.update_ms / 1000.0,
        mode=args.getup_mode,
        speed=args.getup_speed,
    )
    prev_l1_pressed = False
    prev_stop_pressed = False
    prev_getup_pressed = False
    prev_getup_back_pressed = False
    last_pose = dict(STANDING)
    standing_hold_active = True

    balance = None
    imu = None
    last_balance_t = time.monotonic()
    if args.imu_balance:
        balance = IMUBalanceController(
            BalanceConfig(
                max_correction_deg=args.balance_limit_deg,
                roll_deadband_deg=args.balance_deadband_deg,
                pitch_deadband_deg=args.balance_deadband_deg,
            )
        )
        imu = BNO055Reader(
            roll_sign=args.imu_roll_sign,
            pitch_sign=args.imu_pitch_sign,
            yaw_sign=args.imu_yaw_sign,
        )
        imu.open()
        print("[main] IMU balance enabled: BNO055 roll/pitch feedback active.")

    print(
        "\n[PS4 Mode - Real-time ZMP] Up/Down walk, Left/Right side, stick-X turn, J/K side, "
        "L1/L/M dance, R1/G get-up, B get-up back, C stop, Q quit\n"
    )

    try:
        with backend:
            try:
                for state in reader.poll():
                    if state.quit:
                        print("[main] Quit requested.")
                        break
    
                    axis_forward_cmd = state.signed_axis(args.ps4_forward_axis, args.ps4_forward_sign)
                    axis_turn_cmd = state.signed_axis(args.ps4_turn_axis, args.ps4_turn_sign)
                    dpad_forward_cmd = 1.0 if state.dpad_up() else (-1.0 if state.dpad_down() else 0.0)
                    dpad_side_cmd = 1.0 if state.dpad_left() else (-1.0 if state.dpad_right() else 0.0)
                    button_side_cmd = 1.0 if state.button(reader.BTN_L2) else (-1.0 if state.button(reader.BTN_R2) else 0.0)
    
                    if args.input_mode == "stick":
                        input_cmd = axis_forward_cmd
                        turn_input_cmd = axis_turn_cmd
                        side_input_cmd = button_side_cmd
                    elif args.input_mode == "dpad":
                        input_cmd = dpad_forward_cmd
                        turn_input_cmd = 0.0
                        side_input_cmd = dpad_side_cmd or button_side_cmd
                    else:
                        input_cmd = axis_forward_cmd if abs(axis_forward_cmd) > args.input_deadzone else dpad_forward_cmd
                        turn_input_cmd = axis_turn_cmd if abs(axis_turn_cmd) > args.input_deadzone else 0.0
                        side_input_cmd = dpad_side_cmd or button_side_cmd
    
                    vy = input_cmd * args.walk_speed
                    turn_cmd = turn_input_cmd * args.turn_speed
                    side_cmd = side_input_cmd * args.side_speed
                    motion_requested = vy != 0.0 or turn_cmd != 0.0 or side_cmd != 0.0
    
                    if args.input_debug and (
                        abs(input_cmd) > args.input_deadzone
                        or abs(turn_input_cmd) > args.input_deadzone
                        or abs(side_input_cmd) > args.input_deadzone
                        or state.dpad_up()
                        or state.dpad_down()
                        or state.dpad_left()
                        or state.dpad_right()
                        or state.button(reader.BTN_L2)
                        or state.button(reader.BTN_R2)
                        or state.button(reader.BTN_L1)
                        or state.button(reader.BTN_R1)
                        or state.button(reader.BTN_GETUP_BACK)
                        or state.button(reader.BTN_CIRCLE)
                        or state.button(reader.BTN_TRIANGLE)
                        or getup.running
                    ):
                        print(
                            f"[input] up={state.dpad_up()} down={state.dpad_down()} "
                            f"left={state.dpad_left()} right={state.dpad_right()} "
                            f"axis{args.ps4_forward_axis}={state.axis(args.ps4_forward_axis):.3f} "
                            f"turn_axis{args.ps4_turn_axis}={state.axis(args.ps4_turn_axis):.3f} "
                            f"cmd={input_cmd:.3f} turn={turn_input_cmd:.3f} side={side_input_cmd:.3f} "
                            f"l2={state.button(reader.BTN_L2)} r2={state.button(reader.BTN_R2)} "
                            f"l1={state.button(reader.BTN_L1)} r1={state.button(reader.BTN_R1)} "
                            f"b_back={state.button(reader.BTN_GETUP_BACK)} "
                            f"circle={state.button(reader.BTN_CIRCLE)} triangle={state.button(reader.BTN_TRIANGLE)} "
                            f"dance={arm_dance.mode} getup={getup.label} "
                            f"vy={vy:.3f} turn_cmd={turn_cmd:.3f} side_cmd={side_cmd:.3f}"
                        )
    
                    stop_pressed = state.button(reader.BTN_CIRCLE)
                    if stop_pressed:
                        if not prev_stop_pressed:
                            print("[main] C/Circle pressed. Hard stop to STANDING.")
                        prev_stop_pressed = True
                        engine.reset()
                        arm_dance.reset()
                        getup.reset()
                        standing_hold_active = True
                        pose = dict(STANDING)
                        if args.pose_debug:
                            print(
                                "[pose] STANDING "
                                + " ".join(f"{sid}:{pose[sid]}" for sid in sorted(pose))
                            )
                        try:
                            backend.send(pose, duration_ms=args.stop_ms, force=True)
                            last_pose = dict(pose)
                        except Exception as exc:
                            print(f"[main] Backend send exception: {exc}")
                        continue
                    prev_stop_pressed = False
    
                    getup_pressed = state.button(reader.BTN_R1)
                    if getup_pressed and not prev_getup_pressed:
                        engine.reset()
                        arm_dance.reset()
                        standing_hold_active = False
                        label = getup.start(last_pose, mode=args.getup_mode)
                        print(f"[main] R1/G pressed. Running {args.getup_mode} get-up sequence from step {label}.")
                    prev_getup_pressed = getup_pressed
    
                    getup_back_pressed = state.button(reader.BTN_GETUP_BACK)
                    if getup_back_pressed and not prev_getup_back_pressed:
                        engine.reset()
                        arm_dance.reset()
                        standing_hold_active = False
                        label = getup.start(last_pose, mode="back")
                        print(f"[main] B pressed. Running back get-up sequence from step {label}.")
                    prev_getup_back_pressed = getup_back_pressed
    
                    l1_pressed = state.button(reader.BTN_L1)
                    if l1_pressed and not prev_l1_pressed and not getup.running:
                        enabled = arm_dance.toggle()
                        engine.reset()
                        standing_hold_active = not enabled
                        print("[main] L1 arm dance ON." if enabled else "[main] L1 arm dance OFF - returning to STANDING.")
                    prev_l1_pressed = l1_pressed
    
                    if state.button(reader.BTN_TRIANGLE):
                        print("[main] Triangle/E pressed. Resetting walking engine and arm dance.")
                        engine.reset()
                        arm_dance.reset()
                        getup.reset()
                        standing_hold_active = True
                        vy = 0.0
                        turn_cmd = 0.0
                        side_cmd = 0.0
                        motion_requested = False
    
                    pose_from_getup = False
                    if getup.running:
                        vy = 0.0
                        turn_cmd = 0.0
                        side_cmd = 0.0
                        motion_requested = False
                        pose = getup.update()
                        pose_from_getup = True
                        if not getup.running:
                            engine.reset()
                            standing_hold_active = True
                            print("[main] Get-up finished. Holding exact STANDING until movement input.")
                    elif arm_dance.running:
                        vy = 0.0
                        turn_cmd = 0.0
                        side_cmd = 0.0
                        motion_requested = False
                        pose = arm_dance.update()
                    elif standing_hold_active and not motion_requested:
                        pose = dict(STANDING)
                    else:
                        if motion_requested and standing_hold_active:
                            engine.reset()
                            standing_hold_active = False
                        pose = engine.update(vy, turn_cmd=turn_cmd, side_cmd=side_cmd)
                        if not motion_requested and engine.is_idle_ready():
                            engine.reset()
                            standing_hold_active = True
                            pose = dict(STANDING)
                            if args.pose_debug:
                                print("[pose] walking settled. Holding exact STANDING.")
    
                    if args.pose_debug and (
                        vy != 0.0
                        or turn_cmd != 0.0
                        or side_cmd != 0.0
                        or arm_dance.running
                        or pose_from_getup
                        or state.button(reader.BTN_TRIANGLE)
                    ):
                        leg_ids = [1, 2, 3, 4, 5, 20, 21, 22, 23, 24]
                        leg_pose = " ".join(f"{sid}:{pose[sid]}" for sid in leg_ids)
                        if arm_dance.running:
                            print(
                                f"[dance] mode={arm_dance.mode} "
                                f"arm6:{pose[6]} arm7:{pose[7]} arm8:{pose[8]} "
                                f"head16:{pose[16]} arm17:{pose[17]} arm18:{pose[18]} arm19:{pose[19]}"
                            )
                        elif pose_from_getup:
                            contacts = ",".join(getup.contacts) if getup.contacts else "-"
                            print(
                                f"[getup] mode={args.getup_mode} step={getup.label} contacts={contacts} "
                                f"{leg_pose} arm6:{pose[6]} arm7:{pose[7]} arm8:{pose[8]} "
                                f"head16:{pose[16]} arm17:{pose[17]} arm18:{pose[18]} arm19:{pose[19]}"
                            )
                        else:
                            arm_pose = (
                                f" arm6:{pose[6]} arm7:{pose[7]} arm8:{pose[8]}"
                                f" arm17:{pose[17]} arm18:{pose[18]} arm19:{pose[19]}"
                                if args.arm_swing_pwm > 0 else ""
                            )
                            print(
                                f"[pose] support={engine.support_leg} "
                                f"swing={engine.last_swing_leg} liftF={engine.last_lift_factor:.2f} "
                                f"cmd_step={engine.commanded_step_len:.1f}mm cmd_turn={engine.commanded_turn_len:.1f}mm "
                                f"cmd_side={engine.commanded_side_len:.1f}mm "
                                f"liftL={engine.last_foot_L[2]:.1f}mm liftR={engine.last_foot_R[2]:.1f}mm "
                                f"{leg_pose}{arm_pose}"
                            )
    
                    if balance is not None and imu is not None and not pose_from_getup:
                        now = time.monotonic()
                        balance_dt = now - last_balance_t
                        last_balance_t = now
                        reading = imu.read()
                        if reading is not None:
                            pose = balance.apply(
                                pose,
                                roll_deg=reading.roll_deg,
                                pitch_deg=reading.pitch_deg,
                                dt=balance_dt,
                                support_leg=engine.support_leg,
                            )
    
                    try:
                        backend.send(pose, duration_ms=args.update_ms)
                        last_pose = dict(pose)
                    except Exception as exc:
                        print(f"[main] Backend send exception: {exc}")
            except KeyboardInterrupt:
                print("\n[main] Ctrl+C received. Returning to STANDING.")
            finally:
                try:
                    backend.send(STANDING, duration_ms=args.stop_ms, force=True)
                    time.sleep(args.stop_ms / 1000.0)
                except Exception as exc:
                    print(f"[main] Backend send exception while returning to STANDING: {exc}")
    finally:
        reader.quit()
        print("[main] PS4 mode exited.")


@dataclass
class Config:
    # --- Run Mode ---
    ps4: bool = True
    getup: bool = False
    leg_lift_test: bool = False
    knee_lift_test: bool = False

    # --- Hardware ---
    xml: str = "actions/standing.xml"
    backend: str = "serial"
    port: str = "COM24"
    baudrate: int = 115200
    csv: str = "out/log.csv"
    group: int = 0
    update_ms: int = 40
    stop_ms: int = 250

    # --- Walking Engine ---
    walk_speed: float = 0.30
    turn_speed: float = 0.35
    side_speed: float = 0.20
    max_step_len: float = 28.0
    max_turn_step_len: float = 7.0
    max_side_step_len: float = 8.0
    step_height: float = 24.0
    t_step: float = 1.28
    t_dbl: float = 0.04
    zmp_support_ratio: float = 0.88
    hip_abduct_gain: float = 0.35
    ankle_roll_gain: float = -0.38
    step_x_ratio: float = 0.62
    thigh_lift_forward_mm: float = 3.0
    left_swing_x_scale: float = 1.45
    left_step_height_scale: float = 1.25
    right_swing_x_scale: float = 1.45
    right_step_height_scale: float = 1.25
    landing_gap_mm: float = 18.0
    lift_start_phase: float = 0.0
    swing_advance_end_phase: float = 0.60
    lift_end_phase: float = 0.96
    landing_roll_release_start: float = 0.62
    command_rate_limit: float = 24.0
    swing_hip_roll_scale: float = 0.0
    swing_ankle_roll_scale: float = 0.0

    # --- Arms ---
    arm_swing_pwm: int = 260
    arm_right_dir: int = 1
    arm_left_dir: int = 1
    arm_elbow_ratio: float = 0.0
    arm_lift_ratio: float = 0.0
    arm_smooth_tau: float = 0.22
    arm_min_pwm: int = 30
    arm_quantum_pwm: int = 5

    # --- Control Mode ---
    input_mode: str = "auto"
    joystick_index: int = 0
    input_deadzone: float = 0.08
    ps4_forward_axis: int = 1
    ps4_forward_sign: float = -1.0
    ps4_turn_axis: int = 0
    ps4_turn_sign: float = -1.0

    # --- Dance ---
    dance_period: float = 2.4
    dance_transition: float = 0.45
    dance_shoulder_pwm: int = 420
    dance_elbow_pwm: int = 260
    dance_lift_pwm: int = 820
    dance_head_pwm: int = 180
    dance_head_speed: float = 1.0
    dance_smooth_tau: float = 0.08
    dance_max_pwm_per_sec: float = 2200.0
    dance_min_step_pwm: int = 18

    # --- Getup ---
    getup_mode: str = "back"
    getup_speed: float = 0.7
    pre_stand_ms: int = 0
    final_stand_ms: int = 1200
    getup_print_every: int = 20

    # --- Testing ---
    lift_leg: str = "left"
    lift_height_mm: float = 45.0
    lift_forward_mm: float = 35.0
    lift_zmp_ratio: float = 0.58
    lift_ramp_ms: int = 1200
    lift_hold_ms: int = 1500
    lift_print_every: int = 8
    three_hip_roll_deg: float = 7.0
    three_thigh_deg: float = 28.0
    three_knee_delta_deg: float = -20.0
    three_move_ms: int = 150
    three_hold_ms: int = 650
    three_repeat: int = 1
    
    # --- Debug ---
    loop: bool = False
    input_debug: bool = False
    pose_debug: bool = False
    imu_balance: bool = False
    imu_roll_sign: float = 1.0
    imu_pitch_sign: float = 1.0
    imu_yaw_sign: float = 1.0
    balance_limit_deg: float = 6.0
    balance_deadband_deg: float = 0.4
    
    # Deprecated/unused arguments to prevent crash

def main() -> None:
    args = Config()
    if args.knee_lift_test:
        run_knee_lift_test(args)
    elif args.leg_lift_test:
        run_leg_lift_test(args)
    elif args.getup:
        run_getup(args)
    elif args.ps4:
        run_ps4(args)
    else:
        run_direct(args)

if __name__ == "__main__":
    main()
