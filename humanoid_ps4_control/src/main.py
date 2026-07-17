from __future__ import annotations

import time

from .backends import make_backend
from .config import Config


def run_getup(args: Config) -> None:
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


def run_ps4(args: Config) -> None:
    """
    Real-time walking mode.

    D-pad/keyboard:
      Up/W       : walk forward
      Down/S     : walk backward
      Left/A     : side walk left
      Right/D    : side walk right
      Stick X    : turn left/right in auto/stick mode
      J/K        : side walk left/right
      L1/L       : toggle standing arm dance
      Cross/X    : toggle single-leg support test
      R1/G       : run get-up sequence
      B          : run back get-up sequence
      Circle/C   : stop and hold standing
      Triangle/E : reset walking engine
      Q          : quit
    """
    from .ps4_pygame import PS4Reader
    from .walking_engine import DynamicWalkingEngine, SingleSupportTestEngine, STANDING
    from .arm_dance import ArmDanceEngine
    from .getup import GetupEngine
    from .balance import BalanceConfig, IMUBalanceController
    from .sensors import RobotSensorHub

    backend = make_backend(mode=args.backend, port=args.port, baudrate=args.baudrate, csv_path=args.csv)

    poll_hz = int(1000 / args.update_ms)
    reader = PS4Reader(
        joystick_index=args.joystick_index,
        fallback_keys=True,
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
        ankle_roll_gain=args.ankle_roll_gain,
        step_x_ratio=args.step_x_ratio,
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
        trajectory_smoothing=args.trajectory_smoothing,
        arm_swing_pwm=args.arm_swing_pwm,
        arm_right_dir=args.arm_right_dir,
        arm_left_dir=args.arm_left_dir,
        arm_elbow_ratio=args.arm_elbow_ratio,
        arm_lift_ratio=args.arm_lift_ratio,
        arm_smooth_tau=args.arm_smooth_tau,
        arm_min_pwm=args.arm_min_pwm,
        arm_quantum_pwm=args.arm_quantum_pwm,
    )
    single_support = SingleSupportTestEngine(
        dt=args.update_ms / 1000.0,
        lift_height=args.single_support_lift_height,
        zmp_support_ratio=args.zmp_support_ratio,
        hip_abduct_gain=args.hip_abduct_gain,
        ankle_roll_gain=args.ankle_roll_gain,
        arm_pwm=args.single_support_arm_pwm,
        ramp_s=args.single_support_ramp_s,
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
    prev_single_support_pressed = False
    prev_options_pressed = False
    next_single_support_leg = "right"
    last_pose = dict(STANDING)
    standing_hold_active = True

    balance = None
    sensor_hub = None
    sensor_snapshot = None
    last_balance_t = time.monotonic()
    balance_has_valid_imu = False
    if args.sensor_feedback:
        sensor_hub = RobotSensorHub(
            port=args.sensor_port,
            baudrate=args.sensor_baudrate,
            timeout_s=args.sensor_timeout_s,
            use_imu=args.sensor_use_imu,
            use_fsr=args.sensor_use_fsr,
            imu_roll_sign=args.imu_roll_sign,
            imu_pitch_sign=args.imu_pitch_sign,
            imu_yaw_sign=args.imu_yaw_sign,
            imu_vertical_mount=args.imu_vertical_mount,
            imu_board_face_sign=args.imu_board_face_sign,
            fsr_invert=args.fsr_invert,
            fsr_filter_alpha=args.fsr_filter_alpha,
            fsr_left_zero_raw=args.fsr_left_zero_raw,
            fsr_left_full_raw=args.fsr_left_full_raw,
            fsr_right_zero_raw=args.fsr_right_zero_raw,
            fsr_right_full_raw=args.fsr_right_full_raw,
        )
        sensor_hub.open()
        print(f"[main] Sensor feedback enabled: ESP32 serial port={args.sensor_port}.")

    if args.imu_balance and (sensor_hub is None or not args.sensor_use_imu):
        print("[main] IMU balance requested but IMU sensor feedback is disabled.")

    print(
        "\n[PS4 Mode - Real-time ZMP] W/S walk, A/D side, arrows also work, stick-X turn, J/K side, "
        "X single support, L1/L/M dance, R1/G get-up, B get-up back, C stop, Q quit\n"
    )

    try:
        with backend:
            if args.imu_balance and sensor_hub is not None and args.sensor_use_imu:
                backend.send(STANDING, duration_ms=1000, force=True)
                time.sleep(1.0)
                print("[main] Keep the robot upright and still while IMU reference is captured.")
                imu_reference = sensor_hub.capture_imu_reference(
                    sample_seconds=args.imu_reference_seconds,
                    timeout_s=args.imu_reference_timeout_s,
                    min_gyro_cal=args.imu_min_gyro_cal,
                    min_accel_cal=args.imu_min_accel_cal,
                    max_rms_deg=args.imu_reference_max_rms_deg,
                )
                if imu_reference is None:
                    print("[main] IMU reference failed or robot moved. Balance remains disabled.")
                else:
                    target_roll, target_pitch = imu_reference
                    balance = IMUBalanceController(
                        BalanceConfig(
                            target_roll_deg=target_roll,
                            target_pitch_deg=target_pitch,
                            max_correction_deg=args.balance_limit_deg,
                            roll_deadband_deg=args.balance_deadband_deg,
                            pitch_deadband_deg=args.balance_deadband_deg,
                        )
                    )
                    print(
                        f"[main] IMU balance enabled: reference roll={target_roll:.2f}, "
                        f"pitch={target_pitch:.2f}, limit={args.balance_limit_deg:.1f} deg."
                    )
            try:
                for state in reader.poll():
                    if state.quit:
                        print("[main] Returning to function menu.")
                        break

                    options_pressed = state.button(reader.BTN_OPTIONS)
                    if options_pressed and not prev_options_pressed:
                        print("[main] Options/O pressed. Returning to function menu.")
                        break
                    prev_options_pressed = options_pressed

                    if sensor_hub is not None:
                        sensor_snapshot = sensor_hub.read()
                        if args.sensor_use_fsr and sensor_snapshot.foot_load is not None:
                            if args.sensor_debug:
                                load = sensor_snapshot.foot_load
                                print(
                                    f"[sensor] FSR L={load.left:.3f} R={load.right:.3f} "
                                    f"ratio L={load.left_ratio:.2f} R={load.right_ratio:.2f}"
                                )
    
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
    
    
                    stop_pressed = state.button(reader.BTN_CIRCLE)
                    if stop_pressed:
                        if not prev_stop_pressed:
                            print("[main] C/Circle pressed. Hard stop to STANDING.")
                        prev_stop_pressed = True
                        engine.reset()
                        arm_dance.reset()
                        getup.reset()
                        single_support.stop()
                        standing_hold_active = True
                        pose = dict(STANDING)

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
                        single_support.stop()
                        standing_hold_active = False
                        label = getup.start(last_pose, mode=args.getup_mode)
                        print(f"[main] R1/G pressed. Running {args.getup_mode} get-up sequence from step {label}.")
                    prev_getup_pressed = getup_pressed
    
                    getup_back_pressed = state.button(reader.BTN_GETUP_BACK)
                    if getup_back_pressed and not prev_getup_back_pressed:
                        engine.reset()
                        arm_dance.reset()
                        single_support.stop()
                        standing_hold_active = False
                        label = getup.start(last_pose, mode="back")
                        print(f"[main] B pressed. Running back get-up sequence from step {label}.")
                    prev_getup_back_pressed = getup_back_pressed
    
                    l1_pressed = state.button(reader.BTN_L1)
                    if l1_pressed and not prev_l1_pressed and not getup.running:
                        enabled = arm_dance.toggle()
                        engine.reset()
                        single_support.stop()
                        standing_hold_active = not enabled
                        print("[main] L1 arm dance ON." if enabled else "[main] L1 arm dance OFF - returning to STANDING.")
                    prev_l1_pressed = l1_pressed

                    single_support_pressed = state.button(reader.BTN_CROSS)
                    if single_support_pressed and not prev_single_support_pressed and not getup.running:
                        engine.reset()
                        arm_dance.reset()
                        if single_support.running:
                            single_support.stop()
                            standing_hold_active = True
                            print("[main] X/Cross single-support OFF - returning to STANDING.")
                        else:
                            single_support.start(next_single_support_leg, current_pose=last_pose)
                            standing_hold_active = False
                            swing_leg = "left" if next_single_support_leg == "right" else "right"
                            print(f"[main] X/Cross single-support ON: support={next_single_support_leg}, lifted={swing_leg}.")
                            next_single_support_leg = "left" if next_single_support_leg == "right" else "right"
                    prev_single_support_pressed = single_support_pressed
    
                    if state.button(reader.BTN_TRIANGLE):
                        print("[main] Triangle/E pressed. Resetting walking engine and arm dance.")
                        engine.reset()
                        arm_dance.reset()
                        getup.reset()
                        single_support.stop()
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
                    elif single_support.running:
                        vy = 0.0
                        turn_cmd = 0.0
                        side_cmd = 0.0
                        motion_requested = False
                        pose = single_support.update()
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

                    if balance is not None and not pose_from_getup:
                        now = time.monotonic()
                        balance_dt = now - last_balance_t
                        last_balance_t = now
                        reading = sensor_snapshot.imu if sensor_snapshot is not None else None
                        if reading is not None and reading.balance_ready(
                            args.imu_min_gyro_cal,
                            args.imu_min_accel_cal,
                        ):
                            support_leg = single_support.support_leg if single_support.running else engine.support_leg
                            foot_load = sensor_snapshot.foot_load if args.sensor_use_fsr else None
                            if foot_load is not None and foot_load.total >= args.fsr_min_total_load:
                                if foot_load.left_ratio >= args.fsr_support_ratio:
                                    support_leg = "left"
                                elif foot_load.right_ratio >= args.fsr_support_ratio:
                                    support_leg = "right"
                            pose = balance.apply(
                                pose,
                                roll_deg=reading.roll_deg,
                                pitch_deg=reading.pitch_deg,
                                dt=balance_dt,
                                support_leg=support_leg,
                            )
                            balance_has_valid_imu = True
                        elif balance_has_valid_imu:
                            balance.reset()
                            balance_has_valid_imu = False
                    elif balance is not None and balance_has_valid_imu:
                        balance.reset()
                        balance_has_valid_imu = False
    
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
        if sensor_hub is not None:
            sensor_hub.close()
        reader.quit()
        print("[main] PS4 mode exited.")


def main() -> None:
    args = Config()
    if args.getup:
        run_getup(args)
        return

    from .menu import run_menu

    while True:
        choice = run_menu(args.joystick_index)
        if choice == "quit":
            print("[main] Exiting function menu.")
            return
        if choice == "walking":
            try:
                run_ps4(args)
            except Exception as exc:
                print(f"[main] Walking mode unavailable: {exc}")
                time.sleep(1.5)
        elif choice == "vision":
            try:
                from .vision_main import run_vision

                run_vision(args)
            except Exception as exc:
                print(f"[main] Camera Mimic unavailable: {exc}")
                time.sleep(1.5)
        elif choice == "terrain":
            try:
                from .terrain_main import run_terrain

                run_terrain(args)
            except Exception as exc:
                print(f"[main] Terrain Auto unavailable: {exc}")
                time.sleep(1.5)

if __name__ == "__main__":
    main()
