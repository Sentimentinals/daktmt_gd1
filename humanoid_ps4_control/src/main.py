from __future__ import annotations

import time

from .backends import make_backend
from .config import Config


def run_getup(args: Config) -> None:
    """Run one get-up sequence directly, without keyboard input."""
    from .getup import GetupEngine
    from .walking_engine import STANDING

    dt = args.update_ms / 1000.0
    engine = GetupEngine(dt=dt, mode=args.getup_mode, speed=args.getup_speed)
    debug_ids = list(range(9, 26))

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


def run_keyboard(args: Config) -> None:
    """
    Real-time walking mode.

    Keyboard:
      W/S        : walk forward/backward
      A/D        : turn left/right
      J/K        : side walk left/right
      L/M        : toggle standing arm dance
      X          : toggle single-leg support test
      V          : handshake
      G          : run get-up sequence
      B          : run back get-up sequence
      C          : stop and hold standing
      E/T        : reset walking engine
      O/Escape   : return to menu
      Q          : quit
    """
    from .keyboard_input import KeyboardReader, LiveCameraPreview
    from .walking_engine import DynamicWalkingEngine, SingleSupportTestEngine, STANDING
    from .arm_dance import ArmDanceEngine, HandshakeEngine
    from .getup import GetupEngine
    from .balance import BalanceConfig, IMUBalanceController
    from .sensors import RobotSensorHub

    backend = make_backend(mode=args.backend, port=args.port, baudrate=args.baudrate, csv_path=args.csv)

    poll_hz = int(1000 / args.update_ms)
    reader = KeyboardReader(poll_rate_hz=poll_hz)
    reader.init()
    camera_preview = LiveCameraPreview(
        width=args.vision_camera_width,
        height=args.vision_camera_height,
        fps=args.vision_fps,
    )

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
    handshake = HandshakeEngine(
        dt=args.update_ms / 1000.0,
        offer_s=args.handshake_offer_s,
        contact_timeout_s=args.handshake_contact_timeout_s,
        release_timeout_s=args.handshake_release_timeout_s,
        frequency_hz=args.handshake_frequency_hz,
        cycles=args.handshake_cycles,
        lift_pwm=args.handshake_lift_pwm,
        shoulder_pwm=args.handshake_shoulder_pwm,
        elbow_pwm=args.handshake_elbow_pwm,
        shake_pwm=args.handshake_shake_pwm,
        contact_threshold=args.hand_fsr_contact_threshold,
        release_threshold=args.hand_fsr_release_threshold,
        stable_frames=args.hand_fsr_stable_frames,
    )
    getup = GetupEngine(
        dt=args.update_ms / 1000.0,
        mode=args.getup_mode,
        speed=args.getup_speed,
    )
    prev_dance_pressed = False
    prev_stop_pressed = False
    prev_getup_pressed = False
    prev_getup_back_pressed = False
    prev_single_support_pressed = False
    prev_handshake_pressed = False
    prev_menu_pressed = False
    next_single_support_leg = "right"
    last_pose = dict(STANDING)
    standing_hold_active = True

    balance = None
    sensor_hub = None
    sensor_snapshot = None
    last_balance_t = time.monotonic()
    balance_has_valid_imu = False
    previous_handshake_status = handshake.status
    if args.sensor_feedback:
        sensor_hub = RobotSensorHub(
            port=args.sensor_port,
            baudrate=args.sensor_baudrate,
            timeout_s=args.sensor_timeout_s,
            use_imu=args.sensor_use_imu,
            use_hand_fsr=args.sensor_use_hand_fsr,
            imu_roll_sign=args.imu_roll_sign,
            imu_pitch_sign=args.imu_pitch_sign,
            imu_yaw_sign=args.imu_yaw_sign,
            imu_vertical_mount=args.imu_vertical_mount,
            imu_board_face_sign=args.imu_board_face_sign,
            hand_fsr_invert=args.hand_fsr_invert,
            hand_fsr_filter_alpha=args.hand_fsr_filter_alpha,
            hand_fsr_zero_raw=args.hand_fsr_zero_raw,
            hand_fsr_full_raw=args.hand_fsr_full_raw,
        )
        sensor_hub.open()
        print(f"[main] Sensor feedback enabled: ESP32 serial port={args.sensor_port}.")

    if args.imu_balance and (sensor_hub is None or not args.sensor_use_imu):
        print("[main] IMU balance requested but IMU sensor feedback is disabled.")

    print(
        "\n[Keyboard Mode - Real-time ZMP] W/S walk, A/D turn, J/K side, "
        "X single support, V handshake, L/M dance, G get-up, "
        "B get-up back, C stop, E/T reset, O/Esc menu, Q quit\n"
    )

    camera_preview.start()
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

                    menu_pressed = state.menu
                    if menu_pressed and not prev_menu_pressed:
                        print("[main] O/Escape pressed. Returning to function menu.")
                        break
                    prev_menu_pressed = menu_pressed

                    if sensor_hub is not None:
                        sensor_snapshot = sensor_hub.read()
                        if args.sensor_use_hand_fsr and sensor_snapshot.hand_force is not None:
                            if args.sensor_debug:
                                hand = sensor_snapshot.hand_force
                                print(
                                    f"[sensor] hand force={hand.force:.3f} "
                                    f"voltage={hand.voltage:.3f}V raw={hand.raw}"
                                )
    
                    vy = state.forward * args.walk_speed
                    turn_cmd = state.turn * args.turn_speed
                    side_cmd = state.side * args.side_speed
                    motion_requested = vy != 0.0 or turn_cmd != 0.0 or side_cmd != 0.0

                    stop_pressed = state.stop
                    if stop_pressed:
                        if not prev_stop_pressed:
                            print("[main] C pressed. Hard stop to STANDING.")
                        prev_stop_pressed = True
                        engine.reset()
                        arm_dance.reset()
                        handshake.reset()
                        getup.reset()
                        single_support.stop()
                        standing_hold_active = True
                        pose = dict(STANDING)

                        try:
                            backend.send(pose, duration_ms=args.stop_ms, force=True)
                            last_pose = dict(pose)
                        except Exception as exc:
                            print(f"[main] Backend send exception: {exc}")
                        camera_preview.render("STOP / STANDING")
                        continue
                    prev_stop_pressed = False
    
                    getup_pressed = state.getup
                    if getup_pressed and not prev_getup_pressed:
                        engine.reset()
                        arm_dance.reset()
                        handshake.reset()
                        single_support.stop()
                        standing_hold_active = False
                        label = getup.start(last_pose, mode=args.getup_mode)
                        print(f"[main] G pressed. Running {args.getup_mode} get-up sequence from step {label}.")
                    prev_getup_pressed = getup_pressed
    
                    getup_back_pressed = state.getup_back
                    if getup_back_pressed and not prev_getup_back_pressed:
                        engine.reset()
                        arm_dance.reset()
                        handshake.reset()
                        single_support.stop()
                        standing_hold_active = False
                        label = getup.start(last_pose, mode="back")
                        print(f"[main] B pressed. Running back get-up sequence from step {label}.")
                    prev_getup_back_pressed = getup_back_pressed
    
                    dance_pressed = state.dance
                    if dance_pressed and not prev_dance_pressed and not getup.running:
                        handshake.reset()
                        enabled = arm_dance.toggle()
                        engine.reset()
                        single_support.stop()
                        standing_hold_active = not enabled
                        print("[main] L/M arm dance ON." if enabled else "[main] L/M arm dance OFF - returning to STANDING.")
                    prev_dance_pressed = dance_pressed

                    handshake_pressed = state.handshake
                    if handshake_pressed and not prev_handshake_pressed and not getup.running:
                        if handshake.running:
                            handshake.cancel()
                            print("[main] V handshake canceled - returning to STANDING.")
                        elif not standing_hold_active or motion_requested or arm_dance.running or single_support.running:
                            print("[main] Handshake requires the robot to be stationary in STANDING.")
                        elif sensor_snapshot is None or sensor_snapshot.hand_force is None:
                            print("[main] Handshake unavailable: hand FSR data is missing.")
                        elif sensor_snapshot.hand_force.force >= args.hand_fsr_contact_threshold:
                            print("[main] Handshake unavailable: release the hand FSR first.")
                        else:
                            engine.reset()
                            arm_dance.reset()
                            single_support.stop()
                            handshake.start(last_pose)
                            standing_hold_active = False
                            previous_handshake_status = handshake.status
                            print("[main] V handshake started. Waiting for a hand grip.")
                    prev_handshake_pressed = handshake_pressed

                    single_support_pressed = state.single_support
                    if single_support_pressed and not prev_single_support_pressed and not getup.running:
                        engine.reset()
                        arm_dance.reset()
                        handshake.reset()
                        if single_support.running:
                            single_support.stop()
                            standing_hold_active = True
                            print("[main] X single-support OFF - returning to STANDING.")
                        else:
                            single_support.start(next_single_support_leg, current_pose=last_pose)
                            standing_hold_active = False
                            swing_leg = "left" if next_single_support_leg == "right" else "right"
                            print(f"[main] X single-support ON: support={next_single_support_leg}, lifted={swing_leg}.")
                            next_single_support_leg = "left" if next_single_support_leg == "right" else "right"
                    prev_single_support_pressed = single_support_pressed
    
                    if state.reset:
                        print("[main] E/T pressed. Resetting walking engine and arm dance.")
                        engine.reset()
                        arm_dance.reset()
                        handshake.reset()
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
                    elif handshake.running:
                        vy = 0.0
                        turn_cmd = 0.0
                        side_cmd = 0.0
                        motion_requested = False
                        hand_force = (
                            sensor_snapshot.hand_force.force
                            if sensor_snapshot is not None and sensor_snapshot.hand_force is not None
                            else None
                        )
                        pose = handshake.update(hand_force)
                        if handshake.status != previous_handshake_status:
                            print(f"[main] Handshake: {handshake.status}.")
                            previous_handshake_status = handshake.status
                        if not handshake.running:
                            standing_hold_active = True
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

                    if getup.running:
                        camera_status = f"GET-UP: {getup.label.upper()}"
                    elif handshake.running:
                        camera_status = f"HANDSHAKE: {handshake.status}"
                    elif arm_dance.running:
                        camera_status = "ARM DANCE"
                    elif single_support.running:
                        camera_status = f"SINGLE SUPPORT: {single_support.support_leg.upper()}"
                    else:
                        directions = []
                        if vy > 0.0:
                            directions.append("FORWARD")
                        elif vy < 0.0:
                            directions.append("BACKWARD")
                        if turn_cmd > 0.0:
                            directions.append("TURN LEFT")
                        elif turn_cmd < 0.0:
                            directions.append("TURN RIGHT")
                        if side_cmd > 0.0:
                            directions.append("SIDE LEFT")
                        elif side_cmd < 0.0:
                            directions.append("SIDE RIGHT")
                        camera_status = " + ".join(directions) if directions else "STANDING"
                    camera_preview.render(camera_status)
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
        camera_preview.close()
        reader.quit()
        print("[main] Keyboard mode exited.")


def main() -> None:
    args = Config()
    if args.getup:
        run_getup(args)
        return

    from .menu import run_menu

    while True:
        choice = run_menu()
        if choice == "quit":
            print("[main] Exiting function menu.")
            return
        if choice == "walking":
            try:
                run_keyboard(args)
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
