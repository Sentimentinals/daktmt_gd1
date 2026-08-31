from __future__ import annotations

import threading
import time
from concurrent.futures import ThreadPoolExecutor

from .backends import make_backend
from .config import Config


def run_manual(args: Config, dashboard, camera_ready: bool) -> None:
    from .walking_engine import (
        DynamicWalkingEngine,
        STANDING,
    )
    from .arm_dance import ArmDanceEngine
    from .getup import GetupEngine
    from .balance import (
        BalanceConfig,
        IMUBalanceController,
        PushRecoveryConfig,
        PushRecoveryController,
        RecoveryState,
        angle_error_deg,
        configured_fall_detector,
        extend_arms_forward,
        update_fall_detector,
    )
    from .sensors import DepthObstacleGuard, RobotSensorHub

    backend = make_backend(mode=args.backend, port=args.port, baudrate=args.baudrate, csv_path=args.csv)

    obstacle_guard = DepthObstacleGuard(
        stop_distance_mm=args.tof_obstacle_stop_mm,
        clear_margin_mm=args.tof_obstacle_clear_margin_mm,
        stable_frames=args.tof_obstacle_stable_frames,
    )
    engine = DynamicWalkingEngine(
        dt=args.update_ms / 1000.0,
        t_step=args.t_step,
        t_dbl=args.t_dbl,
        max_step_len=args.walk_step_length_mm,
        max_turn_step_len=args.max_turn_step_len,
        max_side_step_len=args.max_side_step_len,
        step_height=args.walk_step_height_mm,
        crouch_depth_mm=args.walk_crouch_depth_mm,
        zmp_support_ratio=args.zmp_support_ratio,
        ankle_roll_gain=args.ankle_roll_gain,
        step_x_ratio=1.0,
        landing_gap_mm=args.walk_step_length_mm,
        lift_start_phase=args.walk_lift_start_phase,
        swing_advance_end_phase=args.walk_swing_advance_end_phase,
        lift_end_phase=args.walk_lift_end_phase,
        landing_roll_release_start=args.walk_landing_roll_release_start,
        arm_swing_pwm=args.arm_swing_pwm,
        arm_right_dir=args.arm_right_dir,
        arm_left_dir=args.arm_left_dir,
        crouch_transition_s=args.walk_crouch_transition_s,
    )
    recovery_engine = DynamicWalkingEngine(
        dt=args.update_ms / 1000.0,
        t_step=args.push_recovery_step_time_s,
        t_dbl=args.t_dbl,
        max_step_len=args.walk_step_length_mm,
        max_side_step_len=args.max_side_step_len,
        step_height=args.push_recovery_step_height_mm,
        step_x_ratio=1.0,
        landing_gap_mm=0.0,
    )
    arm_dance = ArmDanceEngine(
        dt=args.update_ms / 1000.0,
        period_s=args.dance_period,
        transition_s=args.dance_transition,
        shoulder_pwm=args.dance_shoulder_pwm,
        elbow_pwm=args.dance_elbow_pwm,
        lift_pwm=args.dance_lift_pwm,
        head_pwm=args.dance_head_pwm,
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
    last_pose = dict(STANDING)
    standing_hold_active = True

    balance = None
    fall_detector = None
    imu_reference = None
    reference_future = None
    reference_executor = None
    reference_cancel = threading.Event()
    recovery = None
    recovery_step_active = False
    recovery_status = "STABLE"
    previous_recovery_status = recovery_status
    sensor_hub = None
    sensor_snapshot = None
    last_balance_t = time.monotonic()
    last_fall_t = time.monotonic()
    balance_has_valid_imu = False
    obstacle_blocked = False
    obstacle_mm = None
    previous_obstacle_blocked = False
    sensor_required = args.sensor_feedback or args.fall_detection_enabled
    if sensor_required:
        sensor_hub = RobotSensorHub(
            port=args.sensor_port,
            baudrate=args.sensor_baudrate,
            timeout_s=args.sensor_timeout_s,
            depth_timeout_s=args.sensor_depth_timeout_s,
            use_imu=args.sensor_use_imu or args.fall_detection_enabled,
            use_foot_fsr=args.sensor_use_foot_fsr,
            use_depth=args.sensor_use_depth,
            imu_roll_sign=args.imu_roll_sign,
            imu_pitch_sign=args.imu_pitch_sign,
            imu_yaw_sign=args.imu_yaw_sign,
            imu_vertical_mount=args.imu_vertical_mount,
            imu_board_face_sign=args.imu_board_face_sign,
            foot_fsr_invert=args.foot_fsr_invert,
            foot_fsr_filter_alpha=args.foot_fsr_filter_alpha,
            foot_fsr_zero_raw=args.foot_fsr_zero_raw,
            foot_fsr_full_raw=args.foot_fsr_full_raw,
        )
        try:
            sensor_hub.open(wait_for_connection=False)
            print("[main] Sensor connection started in background; web control is independent.")
        except Exception as exc:
            sensor_hub = None
            print(f"[main] Sensors disabled: {exc}. Keyboard control remains available.")

    if (args.imu_balance or args.fall_detection_enabled) and sensor_hub is None:
        print("[main] IMU unavailable. Control remains active without balance/fall protection.")

    try:
        with backend:
            dashboard.set_runtime("manual", "Preparing standing pose")
            dashboard.publish(
                STANDING, engine.telemetry_snapshot(), None,
                "PREPARING STANDING", False, camera_ready, "FALL IMU WAIT",
            )
            backend.send(STANDING, duration_ms=1200, force=True)
            time.sleep(1.2)
            last_pose = dict(engine.ready_pose)
            standing_hold_active = False
            dashboard.set_runtime("manual", "Manual control ready")
            try:
                while True:
                    loop_started = time.monotonic()
                    state = dashboard.control_state()
                    if not state.armed or state.mode != "manual":
                        break

                    if (
                        imu_reference is None and sensor_hub is not None
                        and (args.imu_balance or args.fall_detection_enabled)
                    ):
                        stationary = (
                            not any((state.forward, state.turn, state.side, state.dance, state.getup,
                                     state.getup_back, state.reset, state.stop))
                            and not arm_dance.running and not getup.running and engine.is_idle_ready()
                        )
                        if not stationary:
                            reference_cancel.set()
                        if reference_future is not None and reference_future.done():
                            try:
                                captured = reference_future.result()
                            except Exception as exc:
                                captured = None
                                print(f"[main] IMU reference unavailable: {exc}")
                            reference_future = None
                            if stationary and not reference_cancel.is_set():
                                imu_reference = captured
                        if stationary and imu_reference is None and reference_future is None:
                            reference_cancel.clear()
                            if reference_executor is None:
                                reference_executor = ThreadPoolExecutor(
                                    max_workers=1, thread_name_prefix="imu-reference",
                                )
                            reference_future = reference_executor.submit(
                                sensor_hub.capture_imu_reference,
                                sample_seconds=args.imu_reference_seconds,
                                timeout_s=args.imu_reference_timeout_s,
                                min_gyro_cal=args.imu_min_gyro_cal,
                                min_accel_cal=args.imu_min_accel_cal,
                                max_rms_deg=args.imu_reference_max_rms_deg,
                                cancel_event=reference_cancel,
                            )
                        if imu_reference is not None:
                            target_roll, target_pitch = imu_reference
                            if args.imu_balance:
                                balance = IMUBalanceController(
                                    BalanceConfig(
                                        target_roll_deg=target_roll,
                                        target_pitch_deg=target_pitch,
                                        max_correction_deg=args.balance_limit_deg,
                                        roll_deadband_deg=args.balance_deadband_deg,
                                        pitch_deadband_deg=args.balance_deadband_deg,
                                    )
                                )
                            if args.imu_balance and args.push_recovery_enabled:
                                recovery = PushRecoveryController(
                                    PushRecoveryConfig(
                                        warning_tilt_deg=args.push_recovery_warning_tilt_deg,
                                        recovery_tilt_deg=args.push_recovery_tilt_deg,
                                        safe_lower_tilt_deg=args.push_recovery_safe_lower_tilt_deg,
                                        recovery_rate_deg_s=args.push_recovery_rate_deg_s,
                                        settle_tilt_deg=args.push_recovery_settle_tilt_deg,
                                        recovery_step_forward_cmd=args.push_recovery_step_forward_cmd,
                                        recovery_step_side_cmd=args.push_recovery_step_side_cmd,
                                        recovery_step_timeout_s=args.push_recovery_timeout_s,
                                        counter_lean_s=args.push_recovery_counter_lean_s,
                                        counter_lean_deg=args.push_recovery_counter_lean_deg,
                                    )
                                )
                            if args.fall_detection_enabled:
                                fall_detector = configured_fall_detector(args)
                            print(
                                f"[main] IMU reference roll={target_roll:.2f}, pitch={target_pitch:.2f}; "
                                f"balance={'ON' if balance is not None else 'OFF'}, "
                                f"fall protection={'ON' if fall_detector is not None else 'OFF'}."
                            )

                    if sensor_hub is not None:
                        sensor_snapshot = sensor_hub.read()
                        depth = sensor_snapshot.depth
                        guarded_blocked, guarded_mm = obstacle_guard.update(depth)
                        obstacle_blocked = guarded_blocked if depth is not None else False
                        obstacle_mm = guarded_mm if depth is not None else None
                        if obstacle_blocked != previous_obstacle_blocked:
                            print(
                                f"[main] ToF obstacle {'detected' if obstacle_blocked else 'cleared'}"
                                f"{f' at {obstacle_mm} mm' if obstacle_mm is not None else ''}."
                            )
                            previous_obstacle_blocked = obstacle_blocked
    
                    vy = state.forward * args.walk_speed
                    turn_cmd = state.turn * args.turn_speed
                    side_cmd = state.side * args.side_speed
                    motion_requested = vy != 0.0 or turn_cmd != 0.0 or side_cmd != 0.0

                    head_turn_cmd = turn_cmd

                    stop_pressed = state.stop
                    if stop_pressed:
                        if fall_detector is not None and fall_detector.triggered:
                            pose = extend_arms_forward(
                                last_pose,
                                args.fall_arm_forward_pwm,
                            )
                            backend.send(pose, duration_ms=args.stop_ms, force=True)
                            last_pose = dict(pose)
                            dashboard.set_runtime("manual", "Fall - holding protective pose")
                            continue
                        if not prev_stop_pressed:
                            print("[main] C pressed. Hard stop to STANDING.")
                        prev_stop_pressed = True
                        engine.reset()
                        arm_dance.reset()
                        getup.reset()
                        recovery_engine.reset()
                        recovery_step_active = False
                        if recovery is not None:
                            recovery.reset()
                            recovery_status = "STABLE"
                        standing_hold_active = True
                        pose = dict(STANDING)

                        try:
                            backend.send(pose, duration_ms=args.stop_ms, force=True)
                            last_pose = dict(pose)
                        except Exception as exc:
                            print(f"[main] Backend send exception: {exc}")
                        dashboard.set_runtime("manual", "Stop / standing")
                        continue
                    prev_stop_pressed = False

                    getup_pressed = state.getup
                    if getup_pressed and not prev_getup_pressed:
                        if fall_detector is not None:
                            fall_detector.reset()
                        engine.reset()
                        arm_dance.reset()
                        recovery_engine.reset()
                        recovery_step_active = False
                        if recovery is not None:
                            recovery.reset()
                            recovery_status = "STABLE"
                        standing_hold_active = False
                        protected_pose = extend_arms_forward(last_pose, args.fall_arm_forward_pwm)
                        label = getup.start(protected_pose, mode="front")
                        print(f"[main] G pressed. Running front get-up sequence from step {label}.")
                    prev_getup_pressed = getup_pressed

                    getup_back_pressed = state.getup_back
                    if getup_back_pressed and not prev_getup_back_pressed:
                        if fall_detector is not None:
                            fall_detector.reset()
                        engine.reset()
                        arm_dance.reset()
                        standing_hold_active = False
                        label = getup.start(last_pose, mode="back")
                        print(f"[main] B pressed. Running back get-up sequence from step {label}.")
                    prev_getup_back_pressed = getup_back_pressed
    
                    dance_pressed = state.dance
                    if (
                        dance_pressed
                        and not prev_dance_pressed
                        and not getup.running
                    ):
                        enabled = arm_dance.toggle()
                        engine.reset()
                        standing_hold_active = not enabled
                        print("[main] L/M arm dance ON." if enabled else "[main] L/M arm dance OFF - returning to STANDING.")
                    prev_dance_pressed = dance_pressed

                    if state.reset:
                        if fall_detector is not None and fall_detector.triggered:
                            reading = sensor_snapshot.imu if sensor_snapshot is not None else None
                            reset_tilt = None
                            if imu_reference is not None and reading is not None and reading.balance_ready(
                                args.imu_min_gyro_cal,
                                args.imu_min_accel_cal,
                            ):
                                reset_tilt = max(
                                    abs(angle_error_deg(reading.roll_deg, imu_reference[0])),
                                    abs(angle_error_deg(reading.pitch_deg, imu_reference[1])),
                                )
                            if reset_tilt is None or reset_tilt > args.fall_reset_tilt_deg:
                                print("[main] FALL reset blocked. Hold the robot upright, then press E/T again.")
                                dashboard.set_runtime("manual", "Fall reset blocked")
                                continue
                        print("[main] E/T pressed. Resetting walking engine and arm dance.")
                        engine.reset()
                        arm_dance.reset()
                        getup.reset()
                        recovery_engine.reset()
                        recovery_step_active = False
                        if recovery is not None:
                            recovery.reset()
                            recovery_status = "STABLE"
                        if fall_detector is not None:
                            fall_detector.reset()
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
                    elif not motion_requested and engine.is_idle_ready():
                        engine.reset()
                        standing_hold_active = True
                        pose = dict(engine.ready_pose)
                    else:
                        if motion_requested and standing_hold_active:
                            engine.reset()
                            standing_hold_active = False
                        pose = engine.update(vy, turn_cmd=turn_cmd, side_cmd=side_cmd)

                    fall_active = False
                    fall_now = time.monotonic()
                    fall_dt = fall_now - last_fall_t
                    last_fall_t = fall_now
                    if fall_detector is not None and not getup.running:
                        reading = sensor_snapshot.imu if sensor_snapshot is not None else None
                        was_triggered = fall_detector.triggered
                        update_fall_detector(
                            fall_detector,
                            reading,
                            imu_reference,
                            fall_dt,
                            args,
                        )
                        if fall_detector.triggered:
                            fall_active = True
                            vy = 0.0
                            turn_cmd = 0.0
                            side_cmd = 0.0
                            motion_requested = False
                            if not was_triggered:
                                engine.reset()
                                arm_dance.reset()
                                recovery_engine.reset()
                                recovery_step_active = False
                                if recovery is not None:
                                    recovery.reset()
                                if balance is not None:
                                    balance.reset()
                                standing_hold_active = False
                                print(f"[main] FALL detected: {fall_detector.reason}. Arms moving forward.")
                            pose = extend_arms_forward(
                                last_pose,
                                args.fall_arm_forward_pwm,
                            )
                        elif was_triggered:
                            standing_hold_active = True
                            pose = dict(STANDING)
                            print("[main] IMU upright again. Arms returned to STANDING.")

                    if balance is not None and not pose_from_getup and not fall_active:
                        now = time.monotonic()
                        balance_dt = now - last_balance_t
                        last_balance_t = now
                        reading = sensor_snapshot.imu if sensor_snapshot is not None else None
                        if reading is not None and reading.balance_ready(
                            args.imu_min_gyro_cal,
                            args.imu_min_accel_cal,
                        ):
                            support_leg = (
                                recovery_engine.support_leg
                                if recovery_step_active
                                else engine.support_leg
                            )
                            recovery_roll_offset = 0.0
                            recovery_pitch_offset = 0.0
                            walking_active = (
                                not recovery_step_active
                                and (motion_requested or not engine.is_idle_ready())
                            )
                            recovery_allowed = recovery_step_active or not arm_dance.running
                            if recovery is not None and recovery_allowed:
                                decision = recovery.update(
                                    -angle_error_deg(reading.roll_deg, balance.config.target_roll_deg),
                                    -angle_error_deg(reading.pitch_deg, balance.config.target_pitch_deg),
                                    balance_dt,
                                    walking=walking_active,
                                    now=now,
                                )
                                recovery_status = f"{decision.state.value}: {decision.reason}"
                                recovery_roll_offset = decision.target_roll_offset_deg
                                recovery_pitch_offset = decision.target_pitch_offset_deg
                                if decision.start_step:
                                    recovery_engine.reset()
                                    recovery_step_active = True
                                    engine.reset()
                                    standing_hold_active = False
                                    print("[main] Push recovery: starting near-in-place stomp.")
                                if decision.safe_lower:
                                    recovery_step_active = False
                                    recovery_engine.reset()
                                    engine.reset()
                                    standing_hold_active = True
                                    pose = dict(STANDING)
                                elif recovery_step_active:
                                    pose = recovery_engine.update(
                                        decision.forward_cmd if recovery_engine.step_count == 0 else 0.0,
                                        side_cmd=(
                                            decision.side_cmd
                                            if recovery_engine.step_count == 0
                                            else 0.0
                                        ),
                                    )
                                    support_leg = recovery_engine.support_leg
                                    if recovery_engine.is_idle_ready():
                                        completed = recovery.complete_step(now)
                                        recovery_status = f"{completed.state.value}: {completed.reason}"
                                        recovery_roll_offset = completed.target_roll_offset_deg
                                        recovery_pitch_offset = completed.target_pitch_offset_deg
                                        recovery_step_active = False
                            balance_pose_enabled = (
                                not walking_active
                                or recovery_step_active
                            )
                            if balance_pose_enabled:
                                pose = balance.apply(
                                    pose,
                                    roll_deg=reading.roll_deg,
                                    pitch_deg=reading.pitch_deg,
                                    dt=balance_dt,
                                    support_leg=support_leg,
                                    target_roll_offset_deg=recovery_roll_offset,
                                    target_pitch_offset_deg=recovery_pitch_offset,
                                )
                            else:
                                balance.reset()
                            balance_has_valid_imu = True
                        else:
                            sensor_safe_lower = (
                                recovery_step_active
                                or (
                                    recovery is not None
                                    and recovery.state is RecoveryState.SAFE_LOWER
                                )
                            )
                            if sensor_safe_lower:
                                if recovery is not None:
                                    recovery.force_safe_lower("IMU stream lost")
                                recovery_status = "safe-lower: IMU stream lost"
                                recovery_step_active = False
                                recovery_engine.reset()
                                engine.reset()
                                standing_hold_active = True
                                pose = dict(STANDING)
                            if balance_has_valid_imu:
                                balance.reset()
                                balance_has_valid_imu = False
                    elif balance is not None and balance_has_valid_imu:
                        balance.reset()
                        balance_has_valid_imu = False

                    if recovery_status != previous_recovery_status:
                        print(f"[main] Push recovery: {recovery_status}.")
                        previous_recovery_status = recovery_status

                    if not pose_from_getup and not arm_dance.running and not fall_active:
                        head_target = STANDING[25] + args.head_pan_direction * args.head_pan_pwm * (
                            1 if head_turn_cmd > 0.0 else -1 if head_turn_cmd < 0.0 else 0
                        )
                        pose[25] = round(head_target)
    
                    try:
                        backend.send(pose, duration_ms=args.update_ms)
                        last_pose = dict(pose)
                    except Exception as exc:
                        print(f"[main] Backend send exception: {exc}")

                    if fall_active:
                        camera_status = "FALL DETECTED - ARMS FORWARD"
                    elif getup.running:
                        camera_status = f"GET-UP: {getup.label.upper()}"
                    elif arm_dance.running:
                        camera_status = "ARM DANCE"
                    elif recovery is not None and recovery.state is not RecoveryState.STABLE:
                        camera_status = f"BALANCE: {recovery_status.upper()}"
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
                        camera_status = " + ".join(directions) if directions else "WALK READY"
                    if obstacle_blocked and not fall_active:
                        camera_status += f" | TOF NEAR {obstacle_mm} MM (MANUAL)"
                    gait_state = (
                        recovery_engine.telemetry_snapshot()
                        if recovery_step_active
                        else engine.telemetry_snapshot()
                    )
                    dashboard.publish(
                        pose=last_pose,
                        gait=gait_state,
                        sensor_snapshot=sensor_snapshot,
                        status=camera_status,
                        active=(
                            motion_requested
                            or not engine.is_idle_ready()
                            or recovery_step_active
                            or arm_dance.running
                            or getup.running
                            or fall_active
                        ),
                        camera_ready=camera_ready,
                        balance_status=(
                            "FALL ACTIVE"
                            if fall_active
                            else f"{recovery_status} | "
                            f"{'FALL READY' if fall_detector is not None else 'FALL IMU WAIT'}"
                        ),
                    )
                    dashboard.set_runtime("manual", camera_status)
                    remaining = args.update_ms / 1000.0 - (time.monotonic() - loop_started)
                    if remaining > 0.0:
                        time.sleep(remaining)
            except KeyboardInterrupt:
                print("\n[main] Ctrl+C received. Stopping control output.")
                raise
            finally:
                try:
                    exit_pose = (
                        last_pose
                        if (fall_detector is not None and fall_detector.triggered) or getup.running
                        else STANDING
                    )
                    backend.send(exit_pose, duration_ms=args.stop_ms, force=True)
                    time.sleep(args.stop_ms / 1000.0)
                except Exception as exc:
                    print(f"[main] Backend send exception while stopping: {exc}")
    finally:
        reference_cancel.set()
        if sensor_hub is not None:
            sensor_hub.close()
        if reference_executor is not None:
            reference_executor.shutdown(wait=True)
        dashboard.set_runtime("idle", "Manual control stopped")
        print("[main] Manual web control exited.")


def main() -> None:
    from .camera import HeadlessCamera
    from .gait_dashboard import GaitDashboard, stationary_gait
    from .walking_engine import STANDING

    args = Config()
    camera = HeadlessCamera(
        width=args.vision_camera_width,
        height=args.vision_camera_height,
        fps=args.vision_fps,
    )
    dashboard = GaitDashboard(
        host=args.gait_dashboard_host,
        port=args.gait_dashboard_port,
        stream_hz=args.gait_dashboard_stream_hz,
        command_timeout_s=args.gait_dashboard_command_timeout_s,
        camera=camera,
    )
    dashboard.start()
    camera_ready = camera.start()
    dashboard.publish(
        pose=STANDING,
        gait=stationary_gait(),
        sensor_snapshot=None,
        status="WEB CONTROL READY",
        active=False,
        camera_ready=camera_ready,
        balance_status="IDLE",
    )
    print("[main] Open the dashboard from a laptop on the same LAN, then enable control.")

    try:
        while True:
            state = dashboard.control_state()
            if not state.armed:
                time.sleep(0.05)
                continue
            try:
                if state.mode == "manual":
                    run_manual(args, dashboard, camera_ready)
                elif state.mode == "terrain":
                    from .terrain_main import run_terrain

                    run_terrain(args, dashboard, camera, camera_ready)
                elif state.mode == "follow":
                    from .follow_main import run_follow

                    run_follow(args, dashboard, camera, camera_ready)
                elif state.mode == "pickup":
                    from .pickup_main import run_pickup

                    run_pickup(args, dashboard, camera, camera_ready)
            except Exception as exc:
                dashboard.disarm(f"{state.mode} unavailable: {exc}")
                print(f"[main] {state.mode} unavailable: {exc}")
    except KeyboardInterrupt:
        print("\n[main] Ctrl+C received. Stopping web control.")
    finally:
        dashboard.disarm("Server stopped")
        camera.close()
        dashboard.close()

if __name__ == "__main__":
    main()
