from __future__ import annotations

import time

from .backends import make_backend
from .config import Config


def run_manual(args: Config, dashboard, camera_ready: bool) -> None:
    from .walking_engine import (
        DynamicWalkingEngine,
        SingleSupportTestEngine,
        STANDING,
        clamp_pose_rate,
    )
    from .arm_dance import ArmDanceEngine
    from .getup import GetupEngine
    from .balance import (
        BalanceConfig,
        FallConfig,
        FallDetector,
        IMUBalanceController,
        PushRecoveryConfig,
        PushRecoveryController,
        RecoveryState,
        angle_error_deg,
        extend_arms_forward,
        lower_toward_standing,
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
        command_rate_limit=args.command_rate_limit,
        max_leg_pwm_per_s=args.walk_max_leg_pwm_per_s,
        arm_swing_pwm=args.arm_swing_pwm,
        arm_right_dir=args.arm_right_dir,
        arm_left_dir=args.arm_left_dir,
        arm_smooth_tau=args.arm_smooth_tau,
        arm_min_pwm=args.arm_min_pwm,
        arm_quantum_pwm=args.arm_quantum_pwm,
        crouch_transition_s=args.walk_crouch_transition_s,
    )
    single_support = SingleSupportTestEngine(
        dt=args.update_ms / 1000.0,
        zmp_support_ratio=args.zmp_support_ratio,
        ankle_roll_gain=args.ankle_roll_gain,
        swing_knee_pwm=args.one_foot_swing_knee_pwm,
        arm_lift_pwm=args.one_foot_arm_lift_pwm,
        ramp_s=args.one_foot_ramp_s,
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
        command_rate_limit=1000.0,
    )
    arm_dance = ArmDanceEngine(
        dt=args.update_ms / 1000.0,
        period_s=args.dance_period,
        transition_s=args.dance_transition,
        shoulder_pwm=args.dance_shoulder_pwm,
        elbow_pwm=args.dance_elbow_pwm,
        lift_pwm=args.dance_lift_pwm,
        head_pwm=args.dance_head_pwm,
        smooth_tau=args.dance_smooth_tau,
        max_pwm_per_sec=args.dance_max_pwm_per_sec,
        min_step_pwm=args.dance_min_step_pwm,
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
    next_single_support_leg = "right"
    last_pose = dict(STANDING)
    standing_hold_active = True

    balance = None
    fall_detector = None
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
    head_pwm = float(STANDING[25])
    head_turn_sign = 0
    head_turn_lead_until = 0.0
    last_head_update_t = time.monotonic()
    if args.sensor_feedback:
        sensor_hub = RobotSensorHub(
            port=args.sensor_port,
            baudrate=args.sensor_baudrate,
            timeout_s=args.sensor_timeout_s,
            depth_timeout_s=args.sensor_depth_timeout_s,
            use_imu=args.sensor_use_imu,
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

    if args.imu_balance and (sensor_hub is None or not args.sensor_use_imu):
        print("[main] IMU balance requested but IMU sensor feedback is disabled.")

    try:
        with backend:
            backend.send(STANDING, duration_ms=1200, force=True)
            time.sleep(1.2)
            if args.imu_balance and sensor_hub is not None and args.sensor_use_imu:
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
                    if args.push_recovery_enabled:
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
                        fall_detector = FallDetector(
                            FallConfig(
                                trigger_tilt_deg=args.fall_trigger_tilt_deg,
                                trigger_rate_deg_s=args.fall_trigger_rate_deg_s,
                                hard_tilt_deg=args.fall_hard_tilt_deg,
                                consecutive_frames=args.fall_trigger_frames,
                                reset_tilt_deg=args.fall_reset_tilt_deg,
                                reset_frames=args.fall_reset_frames,
                            )
                        )
                    print(
                        f"[main] IMU balance enabled: reference roll={target_roll:.2f}, "
                        f"pitch={target_pitch:.2f}, limit={args.balance_limit_deg:.1f} deg."
                    )
            elif args.imu_balance:
                print("[main] IMU unavailable. Manual web control remains enabled without balance.")
            backend.send(engine.ready_pose, duration_ms=600, force=True)
            time.sleep(0.6)
            last_pose = dict(engine.ready_pose)
            standing_hold_active = False
            dashboard.set_runtime("manual", "Manual control ready")
            try:
                while True:
                    loop_started = time.monotonic()
                    state = dashboard.control_state()
                    if not state.armed or state.mode != "manual":
                        break

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
                    if vy > 0.0 and obstacle_blocked:
                        vy = 0.0
                        motion_requested = turn_cmd != 0.0 or side_cmd != 0.0

                    head_turn_cmd = turn_cmd
                    turn_sign = 1 if turn_cmd > 0.0 else -1 if turn_cmd < 0.0 else 0
                    now = time.monotonic()
                    if turn_sign != 0 and turn_sign != head_turn_sign:
                        head_turn_sign = turn_sign
                        head_turn_lead_until = now + args.head_turn_lead_s
                    elif turn_sign == 0:
                        head_turn_sign = 0
                        head_turn_lead_until = 0.0
                    if now < head_turn_lead_until:
                        vy = 0.0
                        turn_cmd = 0.0
                        side_cmd = 0.0
                        motion_requested = False

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
                        single_support.reset()
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
                        single_support.reset()
                        standing_hold_active = False
                        label = getup.start(last_pose, mode=args.getup_mode)
                        print(f"[main] G pressed. Running {args.getup_mode} get-up sequence from step {label}.")
                    prev_getup_pressed = getup_pressed

                    getup_back_pressed = state.getup_back
                    if getup_back_pressed and not prev_getup_back_pressed:
                        if fall_detector is not None:
                            fall_detector.reset()
                        engine.reset()
                        arm_dance.reset()
                        single_support.reset()
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
                        single_support.reset()
                        standing_hold_active = not enabled
                        print("[main] L/M arm dance ON." if enabled else "[main] L/M arm dance OFF - returning to STANDING.")
                    prev_dance_pressed = dance_pressed

                    single_support_pressed = state.single_support
                    if (
                        single_support_pressed
                        and not prev_single_support_pressed
                        and not getup.running
                    ):
                        engine.reset()
                        arm_dance.reset()
                        if single_support.active:
                            single_support.stop()
                            standing_hold_active = True
                            print("[main] X one-foot balance OFF - returning to STANDING.")
                        else:
                            recovery_engine.reset()
                            recovery_step_active = False
                            if recovery is not None:
                                recovery.reset()
                                recovery_status = "STABLE"
                            single_support.start(next_single_support_leg, current_pose=last_pose)
                            standing_hold_active = False
                            free_leg = "left" if next_single_support_leg == "right" else "right"
                            print(f"[main] X one-foot balance ON: support={next_single_support_leg}, free_leg={free_leg}.")
                            next_single_support_leg = "left" if next_single_support_leg == "right" else "right"
                    prev_single_support_pressed = single_support_pressed
    
                    if state.reset:
                        if fall_detector is not None and fall_detector.triggered:
                            reading = sensor_snapshot.imu if sensor_snapshot is not None else None
                            reset_tilt = None
                            if reading is not None and reading.balance_ready(
                                args.imu_min_gyro_cal,
                                args.imu_min_accel_cal,
                            ):
                                reset_tilt = max(
                                    abs(angle_error_deg(reading.roll_deg, balance.config.target_roll_deg)),
                                    abs(angle_error_deg(reading.pitch_deg, balance.config.target_pitch_deg)),
                                )
                            if reset_tilt is None or reset_tilt > args.fall_reset_tilt_deg:
                                print("[main] FALL reset blocked. Hold the robot upright, then press E/T again.")
                                dashboard.set_runtime("manual", "Fall reset blocked")
                                continue
                        print("[main] E/T pressed. Resetting walking engine and arm dance.")
                        engine.reset()
                        arm_dance.reset()
                        getup.reset()
                        single_support.reset()
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
                    elif single_support.active:
                        vy = 0.0
                        turn_cmd = 0.0
                        side_cmd = 0.0
                        motion_requested = False
                        pose = single_support.update()
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
                        if reading is not None and reading.balance_ready(
                            args.imu_min_gyro_cal,
                            args.imu_min_accel_cal,
                        ):
                            fall_detector.update(
                                -angle_error_deg(reading.roll_deg, balance.config.target_roll_deg),
                                -angle_error_deg(reading.pitch_deg, balance.config.target_pitch_deg),
                                fall_dt,
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
                                single_support.reset()
                                recovery_engine.reset()
                                recovery_step_active = False
                                if recovery is not None:
                                    recovery.reset()
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
                            if recovery_step_active:
                                support_leg = recovery_engine.support_leg
                            elif single_support.active:
                                support_leg = single_support.support_leg
                            else:
                                support_leg = engine.support_leg
                            recovery_roll_offset = 0.0
                            recovery_pitch_offset = 0.0
                            walking_active = (
                                not recovery_step_active
                                and not single_support.active
                                and (motion_requested or not engine.is_idle_ready())
                            )
                            recovery_allowed = (
                                recovery_step_active
                                or single_support.active
                                or not arm_dance.running
                            )
                            if recovery is not None and recovery_allowed:
                                decision = recovery.update(
                                    -angle_error_deg(reading.roll_deg, balance.config.target_roll_deg),
                                    -angle_error_deg(reading.pitch_deg, balance.config.target_pitch_deg),
                                    balance_dt,
                                    single_support=single_support.active,
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
                                    single_support.reset()
                                    recovery_step_active = False
                                    recovery_engine.reset()
                                    engine.reset()
                                    standing_hold_active = True
                                    pose = lower_toward_standing(
                                        last_pose,
                                        STANDING,
                                        balance_dt,
                                        args.push_recovery_lower_rate_pwm_s,
                                    )
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
                                or single_support.active
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
                                pose = clamp_pose_rate(
                                    last_pose,
                                    pose,
                                    engine.max_pwm_per_frame,
                                )
                            else:
                                balance.reset()
                            balance_has_valid_imu = True
                        else:
                            sensor_safe_lower = (
                                recovery_step_active
                                or single_support.active
                                or (
                                    recovery is not None
                                    and recovery.state is RecoveryState.SAFE_LOWER
                                )
                            )
                            if sensor_safe_lower:
                                if recovery is not None:
                                    recovery.force_safe_lower("IMU stream lost")
                                recovery_status = "safe-lower: IMU stream lost"
                                single_support.reset()
                                recovery_step_active = False
                                recovery_engine.reset()
                                engine.reset()
                                standing_hold_active = True
                                pose = lower_toward_standing(
                                    last_pose,
                                    STANDING,
                                    balance_dt,
                                    args.push_recovery_lower_rate_pwm_s,
                                )
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
                        now = time.monotonic()
                        max_head_delta = args.head_pan_rate_pwm_s * max(0.0, now - last_head_update_t)
                        head_pwm += max(-max_head_delta, min(max_head_delta, head_target - head_pwm))
                        last_head_update_t = now
                        pose[25] = max(500, min(2500, round(head_pwm)))
    
                    try:
                        backend.send(pose, duration_ms=args.update_ms)
                        last_pose = dict(pose)
                    except Exception as exc:
                        print(f"[main] Backend send exception: {exc}")

                    if fall_active:
                        camera_status = "FALL DETECTED - ARMS FORWARD"
                    elif obstacle_blocked:
                        camera_status = f"OBJECT {obstacle_mm} MM - FORWARD BLOCKED"
                    elif getup.running:
                        camera_status = f"GET-UP: {getup.label.upper()}"
                    elif arm_dance.running:
                        camera_status = "ARM DANCE"
                    elif recovery is not None and recovery.state is not RecoveryState.STABLE:
                        camera_status = f"BALANCE: {recovery_status.upper()}"
                    elif single_support.active:
                        camera_status = f"ONE-FOOT BALANCE: {single_support.support_leg.upper()}"
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
                    gait_state = (
                        recovery_engine.telemetry_snapshot()
                        if recovery_step_active
                        else engine.telemetry_snapshot()
                    )
                    if single_support.active:
                        gait_state["phase"] = "one-foot"
                        gait_state["support_leg"] = single_support.support_leg
                        gait_state["swing_leg"] = (
                            "right" if single_support.support_leg == "left" else "left"
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
                            or single_support.active
                            or arm_dance.running
                            or getup.running
                            or fall_active
                        ),
                        camera_ready=camera_ready,
                        balance_status=recovery_status,
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
        if sensor_hub is not None:
            sensor_hub.close()
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
        history_seconds=args.gait_dashboard_history_s,
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

                    run_terrain(args, dashboard, camera_ready)
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
