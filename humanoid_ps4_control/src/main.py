from __future__ import annotations

import time

from .backends import make_backend
from .config import Config


def run_keyboard(args: Config) -> None:
    """
    Real-time walking mode.

    Keyboard:
      W/S        : walk forward/backward
      A/D        : turn left/right
      J/K        : side walk left/right
      L/M        : toggle standing arm dance
      X          : toggle single-leg support test
      G          : run get-up sequence
      B          : run back get-up sequence
      C          : stop and hold standing
      E/T        : reset walking engine
      Y/N        : follow detected person / ignore or stop following
      R          : hold adaptive squat toward a centered detected object
      O/Escape   : return to menu
      Q          : quit
    """
    from pathlib import Path

    from .keyboard_input import KeyboardReader, LiveCameraPreview
    from .person_follow import (
        PersonDetector,
        PersonFollowController,
        PersonFrame,
        SquatTargetController,
    )
    from .walking_engine import (
        AdaptiveSquatEngine,
        DynamicWalkingEngine,
        SingleSupportTestEngine,
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
        lower_toward_standing,
    )
    from .sensors import DepthObstacleGuard, RobotSensorHub

    backend = make_backend(mode=args.backend, port=args.port, baudrate=args.baudrate, csv_path=args.csv)

    poll_hz = int(1000 / args.update_ms)
    reader = KeyboardReader(poll_rate_hz=poll_hz)
    package_root = Path(__file__).resolve().parent.parent
    prototxt_path = package_root / args.person_detect_prototxt
    model_path = package_root / args.person_detect_model
    detector = None
    try:
        detector = PersonDetector(
            prototxt_path=str(prototxt_path.resolve()),
            model_path=str(model_path.resolve()),
            confidence=args.person_detect_confidence,
            detect_every_frames=args.person_detect_every_frames,
        )
        print("[main] MobileNet-SSD person/object detector ready.")
    except Exception as exc:
        print(f"[main] Person detection unavailable: {exc}")

    camera_preview = LiveCameraPreview(
        width=args.vision_camera_width,
        height=args.vision_camera_height,
        fps=args.vision_fps,
        detector=detector,
        stable_frames=args.person_detect_stable_frames,
    )
    person_follow = PersonFollowController(
        turn_deadband=args.person_follow_turn_deadband,
        stop_height_ratio=args.person_follow_stop_height_ratio,
        lost_timeout_s=args.person_follow_lost_timeout_s,
        forward_speed=args.person_follow_speed,
        turn_speed=args.person_follow_turn_speed,
    )
    obstacle_guard = DepthObstacleGuard(
        stop_distance_mm=args.tof_obstacle_stop_mm,
        clear_margin_mm=args.tof_obstacle_clear_margin_mm,
        stable_frames=args.tof_obstacle_stable_frames,
    )
    squat_target = SquatTargetController(
        min_distance_mm=args.squat_min_object_distance_mm,
        max_distance_mm=args.squat_max_object_distance_mm,
        camera_center_tolerance=args.squat_camera_center_tolerance,
        min_depth_ratio=args.squat_min_depth_ratio,
        target_timeout_s=args.squat_target_timeout_s,
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
    squat_engine = AdaptiveSquatEngine(
        dt=args.update_ms / 1000.0,
        min_depth_mm=args.squat_min_depth_mm,
        max_depth_mm=args.squat_max_depth_mm,
        depth_rate_mm_s=args.squat_depth_rate_mm_s,
        max_pwm_per_frame=args.squat_max_pwm_per_frame,
    )
    recovery_engine = DynamicWalkingEngine(
        dt=args.update_ms / 1000.0,
        t_step=args.push_recovery_step_time_s,
        t_dbl=args.t_dbl,
        max_step_len=args.max_step_len,
        max_side_step_len=args.max_side_step_len,
        step_height=args.push_recovery_step_height_mm,
        command_rate_limit=1000.0,
        trajectory_smoothing=args.trajectory_smoothing,
    )
    recovery_engine.stop_extra_steps = 0
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
    prev_follow_pressed = False
    prev_ignore_person_pressed = False
    prev_menu_pressed = False
    next_single_support_leg = "right"
    last_pose = dict(STANDING)
    standing_hold_active = True

    balance = None
    recovery = None
    recovery_step_active = False
    recovery_status = "STABLE"
    previous_recovery_status = recovery_status
    sensor_hub = None
    sensor_snapshot = None
    foot_contact_frames = 0
    last_balance_t = time.monotonic()
    balance_has_valid_imu = False
    obstacle_blocked = False
    obstacle_mm = None
    previous_obstacle_blocked = False
    squat_requested = False
    squat_ratio = 0.0
    squat_status = "OFF"
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
            print("[main] Sensor connection started in background; keyboard control is independent.")
        except Exception as exc:
            sensor_hub = None
            print(f"[main] Sensors disabled: {exc}. Keyboard control remains available.")

    if args.imu_balance and (sensor_hub is None or not args.sensor_use_imu):
        print("[main] IMU balance requested but IMU sensor feedback is disabled.")

    print(
        "\n[Keyboard Mode - Real-time ZMP] W/S walk, A/D turn (head leads), J/K side, "
        "X single support, L/M dance, G get-up, "
        "B get-up back, R hold squat, C stop, E/T reset, O/Esc menu, Q quit\n"
    )

    try:
        with backend:
            initial_imu = sensor_hub.read().imu if sensor_hub is not None else None
            sensor_start_deadline = time.monotonic() + 0.5
            while (
                sensor_hub is not None
                and initial_imu is None
                and time.monotonic() < sensor_start_deadline
            ):
                time.sleep(0.02)
                initial_imu = sensor_hub.read().imu
            if args.imu_balance and initial_imu is not None:
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
                            )
                        )
                    print(
                        f"[main] IMU balance enabled: reference roll={target_roll:.2f}, "
                        f"pitch={target_pitch:.2f}, limit={args.balance_limit_deg:.1f} deg."
                    )
            elif args.imu_balance:
                print("[main] IMU not ready at startup. Manual keyboard control remains enabled without balance.")
            if not reader.init():
                raise RuntimeError("pygame keyboard control is unavailable")
            camera_ready = camera_preview.start()
            camera_preview.render(
                "STANDING" if camera_ready else "CAMERA OFF - KEYBOARD READY",
                follow_enabled=False,
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

                    both_feet_contact = False
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
                        feet = sensor_snapshot.feet
                        both_feet_contact = (
                            feet is not None
                            and feet.left_force >= args.foot_fsr_contact_threshold
                            and feet.right_force >= args.foot_fsr_contact_threshold
                        )
                        foot_contact_frames = foot_contact_frames + 1 if both_feet_contact else 0
    
                    vy = state.forward * args.walk_speed
                    turn_cmd = state.turn * args.turn_speed
                    side_cmd = state.side * args.side_speed
                    motion_requested = vy != 0.0 or turn_cmd != 0.0 or side_cmd != 0.0

                    if motion_requested and person_follow.enabled:
                        person_follow.disable()
                        print("[main] Manual movement canceled person follow.")

                    ignore_pressed = state.ignore_person
                    if ignore_pressed and not prev_ignore_person_pressed:
                        if person_follow.enabled:
                            person_follow.disable()
                            engine.reset()
                            standing_hold_active = True
                            print("[main] Person follow stopped.")
                        else:
                            camera_preview.ignore_person()
                            print("[main] Detected person ignored.")
                    prev_ignore_person_pressed = ignore_pressed

                    follow_pressed = state.follow
                    if follow_pressed and not prev_follow_pressed:
                        can_follow = (
                            camera_preview.person_ready()
                            and not getup.running
                            and not arm_dance.running
                            and not single_support.running
                            and squat_engine.is_idle()
                            and not motion_requested
                        )
                        if can_follow:
                            person_follow.enable()
                            engine.reset()
                            standing_hold_active = False
                            print("[main] Person follow enabled. Press N, C, or move manually to stop.")
                        else:
                            print("[main] Follow rejected: one stable person and STANDING are required.")
                    prev_follow_pressed = follow_pressed

                    follow_status = "OFF"
                    if person_follow.enabled:
                        person_frame = camera_preview.person_frame() or PersonFrame()
                        vy, turn_cmd, follow_status = person_follow.command(person_frame)
                        side_cmd = 0.0
                        motion_requested = vy != 0.0 or turn_cmd != 0.0
                        if follow_status in ("TARGET LOST", "MULTIPLE PEOPLE"):
                            person_follow.disable()
                            engine.reset()
                            standing_hold_active = True
                            vy = 0.0
                            turn_cmd = 0.0
                            motion_requested = False
                            print(f"[main] Person follow stopped: {follow_status.lower()}.")

                    depth_missing = bool(
                        args.sensor_feedback
                        and args.sensor_use_depth
                        and (sensor_snapshot is None or sensor_snapshot.depth is None)
                    )
                    if vy > 0.0 and (obstacle_blocked or (person_follow.enabled and depth_missing)):
                        vy = 0.0
                        motion_requested = turn_cmd != 0.0 or side_cmd != 0.0
                        follow_status = (
                            f"OBJECT {obstacle_mm} MM"
                            if obstacle_blocked and obstacle_mm is not None
                            else "TOF WAIT"
                        )

                    head_turn_cmd = turn_cmd
                    turn_sign = 1 if turn_cmd > 0.0 else -1 if turn_cmd < 0.0 else 0
                    now = time.monotonic()
                    if not person_follow.enabled and turn_sign != 0 and turn_sign != head_turn_sign:
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
                        if not prev_stop_pressed:
                            print("[main] C pressed. Hard stop to STANDING.")
                        prev_stop_pressed = True
                        person_follow.disable()
                        engine.reset()
                        arm_dance.reset()
                        getup.reset()
                        single_support.stop()
                        recovery_engine.reset()
                        squat_engine.reset()
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
                        camera_preview.render("STOP / STANDING", follow_enabled=False)
                        continue
                    prev_stop_pressed = False
    
                    getup_pressed = state.getup
                    if getup_pressed and not prev_getup_pressed and squat_engine.is_idle():
                        person_follow.disable()
                        engine.reset()
                        arm_dance.reset()
                        single_support.stop()
                        standing_hold_active = False
                        label = getup.start(last_pose, mode=args.getup_mode)
                        print(f"[main] G pressed. Running {args.getup_mode} get-up sequence from step {label}.")
                    prev_getup_pressed = getup_pressed
    
                    getup_back_pressed = state.getup_back
                    if getup_back_pressed and not prev_getup_back_pressed and squat_engine.is_idle():
                        person_follow.disable()
                        engine.reset()
                        arm_dance.reset()
                        single_support.stop()
                        standing_hold_active = False
                        label = getup.start(last_pose, mode="back")
                        print(f"[main] B pressed. Running back get-up sequence from step {label}.")
                    prev_getup_back_pressed = getup_back_pressed
    
                    dance_pressed = state.dance
                    if (
                        dance_pressed
                        and not prev_dance_pressed
                        and not getup.running
                        and squat_engine.is_idle()
                    ):
                        person_follow.disable()
                        enabled = arm_dance.toggle()
                        engine.reset()
                        single_support.stop()
                        standing_hold_active = not enabled
                        print("[main] L/M arm dance ON." if enabled else "[main] L/M arm dance OFF - returning to STANDING.")
                    prev_dance_pressed = dance_pressed

                    single_support_pressed = state.single_support
                    if (
                        single_support_pressed
                        and not prev_single_support_pressed
                        and not getup.running
                        and squat_engine.is_idle()
                    ):
                        person_follow.disable()
                        engine.reset()
                        arm_dance.reset()
                        if single_support.running:
                            single_support.stop()
                            standing_hold_active = True
                            print("[main] X single-support OFF - returning to STANDING.")
                        elif foot_contact_frames < args.foot_fsr_stable_frames:
                            print("[main] X single-support requires stable contact on both foot FSRs.")
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
                        getup.reset()
                        single_support.stop()
                        recovery_engine.reset()
                        squat_engine.reset()
                        recovery_step_active = False
                        if recovery is not None:
                            recovery.reset()
                            recovery_status = "STABLE"
                        standing_hold_active = True
                        person_follow.disable()
                        vy = 0.0
                        turn_cmd = 0.0
                        side_cmd = 0.0
                        motion_requested = False

                    squat_requested = False
                    squat_ratio = 0.0
                    squat_status = "OFF"
                    if state.squat:
                        person_follow.disable()
                        vy = 0.0
                        turn_cmd = 0.0
                        side_cmd = 0.0
                        motion_requested = False
                        busy = any((getup.running, arm_dance.running, single_support.running))
                        if busy:
                            squat_status = "BUSY"
                        elif foot_contact_frames < args.foot_fsr_stable_frames:
                            squat_status = "FSR WAIT"
                        elif (
                            balance is None
                            or sensor_snapshot is None
                            or sensor_snapshot.imu is None
                            or not sensor_snapshot.imu.balance_ready(
                                args.imu_min_gyro_cal,
                                args.imu_min_accel_cal,
                            )
                        ):
                            squat_status = "IMU WAIT"
                        elif not standing_hold_active and squat_engine.is_idle():
                            squat_status = "WAIT STANDING"
                        else:
                            target_frame = camera_preview.person_frame() or PersonFrame()
                            depth = sensor_snapshot.depth if sensor_snapshot is not None else None
                            squat_ratio, squat_status = squat_target.command(depth, target_frame)
                            if squat_ratio > 0.0:
                                squat_requested = True
                                engine.reset()
                                standing_hold_active = False
                        if not squat_requested and not squat_engine.is_idle():
                            squat_requested = True
                            squat_status = f"{squat_status} - RETURNING"
                    elif not squat_engine.is_idle():
                        squat_requested = True
                        squat_status = "RETURNING"

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
                        feet = sensor_snapshot.feet if sensor_snapshot is not None else None
                        support_force = (
                            feet.left_force if single_support.support_leg == "left" and feet is not None
                            else feet.right_force if feet is not None
                            else 0.0
                        )
                        if support_force < args.foot_fsr_contact_threshold:
                            single_support.stop()
                            standing_hold_active = True
                            if recovery is not None:
                                recovery.force_safe_lower("support FSR lost")
                                recovery_status = "safe-lower: support FSR lost"
                            pose = lower_toward_standing(
                                last_pose,
                                STANDING,
                                args.update_ms / 1000.0,
                                args.push_recovery_lower_rate_pwm_s,
                            )
                        else:
                            pose = single_support.update()
                    elif squat_requested:
                        pose = squat_engine.update(squat_ratio)
                        if squat_engine.is_idle():
                            standing_hold_active = True
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
                            if recovery_step_active:
                                support_leg = recovery_engine.support_leg
                            elif single_support.running:
                                support_leg = single_support.support_leg
                            elif squat_requested:
                                support_leg = "double"
                            else:
                                support_leg = engine.support_leg
                            recovery_allowed = (
                                recovery_step_active
                                or single_support.running
                                or (
                                    standing_hold_active
                                    and not motion_requested
                                    and not arm_dance.running
                                )
                            )
                            if recovery is not None and recovery_allowed:
                                feet = sensor_snapshot.feet if sensor_snapshot is not None else None
                                left_contact = (
                                    feet is not None
                                    and feet.left_force >= args.foot_fsr_contact_threshold
                                )
                                right_contact = (
                                    feet is not None
                                    and feet.right_force >= args.foot_fsr_contact_threshold
                                )
                                decision = recovery.update(
                                    -angle_error_deg(reading.roll_deg, balance.config.target_roll_deg),
                                    -angle_error_deg(reading.pitch_deg, balance.config.target_pitch_deg),
                                    balance_dt,
                                    left_contact,
                                    right_contact,
                                    single_support=single_support.running,
                                    support_leg=support_leg,
                                    now=now,
                                )
                                recovery_status = f"{decision.state.value}: {decision.reason}"
                                if decision.start_step:
                                    recovery_engine.reset()
                                    recovery_engine.stop_extra_steps = 0
                                    recovery_step_active = True
                                    engine.reset()
                                    person_follow.disable()
                                    standing_hold_active = False
                                    print("[main] Push recovery: starting near-in-place stomp.")
                                if decision.safe_lower:
                                    single_support.stop()
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
                                        decision.forward_cmd if decision.start_step else 0.0,
                                        side_cmd=decision.side_cmd if decision.start_step else 0.0,
                                    )
                                    support_leg = recovery_engine.support_leg
                                    swing_leg = recovery_engine.last_swing_leg
                                    if (
                                        recovery_engine.last_phase_mode == "land"
                                        and recovery_engine.last_landing_progress >= 0.85
                                        and swing_leg in ("left", "right")
                                        and not (left_contact if swing_leg == "left" else right_contact)
                                    ):
                                        recovery.force_safe_lower("swing FSR did not contact")
                                        recovery_status = "safe-lower: swing FSR did not contact"
                                        recovery_step_active = False
                                        pose = lower_toward_standing(
                                            last_pose,
                                            STANDING,
                                            balance_dt,
                                            args.push_recovery_lower_rate_pwm_s,
                                        )
                                    elif recovery_engine.is_idle_ready():
                                        completed = recovery.complete_step(now)
                                        recovery_status = f"{completed.state.value}: {completed.reason}"
                                        recovery_step_active = False
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

                    if recovery_status != previous_recovery_status:
                        print(f"[main] Push recovery: {recovery_status}.")
                        previous_recovery_status = recovery_status

                    if not pose_from_getup and not arm_dance.running:
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

                    if squat_requested:
                        camera_status = f"SQUAT {round(squat_ratio * 100)}%: {squat_status}"
                    elif obstacle_blocked:
                        camera_status = f"OBJECT {obstacle_mm} MM - FORWARD BLOCKED"
                    elif person_follow.enabled:
                        camera_status = follow_status
                    elif getup.running:
                        camera_status = f"GET-UP: {getup.label.upper()}"
                    elif arm_dance.running:
                        camera_status = "ARM DANCE"
                    elif recovery is not None and recovery.state is not RecoveryState.STABLE:
                        camera_status = f"BALANCE: {recovery_status.upper()}"
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
                    camera_preview.render(camera_status, follow_enabled=person_follow.enabled)
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
        elif choice == "terrain":
            try:
                from .terrain_main import run_terrain

                run_terrain(args)
            except Exception as exc:
                print(f"[main] Terrain Auto unavailable: {exc}")
                time.sleep(1.5)

if __name__ == "__main__":
    main()
