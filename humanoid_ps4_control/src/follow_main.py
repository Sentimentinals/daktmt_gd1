from __future__ import annotations

import time
from pathlib import Path

from .backends import make_backend
from .balance import configured_fall_detector, extend_arms_forward, update_fall_detector
from .config import Config, STANDING
from .person_follow import PersonDetector, PersonFollowController, PersonFrame
from .sensors import DepthObstacleGuard, RobotSensorHub
from .walking_engine import DynamicWalkingEngine


def run_follow(args: Config, dashboard, camera, camera_ready: bool) -> None:
    dashboard.set_runtime("follow", "Starting Person Follow")
    package_root = Path(__file__).resolve().parent.parent
    detector = PersonDetector(
        prototxt_path=str((package_root / args.person_detect_prototxt).resolve()),
        model_path=str((package_root / args.person_detect_model).resolve()),
        confidence=args.person_detect_confidence,
        detect_every_frames=args.person_detect_every_frames,
    )
    camera.set_detector(detector, stable_frames=args.person_detect_stable_frames)
    follow = PersonFollowController(
        turn_deadband=args.person_follow_turn_deadband,
        stop_height_ratio=args.person_follow_stop_height_ratio,
        lost_timeout_s=args.person_follow_lost_timeout_s,
        forward_speed=args.person_follow_speed,
        turn_speed=args.person_follow_turn_speed,
        target_distance_mm=args.person_follow_target_distance_mm,
        distance_deadband_mm=args.person_follow_distance_deadband_mm,
        slow_range_mm=args.person_follow_slow_range_mm,
        tof_filter_alpha=args.person_follow_tof_filter_alpha,
    )
    engine = DynamicWalkingEngine(
        dt=args.update_ms / 1000.0,
        t_step=args.t_step,
        t_dbl=args.t_dbl,
        max_step_len=args.walk_step_length_mm,
        max_turn_step_len=args.max_turn_step_len,
        step_height=args.walk_step_height_mm,
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
    )
    obstacle_guard = DepthObstacleGuard(
        stop_distance_mm=args.tof_obstacle_stop_mm,
        clear_margin_mm=args.tof_obstacle_clear_margin_mm,
        stable_frames=args.tof_obstacle_stable_frames,
    )
    sensor_hub = None
    need_imu = args.fall_detection_enabled
    need_depth = args.sensor_feedback and args.sensor_use_depth
    if need_imu or need_depth:
        sensor_hub = RobotSensorHub(
            port=args.sensor_port,
            baudrate=args.sensor_baudrate,
            timeout_s=args.sensor_timeout_s,
            depth_timeout_s=args.sensor_depth_timeout_s,
            use_imu=need_imu,
            use_foot_fsr=False,
            use_depth=need_depth,
            imu_roll_sign=args.imu_roll_sign,
            imu_pitch_sign=args.imu_pitch_sign,
            imu_yaw_sign=args.imu_yaw_sign,
            imu_vertical_mount=args.imu_vertical_mount,
            imu_board_face_sign=args.imu_board_face_sign,
        )
        try:
            sensor_hub.open(wait_for_connection=False)
        except Exception as exc:
            sensor_hub = None
            print(f"[follow] Sensors unavailable: {exc}. Follow remains available without ToF/fall protection.")

    backend = make_backend(
        mode=args.backend,
        port=args.port,
        baudrate=args.baudrate,
        csv_path=args.csv,
    )
    previous_follow = False
    previous_ignore = False
    previous_stop = False
    last_fall_at = time.monotonic()
    last_pose = dict(STANDING)
    imu_reference = None
    fall_detector = None

    try:
        with backend:
            backend.send(STANDING, duration_ms=900, force=True)
            time.sleep(0.9)
            if need_imu and sensor_hub is not None:
                print("[follow] Keep the robot upright and still while fall protection calibrates.")
                imu_reference = sensor_hub.capture_imu_reference(
                    sample_seconds=args.imu_reference_seconds,
                    timeout_s=args.imu_reference_timeout_s,
                    min_gyro_cal=args.imu_min_gyro_cal,
                    min_accel_cal=args.imu_min_accel_cal,
                    max_rms_deg=args.imu_reference_max_rms_deg,
                )
                if imu_reference is not None:
                    fall_detector = configured_fall_detector(args)
                else:
                    print("[follow] IMU reference failed. Follow remains available without fall protection.")
            try:
                dashboard.set_runtime("follow", "Follow ready")
                while True:
                    loop_started = time.monotonic()
                    control = dashboard.control_state()
                    if not control.armed or control.mode != "follow":
                        break

                    follow_pressed = control.follow
                    if follow_pressed and not previous_follow:
                        if camera.person_ready():
                            follow.enable()
                            engine.reset()
                            print("[follow] Person follow enabled.")
                        else:
                            print("[follow] Follow rejected: one stable person is required.")
                    previous_follow = follow_pressed

                    ignore_pressed = control.ignore_person
                    if ignore_pressed and not previous_ignore:
                        if follow.enabled:
                            follow.disable()
                            engine.reset()
                            print("[follow] Person follow stopped.")
                        else:
                            camera.ignore_person()
                            print("[follow] Detected person ignored.")
                    previous_ignore = ignore_pressed

                    stop_pressed = control.stop
                    if stop_pressed and not previous_stop:
                        follow.disable()
                        engine.reset()
                        if fall_detector is None or not fall_detector.triggered:
                            backend.send(STANDING, duration_ms=args.stop_ms, force=True)
                            last_pose = dict(STANDING)
                            print("[follow] Stopped at STANDING.")
                    previous_stop = stop_pressed

                    snapshot = sensor_hub.read() if sensor_hub is not None else None
                    depth = snapshot.depth if snapshot is not None else None
                    obstacle_blocked, obstacle_mm = obstacle_guard.update(depth)

                    forward = 0.0
                    turn = 0.0
                    status = "FOLLOW READY"
                    if follow.enabled:
                        frame = camera.person_frame() or PersonFrame()
                        distance_mm = depth.tracking_distance_mm if depth is not None else None
                        distance_sample_id = depth.sensor_time_ms if depth is not None else None
                        forward, turn, status = follow.command(
                            frame,
                            distance_mm=distance_mm,
                            distance_sample_id=distance_sample_id,
                        )
                        if status in ("TARGET LOST", "MULTIPLE PEOPLE"):
                            follow.disable()
                            engine.reset()
                            forward = 0.0
                            turn = 0.0
                            print(f"[follow] Stopped: {status.lower()}.")
                        elif forward > 0.0 and obstacle_blocked:
                            forward = 0.0
                            status = f"OBJECT {obstacle_mm} MM" if obstacle_mm is not None else "OBJECT"

                    if follow.enabled or not engine.is_idle_ready():
                        pose = engine.update(forward, turn_cmd=turn)
                    else:
                        pose = dict(STANDING)

                    fall_active = False
                    was_triggered = fall_detector.triggered if fall_detector is not None else False
                    now = time.monotonic()
                    if fall_detector is not None:
                        imu = snapshot.imu if snapshot is not None else None
                        fall_active = update_fall_detector(
                            fall_detector,
                            imu,
                            imu_reference,
                            now - last_fall_at,
                            args,
                        )
                        if fall_active:
                            follow.disable()
                            engine.reset()
                            pose = extend_arms_forward(last_pose, args.fall_arm_forward_pwm)
                            status = f"FALL: {fall_detector.reason}"
                            if not was_triggered:
                                print(f"[follow] {status}. Arms moving forward.")
                        elif was_triggered:
                            pose = dict(STANDING)
                            status = "UPRIGHT - ARMS RETURNED"
                            print("[follow] IMU upright again. Arms returned to STANDING.")
                    last_fall_at = now
                    pose[25] = STANDING[25]
                    backend.send(pose, duration_ms=args.update_ms)
                    last_pose = dict(pose)
                    dashboard.publish(
                        pose=pose,
                        gait=engine.telemetry_snapshot(),
                        sensor_snapshot=snapshot,
                        status=status,
                        active=fall_active or follow.enabled or not engine.is_idle_ready(),
                        camera_ready=camera_ready,
                        balance_status=(
                            "FALL ACTIVE"
                            if fall_active
                            else "FALL READY" if fall_detector is not None else "FALL IMU WAIT"
                        ),
                    )
                    dashboard.set_runtime("follow", status)
                    remaining = args.update_ms / 1000.0 - (time.monotonic() - loop_started)
                    if remaining > 0.0:
                        time.sleep(remaining)
            finally:
                try:
                    exit_pose = (
                        last_pose
                        if fall_detector is not None and fall_detector.triggered
                        else STANDING
                    )
                    backend.send(exit_pose, duration_ms=args.stop_ms, force=True)
                    time.sleep(args.stop_ms / 1000.0)
                except Exception as exc:
                    print(f"[follow] Failed to return to STANDING: {exc}")
    finally:
        if sensor_hub is not None:
            sensor_hub.close()
        camera.set_detector(None)
        dashboard.set_runtime("idle", "Person follow stopped")
        print("[follow] Person Follow exited.")
