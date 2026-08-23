from __future__ import annotations

import time

from .backends import make_backend
from .balance import configured_fall_detector, extend_arms_forward, update_fall_detector
from .config import Config, STANDING
from .gait_dashboard import stationary_gait
from .object_detection import SimpleObjectDetector
from .sensors import RobotSensorHub
from .walking_engine import AdaptiveSquatEngine


def run_pickup(args: Config, dashboard, camera, camera_ready: bool) -> None:
    squat = AdaptiveSquatEngine(
        dt=args.update_ms / 1000.0,
        min_depth_mm=args.squat_min_depth_mm,
        max_depth_mm=args.squat_max_depth_mm,
        depth_rate_mm_s=args.squat_depth_rate_mm_s,
        max_pwm_per_frame=args.squat_max_pwm_per_frame,
    )
    detector = None
    if camera_ready:
        detector = SimpleObjectDetector(
            min_area_ratio=args.pickup_object_min_area_ratio,
            max_area_ratio=args.pickup_object_max_area_ratio,
            detect_every_frames=args.pickup_detect_every_frames,
        )
        camera.set_detector(detector)
    backend = make_backend(
        mode=args.backend,
        port=args.port,
        baudrate=args.baudrate,
        csv_path=args.csv,
    )
    previous_stop = False
    sensor_hub = None
    if args.fall_detection_enabled:
        sensor_hub = RobotSensorHub(
            port=args.sensor_port,
            baudrate=args.sensor_baudrate,
            timeout_s=args.sensor_timeout_s,
            use_imu=True,
            use_foot_fsr=False,
            use_depth=False,
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
            print(f"[pickup] IMU unavailable: {exc}. Pickup remains available without fall protection.")
    imu_reference = None
    fall_detector = None
    last_fall_at = time.monotonic()
    last_pose = dict(STANDING)

    try:
        with backend:
            backend.send(STANDING, duration_ms=900, force=True)
            time.sleep(0.9)
            if sensor_hub is not None:
                print("[pickup] Keep the robot upright and still while fall protection calibrates.")
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
                    print("[pickup] IMU reference failed. Pickup remains available without fall protection.")
            try:
                squat.reset(STANDING)
                dashboard.set_runtime("pickup", "Pickup positioning ready")
                while True:
                    loop_started = time.monotonic()
                    control = dashboard.control_state()
                    if not control.armed or control.mode != "pickup":
                        break

                    stop_pressed = control.stop
                    if stop_pressed and not previous_stop:
                        squat.reset(STANDING)
                        if fall_detector is None or not fall_detector.triggered:
                            backend.send(STANDING, duration_ms=args.stop_ms, force=True)
                            last_pose = dict(STANDING)
                            print("[pickup] Stopped at STANDING.")
                    previous_stop = stop_pressed

                    snapshot = sensor_hub.read() if sensor_hub is not None else None
                    ratio = 1.0 if control.squat and not stop_pressed else 0.0
                    pose = squat.update(ratio)
                    depth_ratio = squat.depth_mm / max(1.0, args.squat_max_depth_mm)
                    object_frame = camera.object_frame() if detector is not None else None
                    detected = object_frame.primary_object if object_frame is not None else None
                    if detected is not None and time.monotonic() - object_frame.captured_at > 0.8:
                        detected = None
                    object_name = f"{detected.color} {detected.label}" if detected is not None else ""
                    object_status = (
                        f"{object_name} {detected.confidence:.2f}" if detected is not None else ""
                    )
                    if ratio > 0.0 or not squat.is_idle():
                        status = f"PICK-UP POSITION {round(depth_ratio * 100)}%"
                        if object_status:
                            status += f" | {object_status}"
                    elif object_status:
                        status = f"DETECTED {object_status}"
                    else:
                        status = "SHOW CAN, BALL OR RUBIK CUBE" if camera_ready else "PICK-UP CAMERA OFFLINE"

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
                            squat.reset(last_pose)
                            pose = extend_arms_forward(last_pose, args.fall_arm_forward_pwm)
                            status = f"FALL: {fall_detector.reason}"
                            if not was_triggered:
                                print(f"[pickup] {status}. Arms moving forward.")
                        elif was_triggered:
                            squat.reset(STANDING)
                            pose = dict(STANDING)
                            status = "UPRIGHT - ARMS RETURNED"
                            print("[pickup] IMU upright again. Arms returned to STANDING.")
                    last_fall_at = now
                    backend.send(pose, duration_ms=args.update_ms)
                    last_pose = dict(pose)
                    dashboard.publish(
                        pose=pose,
                        gait=stationary_gait("squat" if not squat.is_idle() else "idle"),
                        sensor_snapshot=snapshot,
                        status=status,
                        active=fall_active or detected is not None or not squat.is_idle(),
                        camera_ready=camera_ready,
                        balance_status=(
                            "FALL ACTIVE"
                            if fall_active
                            else "FALL READY" if fall_detector is not None else object_name or "FALL IMU WAIT"
                        ),
                    )
                    dashboard.set_runtime("pickup", status)
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
                    print(f"[pickup] Failed to return to STANDING: {exc}")
    finally:
        if sensor_hub is not None:
            sensor_hub.close()
        camera.set_detector(None)
        dashboard.set_runtime("idle", "Pickup positioning stopped")
        print("[pickup] Pick Up Positioning exited.")
