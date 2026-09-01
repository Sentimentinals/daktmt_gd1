from __future__ import annotations

import math
import time
from pathlib import Path

from .backends import make_backend
from .balance import configured_fall_detector, extend_arms_forward, update_fall_detector
from .config import Config, STANDING
from .gait_dashboard import stationary_gait
from .object_detection import PickupObjectDetector
from .sensors import RobotSensorHub
from .walking_engine import AdaptiveSquatEngine, DynamicWalkingEngine


def _reach_pose(base_pose: dict[int, int], reach_pwm: int) -> dict[int, int]:
    pose = dict(base_pose)
    pose[11] = STANDING[11] - reach_pwm
    pose[22] = STANDING[22] + reach_pwm
    return pose


def run_pickup(args: Config, dashboard, camera, camera_ready: bool) -> None:
    dashboard.set_runtime("pickup", "Starting Pick Up")
    squat = AdaptiveSquatEngine(
        min_depth_mm=args.squat_min_depth_mm,
        max_depth_mm=args.squat_max_depth_mm,
    )
    walking = DynamicWalkingEngine(
        dt=args.update_ms / 1000.0,
        t_step=args.t_step,
        t_dbl=args.t_dbl,
        max_step_len=args.walk_step_length_mm,
        max_turn_step_len=args.max_turn_step_len,
        step_height=args.walk_step_height_mm,
        zmp_support_ratio=args.zmp_support_ratio,
        ankle_roll_gain=args.ankle_roll_gain,
        landing_gap_mm=args.walk_step_length_mm,
        lift_start_phase=args.walk_lift_start_phase,
        swing_advance_end_phase=args.walk_swing_advance_end_phase,
        lift_end_phase=args.walk_lift_end_phase,
        landing_roll_release_start=args.walk_landing_roll_release_start,
        arm_swing_pwm=0,
    )

    detector = None
    if camera_ready:
        model_path = Path(__file__).resolve().parent.parent / args.pickup_object_model
        try:
            detector = PickupObjectDetector(
                model_path=str(model_path),
                confidence=args.pickup_object_confidence,
                iou_threshold=args.pickup_object_iou_threshold,
                input_size=args.pickup_object_input_size,
                detect_every_frames=args.pickup_detect_every_frames,
            )
            camera.set_detector(detector)
        except Exception as exc:
            print(f"[pickup] Object detector unavailable: {exc}")

    need_imu = args.fall_detection_enabled
    need_depth = args.sensor_feedback and args.sensor_use_depth
    sensor_hub = None
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
            print(f"[pickup] Sensors unavailable: {exc}. Camera positioning remains available.")

    backend = make_backend(
        mode=args.backend,
        port=args.port,
        baudrate=args.baudrate,
        csv_path=args.csv,
    )
    phase = "idle"
    phase_started = time.monotonic()
    target_key = None
    target_frames = 0
    target_squat_mm = args.squat_min_depth_mm
    last_detection_at = None
    previous_stop = False
    imu_reference = None
    fall_detector = None
    last_fall_at = time.monotonic()
    last_pose = dict(STANDING)

    def enter(next_phase: str) -> None:
        nonlocal phase, phase_started
        phase = next_phase
        phase_started = time.monotonic()

    def cancel() -> None:
        nonlocal target_key, target_frames
        enter("idle")
        target_key = None
        target_frames = 0
        walking.reset()
        squat.reset()

    try:
        with backend:
            backend.send(STANDING, duration_ms=900, force=True)
            time.sleep(0.9)
            if sensor_hub is not None and need_imu:
                print("[pickup] Keep the robot upright while fall protection calibrates.")
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
                    print("[pickup] IMU reference failed. Pickup remains available.")

            dashboard.set_runtime("pickup", "Show one object, then press R")
            while True:
                loop_started = time.monotonic()
                control = dashboard.control_state()
                if not control.armed or control.mode != "pickup":
                    break

                stop_pressed = control.stop
                if stop_pressed and not previous_stop:
                    cancel()
                    backend.send(STANDING, duration_ms=args.stop_ms, force=True)
                    last_pose = dict(STANDING)
                previous_stop = stop_pressed

                if control.pickup_toggle:
                    if phase == "idle":
                        enter("acquire")
                    elif phase == "ready":
                        enter("lift")
                    else:
                        cancel()

                snapshot = sensor_hub.read() if sensor_hub is not None else None
                object_frame = camera.object_frame() if detector is not None else None
                detected = object_frame.primary_object if object_frame is not None else None
                now = time.monotonic()
                if detected is not None and now - object_frame.captured_at > args.pickup_target_timeout_s:
                    detected = None

                if detected is not None and object_frame.captured_at != last_detection_at:
                    key = (detected.label, detected.color)
                    target_frames = target_frames + 1 if key == target_key else 1
                    target_key = key
                    last_detection_at = object_frame.captured_at
                elif detected is None:
                    target_frames = 0
                    target_key = None

                object_name = ""
                if detected is not None:
                    object_name = " ".join(part for part in (detected.color, detected.label) if part)
                status = "SHOW CAN, BALL OR RUBIK CUBE"
                pose = dict(STANDING)
                gait = stationary_gait()

                if detector is None:
                    status = "PICKUP DETECTOR OFFLINE"
                elif phase == "idle":
                    status = f"DETECTED {object_name} - PRESS R" if detected is not None else status
                elif phase == "acquire":
                    status = f"LOCKING {object_name}" if detected is not None else "SEARCHING OBJECT"
                    if target_frames >= args.pickup_target_stable_frames:
                        enter("align")
                elif phase in {"align", "approach"}:
                    if detected is None:
                        enter("acquire")
                        walking.reset()
                        pose = dict(STANDING)
                        status = "TARGET LOST"
                    else:
                        x1, y1, x2, y2 = detected.box
                        center_x = (x1 + x2) / (2.0 * args.vision_camera_width)
                        center_y = (y1 + y2) / (2.0 * args.vision_camera_height)
                        horizontal_error = 0.5 - center_x
                        if phase == "align" and abs(horizontal_error) > args.pickup_align_deadband:
                            turn = math.copysign(args.pickup_turn_speed, horizontal_error)
                            pose = walking.update(0.0, turn_cmd=turn)
                            gait = walking.telemetry_snapshot()
                            status = f"ALIGNING {object_name}"
                        else:
                            if phase == "align":
                                enter("approach")
                            distance = (
                                snapshot.depth.tracking_distance_mm
                                if snapshot is not None and snapshot.depth is not None
                                else None
                            )
                            if abs(horizontal_error) > args.pickup_align_deadband * 1.5:
                                enter("align")
                                pose = walking.update(0.0)
                                gait = walking.telemetry_snapshot()
                                status = f"REALIGNING {object_name}"
                            else:
                                camera_ready_distance = detected.area_ratio >= args.pickup_target_area_ratio
                                tof_ready = (
                                    distance is not None
                                    and distance <= args.pickup_target_distance_mm + args.pickup_distance_deadband_mm
                                )
                                if tof_ready or (distance is None and camera_ready_distance):
                                    pose = walking.update(0.0)
                                    gait = walking.telemetry_snapshot()
                                    status = f"POSITIONED {object_name}"
                                    if walking.is_idle_ready():
                                        y_ratio = max(0.0, min(1.0, center_y))
                                        target_squat_mm = args.squat_min_depth_mm + y_ratio * (
                                            args.squat_max_depth_mm - args.squat_min_depth_mm
                                        )
                                        enter("squat")
                                elif distance is None:
                                    pose = walking.update(0.0)
                                    gait = walking.telemetry_snapshot()
                                    status = "TOF WAIT - MOVE ROBOT CLOSER"
                                else:
                                    pose = walking.update(args.pickup_approach_speed)
                                    gait = walking.telemetry_snapshot()
                                    status = f"APPROACHING {object_name} {distance} MM"
                elif phase == "squat":
                    progress = min(1.0, (now - phase_started) / args.pickup_squat_duration_s)
                    pose = squat.update_depth(target_squat_mm * progress)
                    gait = stationary_gait("squat")
                    status = f"SQUAT {round(progress * 100)}%"
                    if progress >= 1.0:
                        enter("reach")
                elif phase == "reach":
                    progress = min(1.0, (now - phase_started) / args.pickup_reach_duration_s)
                    base_pose = squat.update_depth(target_squat_mm)
                    pose = _reach_pose(base_pose, round(args.pickup_reach_pwm * progress))
                    gait = stationary_gait("reach")
                    status = f"REACH {round(progress * 100)}%"
                    if progress >= 1.0:
                        enter("ready")
                elif phase == "ready":
                    pose = _reach_pose(squat.update_depth(target_squat_mm), args.pickup_reach_pwm)
                    gait = stationary_gait("reach")
                    status = "OBJECT READY - PRESS R TO LIFT"
                elif phase == "lift":
                    progress = min(1.0, (now - phase_started) / args.pickup_lift_duration_s)
                    pose = _reach_pose(
                        squat.update_depth(target_squat_mm * (1.0 - progress)),
                        args.pickup_reach_pwm,
                    )
                    gait = stationary_gait("lift")
                    status = f"LIFT {round(progress * 100)}%"
                    if progress >= 1.0:
                        enter("holding")
                elif phase == "holding":
                    pose = _reach_pose(STANDING, args.pickup_reach_pwm)
                    gait = stationary_gait("hold")
                    status = "LIFTED - PRESS R TO RESET"

                gait["crouch_mm"] = squat.depth_mm

                fall_active = False
                was_triggered = fall_detector.triggered if fall_detector is not None else False
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
                        if not was_triggered:
                            cancel()
                            print(f"[pickup] FALL: {fall_detector.reason}. Arms forward.")
                        pose = extend_arms_forward(last_pose, args.fall_arm_forward_pwm)
                        status = f"FALL: {fall_detector.reason}"
                    elif was_triggered:
                        pose = dict(STANDING)
                        status = "UPRIGHT - ARMS RETURNED"
                last_fall_at = now

                backend.send(pose, duration_ms=args.update_ms)
                last_pose = dict(pose)
                dashboard.publish(
                    pose=pose,
                    gait=gait,
                    sensor_snapshot=snapshot,
                    status=status,
                    active=phase != "idle" or fall_active,
                    camera_ready=camera_ready,
                    balance_status=(
                        "FALL ACTIVE"
                        if fall_active
                        else "FALL READY" if fall_detector is not None else "PICKUP"
                    ),
                )
                dashboard.set_runtime("pickup", status)
                remaining = args.update_ms / 1000.0 - (time.monotonic() - loop_started)
                if remaining > 0.0:
                    time.sleep(remaining)
    finally:
        if sensor_hub is not None:
            sensor_hub.close()
        camera.set_detector(None)
        dashboard.set_runtime("idle", "Pickup stopped")
        print("[pickup] Pick Up exited.")
