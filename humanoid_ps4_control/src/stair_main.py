from __future__ import annotations

import math
import time
from pathlib import Path

from .balance import (
    BalanceConfig,
    IMUBalanceController,
    angle_error_deg,
)
from .config import Config, ROBOT, STANDING
from .gait_dashboard import stationary_gait
from .stair_motion import StairStepEngine
from .stair_perception import StairDetector, estimate_stair_geometry
from .walking_engine import DynamicWalkingEngine


def run_terrain_auto(
    args: Config,
    dashboard,
    camera,
    backend,
    sensor_hub,
    fall_safety,
) -> None:
    dashboard.set_runtime("terrain", "Starting Terrain Auto")
    model_path = Path(__file__).resolve().parent.parent / args.stair_model
    detector_error = ""
    try:
        detector = StairDetector(
            model_path=str(model_path),
            confidence=args.stair_model_confidence,
            iou_threshold=args.stair_model_iou_threshold,
            input_size=args.stair_model_input_size,
            detect_every_frames=args.stair_detect_every_frames,
        )
    except Exception as exc:
        detector = None
        detector_error = str(exc)
        print(f"[terrain] Stair vision unavailable: {exc}. IMU balance remains available.")
    camera.set_detector(detector, stable_frames=args.stair_detect_stable_frames)
    if detector is not None:
        mode = "ONNX+geometry" if detector.model_ready else "geometry fallback"
        print(f"[terrain] Stair detector configured ({mode}).")

    approach = DynamicWalkingEngine(
        dt=args.update_ms / 1000.0,
        step_time_s=args.auto_step_time_s,
        settle_time_s=args.auto_settle_time_s,
        max_step_len=args.stair_approach_step_mm,
        max_turn_step_len=args.stair_turn_step_mm,
        max_side_step_len=0.0,
        step_height=24.0,
        zmp_support_ratio=args.zmp_support_ratio,
        ankle_roll_gain=args.ankle_roll_gain,
        arm_swing_pwm=0,
    )
    stepper = StairStepEngine(
        clearance_mm=args.stair_foot_clearance_mm,
        shift_s=args.stair_phase_shift_s,
        swing_s=args.stair_phase_swing_s,
        transfer_s=args.stair_phase_transfer_s,
        settle_s=args.stair_phase_settle_s,
        zmp_support_ratio=args.zmp_support_ratio,
        ankle_roll_gain=args.ankle_roll_gain,
        crouch_depth_mm=args.stair_crouch_depth_mm,
    )

    reference = None
    balance = None
    balance_enabled = False
    previous_fall_active = fall_safety.active
    enabled = False
    stable_frames = 0
    last_detection_timestamp = None
    last_depth_timestamp = None
    last_geometry = None
    last_balance_at = time.monotonic()
    cooldown_until = 0.0
    lead_leg = "left"
    calibration_error = ""
    geometry_settings = (
        args.stair_foot_toe_mm, args.stair_foot_heel_mm, args.stair_foot_width_mm,
        args.stair_landing_margin_mm, args.stair_tof_forward_offset_mm,
        args.stair_tof_mount_height_mm, args.stair_tof_pitch_down_deg,
        args.stair_tof_vertical_fov_deg, args.stair_tread_depth_mm,
        args.stair_width_mm, args.stair_step_depth_mm,
        args.stair_min_riser_mm, args.stair_default_riser_mm, args.stair_max_riser_mm,
    )
    geometry_finite = all(math.isfinite(v) for v in geometry_settings)
    if not geometry_finite:
        calibration_error = "STAIR LOCKED | NON-FINITE GEOMETRY SETTINGS"
    elif not args.stair_geometry_calibrated:
        calibration_error = "STAIR PREVIEW | CALIBRATE TOF AND FOOT DIMENSIONS"
    elif min(args.stair_foot_toe_mm, args.stair_foot_heel_mm, args.stair_foot_width_mm) <= 0:
        calibration_error = "STAIR LOCKED | FOOT DIMENSIONS MISSING"
    elif args.stair_foot_toe_mm + args.stair_foot_heel_mm + 2 * args.stair_landing_margin_mm > args.stair_tread_depth_mm:
        calibration_error = "STAIR LOCKED | FOOT DOES NOT FIT TREAD"
    elif 2 * ROBOT["half_hip"] + args.stair_foot_width_mm + 2 * args.stair_landing_margin_mm > args.stair_width_mm:
        calibration_error = "STAIR LOCKED | FEET DO NOT FIT STAIR WIDTH"
    elif (
        args.stair_landing_margin_mm < 0 or args.stair_tof_mount_height_mm <= 0
        or not 0 < args.stair_tof_vertical_fov_deg < 90
        or abs(args.stair_tof_pitch_down_deg) + args.stair_tof_vertical_fov_deg / 2 >= 90
        or args.stair_step_depth_mm <= 0
        or not 0 < args.stair_min_riser_mm <= args.stair_default_riser_mm <= args.stair_max_riser_mm
    ):
        calibration_error = "STAIR LOCKED | INVALID FLOOR OR SENSOR GEOMETRY"

    try:
        with backend:
            dashboard.set_runtime("terrain", "Terrain Auto ready - V balance, U stairs")

            while True:
                loop_started = time.monotonic()
                control = dashboard.control_state()
                if control.reset:
                    stepper.reset()
                    approach.reset()
                    backend.send(STANDING, duration_ms=args.stop_ms, force=True)
                    break
                if not control.armed or control.mode != "terrain":
                    break

                shared_reference = fall_safety.reference
                if shared_reference is not None and shared_reference != reference:
                    reference = shared_reference
                    balance = IMUBalanceController(
                        BalanceConfig(
                            target_roll_deg=reference[0],
                            target_pitch_deg=reference[1],
                            max_correction_deg=args.terrain_balance_limit_deg,
                            deadband_deg=args.terrain_balance_deadband_deg,
                        )
                    )
                    balance_enabled = True
                    print("[terrain] Shared IMU reference ready; balance ON.")

                if control.auto_toggle:
                    if balance is None:
                        print("[terrain] IMU balance unavailable.")
                    else:
                        balance_enabled = not balance_enabled
                        balance.reset()
                        print(f"[terrain] IMU balance {'ON' if balance_enabled else 'OFF'}.")
                if control.stair_toggle:
                    enabled = not enabled
                    stable_frames = 0
                    if enabled:
                        message = "ON"
                    elif stepper.active:
                        message = "OFF AFTER CURRENT STEP"
                    else:
                        message = "OFF"
                    print(f"[terrain] Auto stair {message}.")
                if control.stop:
                    enabled = False
                    stable_frames = 0
                    cooldown_until = loop_started + args.stop_ms / 1000.0

                snapshot = sensor_hub.read() if sensor_hub is not None else None
                now = time.monotonic()
                imu = snapshot.imu if snapshot is not None else None
                depth = snapshot.depth if snapshot is not None else None
                pitch_delta = angle_error_deg(imu.pitch_deg, reference[1]) if imu is not None and reference is not None else 0.0
                roll_delta = angle_error_deg(imu.roll_deg, reference[0]) if imu is not None and reference is not None else 0.0
                stair_frame = camera.stair_frame()
                detection = stair_frame.primary_stair if stair_frame is not None else None
                detection_timestamp = stair_frame.captured_at if detection is not None else None
                if stair_frame is not None and now - stair_frame.captured_at > 0.8:
                    detection = None
                    detection_timestamp = None

                geometry = None
                if geometry_finite and (detection is not None or depth is not None):
                    geometry = estimate_stair_geometry(
                        detection,
                        depth,
                        default_riser_mm=args.stair_default_riser_mm,
                        min_riser_mm=args.stair_min_riser_mm,
                        max_riser_mm=args.stair_max_riser_mm,
                        mount_height_mm=args.stair_tof_mount_height_mm,
                        pitch_down_deg=args.stair_tof_pitch_down_deg + pitch_delta,
                        vertical_fov_deg=args.stair_tof_vertical_fov_deg,
                        flip_vertical=args.stair_tof_flip_vertical,
                        forward_offset_mm=args.stair_tof_forward_offset_mm,
                        roll_deg=roll_delta,
                    )

                valid_geometry = (
                    geometry is not None and geometry.riser_measured
                    and detection is not None and depth is not None
                    and geometry.confidence >= args.stair_model_confidence
                )
                if valid_geometry and not stepper.active:
                    depth_timestamp = depth.sensor_time_ms
                    if detection_timestamp != last_detection_timestamp and depth_timestamp != last_depth_timestamp:
                        last_detection_timestamp = detection_timestamp
                        last_depth_timestamp = depth_timestamp
                        if last_geometry is not None and (
                            geometry.direction != last_geometry.direction
                            or abs(geometry.riser_height_mm - last_geometry.riser_height_mm) > 5.0
                            or abs(geometry.edge_distance_mm - last_geometry.edge_distance_mm) > max(
                                10.0, geometry.edge_uncertainty_mm + last_geometry.edge_uncertainty_mm,
                            )
                        ):
                            stable_frames = 0
                        stable_frames += 1
                        last_geometry = geometry
                else:
                    stable_frames = 0
                    last_geometry = None
                    last_detection_timestamp = detection_timestamp
                    last_depth_timestamp = depth.sensor_time_ms if depth is not None else None

                pose = dict(STANDING)
                gait = stationary_gait("terrain-wait")
                forward = turn = 0.0
                max_stride = min(args.stair_step_depth_mm, stepper.max_stride_mm(
                    geometry.riser_height_mm if geometry is not None else args.stair_default_riser_mm,
                )) if geometry_finite else 0.0
                edge_near = edge_far = landing_stride = landing_max = None
                if geometry is not None and geometry.edge_distance_mm is not None:
                    edge_near = geometry.edge_distance_mm - geometry.edge_uncertainty_mm
                    edge_far = geometry.edge_distance_mm + geometry.edge_uncertainty_mm
                    landing_stride = edge_far + args.stair_foot_heel_mm + args.stair_landing_margin_mm
                    landing_max = edge_near + args.stair_tread_depth_mm - args.stair_foot_toe_mm - args.stair_landing_margin_mm
                preview = ""
                if geometry is not None:
                    if geometry.direction == "unknown":
                        preview = "CAMERA EDGE | TOF EDGE UNKNOWN"
                    else:
                        lift_mm = geometry.riser_height_mm + args.stair_foot_clearance_mm
                        preview = (
                            f"{geometry.direction.upper()} {geometry.edge_distance_mm} MM"
                            f" | LIFT {lift_mm:.0f} MM"
                        )
                status = "BALANCE ON | STAIR OFF" if balance_enabled else "TERRAIN IDLE"
                if not enabled and preview:
                    status += f" | PREVIEW {preview}"
                fall_active = fall_safety.active
                if fall_active:
                    enabled = False
                    stepper.reset()
                    approach.reset()
                    stable_frames = 0
                    pose = backend.current_pose
                    status = f"FALL: {fall_safety.reason}"
                    gait = stationary_gait("fall")
                elif previous_fall_active:
                    status = "UPRIGHT - ARMS RETURNED"
                    cooldown_until = now + args.stop_ms / 1000.0
                elif stepper.active:
                    pause_reason = ""
                    if imu is None or reference is None:
                        pause_reason = "IMU LOST"
                    elif not imu.balance_ready(args.imu_min_gyro_cal, args.imu_min_accel_cal):
                        pause_reason = "IMU NOT CALIBRATED"
                    elif depth is None or depth.center_distance_mm is None:
                        pause_reason = "TOF LOST"
                    elif stair_frame is None or now - stair_frame.captured_at > 0.8:
                        pause_reason = "CAMERA LOST"
                    elif max(abs(roll_delta), abs(pitch_delta)) > args.terrain_balance_limit_deg:
                        pause_reason = "TILT LIMIT"
                    elif stepper.paused and not enabled:
                        pause_reason = "PRESS U TO RESUME"
                    if pause_reason:
                        enabled = False
                    pose = stepper.update(now, paused=bool(pause_reason))
                    gait = stepper.telemetry_snapshot()
                    prefix = "STAIR" if enabled else "STOPPING"
                    status = f"STAIR HOLD | {pause_reason}" if pause_reason else f"{prefix} {stepper.direction.upper()} | {stepper.phase.upper()}"
                    if not stepper.active:
                        cooldown_until = now + args.stair_step_pause_s
                        lead_leg = "right" if lead_leg == "left" else "left"
                        stable_frames = 0
                        approach.reset()
                elif not enabled and not approach.is_idle_ready():
                    status = "FINISHING APPROACH STEP"
                elif enabled and now < cooldown_until:
                    status = "VERIFYING NEXT STEP"
                elif enabled and calibration_error:
                    status = f"{calibration_error} | {preview}" if preview else calibration_error
                elif enabled and (
                    imu is None or reference is None or not balance_enabled
                    or not imu.balance_ready(args.imu_min_gyro_cal, args.imu_min_accel_cal)
                    or abs(roll_delta) > 3.0 or abs(pitch_delta) > 3.0
                ):
                    status = "WAITING FOR UPRIGHT IMU AND BALANCE"
                elif enabled and detector_error:
                    status = f"STAIR VISION UNAVAILABLE | {detector_error}"
                elif enabled and (stair_frame is None or now - stair_frame.captured_at > 0.8):
                    status = "WAITING FOR LIVE CAMERA"
                elif enabled and depth is None:
                    status = "WAITING FOR LIVE TOF"
                elif enabled and geometry is None:
                    status = "SEARCHING FOR STAIRS"
                elif enabled and detection is None:
                    status = "NO CAMERA STAIR | TOF PREVIEW ONLY"
                elif enabled and not geometry.riser_measured:
                    status = "STAIR FOUND | TOF FLOOR LEVELS NOT RESOLVED"
                elif enabled and stable_frames < args.stair_detect_stable_frames:
                    status = f"VERIFYING STAIR {stable_frames}/{args.stair_detect_stable_frames}"
                elif enabled and geometry.edge_distance_mm is None:
                    status = "WAITING FOR TOF DISTANCE"
                elif enabled and edge_near < args.stair_foot_toe_mm + args.stair_landing_margin_mm:
                    status = "TOO CLOSE TO EDGE | REPOSITION MANUALLY"
                elif enabled and abs(geometry.center_error) > args.stair_camera_align_deadband:
                    turn_room = edge_near > args.stair_foot_toe_mm + args.stair_landing_margin_mm + 18.0
                    turn = -math.copysign(1.0, geometry.center_error) if turn_room else 0.0
                    status = f"ALIGNING {geometry.center_error:+.2f}" if turn_room else "TOO CLOSE TO TURN | REPOSITION MANUALLY"
                elif enabled and (landing_stride > max_stride or landing_stride > landing_max) and edge_near > args.stair_foot_toe_mm + args.stair_landing_margin_mm + 2 * args.stair_approach_step_mm:
                    forward = 1.0
                    status = f"APPROACHING {geometry.edge_distance_mm} MM"
                elif enabled and landing_stride > landing_max:
                    status = "STAIR EDGE TOO UNCERTAIN FOR FULL FOOT LANDING"
                elif enabled and landing_stride > max_stride:
                    status = "REQUIRED LANDING STRIDE EXCEEDS STAIR REACH"
                elif enabled and not approach.is_idle_ready():
                    status = "FINISHING APPROACH STEP"
                elif enabled:
                    try:
                        stepper.start(
                            geometry.direction,
                            geometry.riser_height_mm,
                            landing_stride,
                            lead_leg,
                            now,
                        )
                        pose = stepper.update(now)
                        gait = stepper.telemetry_snapshot()
                        status = f"STAIR {geometry.direction.upper()} | SHIFT"
                    except ValueError as exc:
                        enabled = False
                        status = f"STAIR REJECTED | {exc}"
                        print(f"[terrain] {status}")

                if not fall_active and not previous_fall_active and gait["phase"] == "terrain-wait" and (
                    forward or turn or not approach.is_idle_ready()
                ):
                    pose = approach.update(forward, turn, 0.0)
                    gait = approach.telemetry_snapshot()
                previous_fall_active = fall_active
                gait["terrain"] = {
                    "stairs_enabled": enabled,
                    "calibration_error": calibration_error,
                    "stable_frames": stable_frames,
                    "max_stride_mm": max_stride,
                }
                if geometry is not None:
                    gait["perception"] = {
                        "direction": geometry.direction,
                        "confidence": geometry.confidence,
                        "edge_mm": geometry.edge_distance_mm,
                        "riser_mm": geometry.riser_height_mm,
                        "riser_measured": geometry.riser_measured,
                        "lift_mm": geometry.riser_height_mm + args.stair_foot_clearance_mm,
                        "center_error": geometry.center_error,
                        "source": geometry.source,
                        "edge_uncertainty_mm": geometry.edge_uncertainty_mm,
                        "landing_stride_mm": landing_stride,
                        "tread_depth_mm": args.stair_tread_depth_mm,
                        "calibrated": args.stair_geometry_calibrated,
                    }
                dt = max(0.001, now - last_balance_at)
                last_balance_at = now
                balance_active = (
                    balance_enabled and balance is not None and imu is not None
                    and not fall_active and not enabled and not stepper.active
                    and approach.is_idle_ready() and gait["phase"] == "terrain-wait"
                    and now >= cooldown_until
                )
                if balance_active:
                    pose = balance.apply(
                        pose,
                        roll_deg=imu.roll_deg,
                        pitch_deg=imu.pitch_deg,
                        dt=dt,
                    )
                elif balance is not None:
                    balance.reset()

                backend.send(pose, duration_ms=args.update_ms)
                dashboard.publish(
                    pose=backend.current_pose,
                    gait=gait,
                    sensor_snapshot=snapshot,
                    status=status,
                    active=enabled or stepper.active or balance_enabled or fall_active,
                    balance_status=(
                        fall_safety.status
                        if fall_active or balance is None
                        else f"{'IMU ON' if balance_active else 'IMU PAUSED' if balance_enabled else 'IMU OFF'} | {fall_safety.status}"
                    ),
                )
                dashboard.set_runtime("terrain", status)
                remaining = args.update_ms / 1000.0 - (time.monotonic() - loop_started)
                if remaining > 0.0:
                    time.sleep(remaining)

            holding = stepper.active or not approach.is_idle_ready() or fall_safety.active
            backend.send(backend.current_pose if holding else STANDING, duration_ms=args.stop_ms, force=True)
            time.sleep(args.stop_ms / 1000.0)
    finally:
        camera.set_detector(None)
        holding = stepper.active or not approach.is_idle_ready() or fall_safety.active
        status = "STAIR HOLD | SUPPORT ROBOT BEFORE RESET" if holding else "Terrain Auto stopped"
        if holding:
            dashboard.disarm(status)
        dashboard.set_runtime("idle", status)
        print("[terrain] Terrain Auto exited.")
