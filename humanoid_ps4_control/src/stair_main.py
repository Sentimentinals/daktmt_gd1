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
    camera_ready: bool,
    backend,
    sensor_hub,
    fall_safety,
) -> None:
    dashboard.set_runtime("terrain", "Starting Terrain Auto")
    model_path = Path(__file__).resolve().parent.parent / args.stair_model
    detector = StairDetector(
        model_path=str(model_path),
        confidence=args.stair_model_confidence,
        iou_threshold=args.stair_model_iou_threshold,
        input_size=args.stair_model_input_size,
        detect_every_frames=args.stair_detect_every_frames,
    )
    if camera_ready:
        camera.set_detector(detector, stable_frames=args.stair_detect_stable_frames)
        mode = "ONNX+geometry" if detector.model_ready else "geometry fallback"
        print(f"[terrain] Stair detector ON ({mode}).")
    else:
        print("[terrain] Camera unavailable. Auto stair remains locked.")

    approach = DynamicWalkingEngine(
        dt=args.update_ms / 1000.0,
        t_step=max(1.25, args.t_step),
        t_dbl=args.t_dbl,
        max_step_len=18.0,
        max_turn_step_len=4.0,
        max_side_step_len=0.0,
        step_height=24.0,
        crouch_depth_mm=0.0,
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
    previous_toggle = False
    previous_balance_toggle = False
    previous_stop = False
    stable_frames = 0
    last_detection_at = 0.0
    last_stair_timestamp = None
    last_depth_timestamp = None
    last_direction = "unknown"
    last_balance_at = time.monotonic()
    cooldown_until = 0.0
    lead_leg = "left"
    last_pose = dict(STANDING)
    calibration_error = ""
    if not args.stair_geometry_calibrated:
        calibration_error = "STAIR PREVIEW | CALIBRATE TOF AND FOOT DIMENSIONS"
    elif min(args.stair_foot_toe_mm, args.stair_foot_heel_mm, args.stair_foot_width_mm) <= 0:
        calibration_error = "STAIR LOCKED | FOOT DIMENSIONS MISSING"
    elif args.stair_foot_toe_mm + args.stair_foot_heel_mm + 2 * args.stair_landing_margin_mm > args.stair_tread_depth_mm:
        calibration_error = "STAIR LOCKED | FOOT DOES NOT FIT TREAD"
    elif 2 * ROBOT["half_hip"] + args.stair_foot_width_mm + 2 * args.stair_landing_margin_mm > args.stair_width_mm:
        calibration_error = "STAIR LOCKED | FEET DO NOT FIT STAIR WIDTH"

    try:
        with backend:
            last_pose = backend.current_pose
            dashboard.set_runtime("terrain", "Terrain Auto ready - V balance, U stairs")

            while True:
                loop_started = time.monotonic()
                control = dashboard.control_state()
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
                            roll_deadband_deg=args.terrain_balance_deadband_deg,
                            pitch_deadband_deg=args.terrain_balance_deadband_deg,
                            pitch_ankle_gain=1.0,
                            pitch_hip_gain=0.30,
                            roll_ankle_gain=1.0,
                            roll_hip_gain=0.25,
                            double_support_gain=1.0,
                        )
                    )
                    balance_enabled = True
                    print("[terrain] Shared IMU reference ready; balance ON.")

                if control.auto_toggle and not previous_balance_toggle:
                    if balance is None:
                        print("[terrain] IMU balance unavailable.")
                    else:
                        balance_enabled = not balance_enabled
                        balance.reset()
                        print(f"[terrain] IMU balance {'ON' if balance_enabled else 'OFF'}.")
                previous_balance_toggle = control.auto_toggle

                if control.stair_toggle and not previous_toggle:
                    if not camera_ready:
                        enabled = False
                        print("[terrain] Auto stair rejected: camera unavailable.")
                    else:
                        enabled = not enabled
                        stable_frames = 0
                        if enabled:
                            message = "ON"
                        elif stepper.active:
                            message = "OFF AFTER CURRENT STEP"
                        else:
                            message = "OFF"
                        print(f"[terrain] Auto stair {message}.")
                previous_toggle = control.stair_toggle

                if control.stop and not previous_stop:
                    enabled = False
                    stable_frames = 0
                    stepper.reset()
                    approach.reset()
                    backend.send(STANDING, duration_ms=args.stop_ms, force=True)
                previous_stop = control.stop

                snapshot = sensor_hub.read() if sensor_hub is not None else None
                now = time.monotonic()
                imu = snapshot.imu if snapshot is not None else None
                depth = snapshot.depth if snapshot is not None else None
                pitch_delta = angle_error_deg(imu.pitch_deg, reference[1]) if imu is not None and reference is not None else 0.0
                roll_delta = angle_error_deg(imu.roll_deg, reference[0]) if imu is not None and reference is not None else 0.0
                stair_frame = camera.stair_frame() if camera_ready else None
                detection = stair_frame.primary_stair if stair_frame is not None else None
                if stair_frame is not None and now - stair_frame.captured_at > 0.8:
                    detection = None
                geometry = None
                if detection is not None:
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
                    )
                    depth_timestamp = depth.sensor_time_ms if depth is not None else None
                    if geometry.direction != last_direction or depth is None:
                        stable_frames = 0
                    last_direction = geometry.direction
                    if stair_frame.captured_at != last_stair_timestamp and depth_timestamp != last_depth_timestamp:
                        last_stair_timestamp = stair_frame.captured_at
                        last_depth_timestamp = depth_timestamp
                        if (
                            geometry.direction != "unknown"
                            and geometry.confidence >= args.stair_model_confidence
                        ):
                            stable_frames += 1
                            last_detection_at = now
                        else:
                            stable_frames = 0
                elif now - last_detection_at > 0.5:
                    stable_frames = 0

                pose = dict(STANDING)
                gait = stationary_gait("terrain-wait")
                edge_near = edge_far = landing_stride = landing_max = None
                if geometry is not None and geometry.edge_distance_mm is not None:
                    edge_near = geometry.edge_distance_mm - geometry.edge_uncertainty_mm
                    edge_far = geometry.edge_distance_mm + geometry.edge_uncertainty_mm
                    landing_stride = edge_far + args.stair_foot_heel_mm + args.stair_landing_margin_mm
                    landing_max = edge_near + args.stair_tread_depth_mm - args.stair_foot_toe_mm - args.stair_landing_margin_mm
                status = "BALANCE ON | STAIR OFF" if balance_enabled else "TERRAIN IDLE"
                if stepper.active:
                    pose = stepper.update(now)
                    gait = stepper.telemetry_snapshot()
                    prefix = "STAIR" if enabled else "STOPPING"
                    status = f"{prefix} {stepper.direction.upper()} | {stepper.phase.upper()}"
                    if not stepper.active:
                        cooldown_until = now + args.stair_step_pause_s
                        lead_leg = "right" if lead_leg == "left" else "left"
                        stable_frames = 0
                elif not enabled and not approach.is_idle_ready():
                    pose = approach.update(0.0)
                    gait = approach.telemetry_snapshot()
                    status = "FINISHING APPROACH STEP"
                elif enabled and now < cooldown_until:
                    status = "VERIFYING NEXT STEP"
                elif enabled and (calibration_error or not detector.model_ready):
                    status = calibration_error or "STAIR LOCKED | TRAINED MODEL MISSING"
                elif enabled and (imu is None or reference is None or not balance_enabled or abs(roll_delta) > 3.0 or abs(pitch_delta) > 3.0):
                    pose = approach.update(0.0)
                    gait = approach.telemetry_snapshot()
                    status = "WAITING FOR UPRIGHT IMU AND BALANCE"
                elif enabled and geometry is None:
                    pose = approach.update(0.0)
                    gait = approach.telemetry_snapshot()
                    status = "SEARCHING FOR STAIRS"
                elif enabled and geometry.direction == "unknown":
                    pose = approach.update(0.0)
                    gait = approach.telemetry_snapshot()
                    status = "STAIR FOUND | TOF HAS NOT RESOLVED TWO LEVELS"
                elif enabled and stable_frames < args.stair_detect_stable_frames:
                    pose = approach.update(0.0)
                    gait = approach.telemetry_snapshot()
                    status = f"VERIFYING STAIR {stable_frames}/{args.stair_detect_stable_frames}"
                elif enabled and geometry.edge_distance_mm is None:
                    pose = approach.update(0.0)
                    gait = approach.telemetry_snapshot()
                    status = "WAITING FOR TOF DISTANCE"
                elif enabled and landing_stride > landing_max:
                    pose = approach.update(0.0)
                    gait = approach.telemetry_snapshot()
                    status = "STAIR EDGE TOO UNCERTAIN FOR FULL FOOT LANDING"
                elif enabled and edge_near < args.stair_foot_toe_mm + args.stair_landing_margin_mm:
                    pose = approach.update(0.0)
                    gait = approach.telemetry_snapshot()
                    status = "TOO CLOSE TO EDGE | REPOSITION MANUALLY"
                elif enabled and abs(geometry.center_error) > args.stair_camera_align_deadband:
                    turn_room = edge_near > args.stair_foot_toe_mm + args.stair_landing_margin_mm + 18.0
                    turn = -math.copysign(args.stair_turn_speed, geometry.center_error) if turn_room else 0.0
                    pose = approach.update(0.0, turn, 0.0)
                    gait = approach.telemetry_snapshot()
                    status = f"ALIGNING {geometry.center_error:+.2f}" if turn_room else "TOO CLOSE TO TURN | REPOSITION MANUALLY"
                elif enabled and landing_stride > args.stair_step_depth_mm and edge_near > args.stair_foot_toe_mm + args.stair_landing_margin_mm + 18.0:
                    pose = approach.update(args.stair_approach_speed, 0.0, 0.0)
                    gait = approach.telemetry_snapshot()
                    status = f"APPROACHING {geometry.edge_distance_mm} MM"
                elif enabled and landing_stride > args.stair_step_depth_mm:
                    pose = approach.update(0.0)
                    gait = approach.telemetry_snapshot()
                    status = "REQUIRED LANDING STRIDE EXCEEDS STAIR REACH"
                elif enabled and not approach.is_idle_ready():
                    pose = approach.update(0.0)
                    gait = approach.telemetry_snapshot()
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

                if geometry is not None:
                    gait["perception"] = {
                        "direction": geometry.direction,
                        "confidence": geometry.confidence,
                        "edge_mm": geometry.edge_distance_mm,
                        "riser_mm": geometry.riser_height_mm,
                        "center_error": geometry.center_error,
                        "source": geometry.source,
                        "edge_uncertainty_mm": geometry.edge_uncertainty_mm,
                        "landing_stride_mm": landing_stride,
                        "tread_depth_mm": args.stair_tread_depth_mm,
                        "calibrated": args.stair_geometry_calibrated,
                    }

                dt = max(0.001, now - last_balance_at)
                last_balance_at = now
                fall_active = fall_safety.active
                if fall_active:
                    if not previous_fall_active:
                        enabled = False
                        stepper.reset()
                        approach.reset()
                    pose = backend.current_pose
                    status = f"FALL: {fall_safety.reason}"
                elif previous_fall_active:
                    pose = dict(STANDING)
                    status = "UPRIGHT - ARMS RETURNED"
                previous_fall_active = fall_active
                if not fall_active and imu is not None and balance is not None and balance_enabled:
                    pose = balance.apply(
                        pose,
                        roll_deg=imu.roll_deg,
                        pitch_deg=imu.pitch_deg,
                        dt=dt,
                        support_leg=str(gait.get("support_leg", "double")),
                    )

                backend.send(pose, duration_ms=args.update_ms)
                last_pose = backend.current_pose
                dashboard.publish(
                    pose=last_pose,
                    gait=gait,
                    sensor_snapshot=snapshot,
                    status=status,
                    active=enabled or stepper.active or balance_enabled or fall_active,
                    camera_ready=camera_ready,
                    balance_status=(
                        fall_safety.status
                        if fall_active or balance is None
                        else f"{'IMU ON' if balance_enabled else 'IMU OFF'} | {fall_safety.status}"
                    ),
                )
                dashboard.set_runtime("terrain", status)
                remaining = args.update_ms / 1000.0 - (time.monotonic() - loop_started)
                if remaining > 0.0:
                    time.sleep(remaining)

            backend.send(STANDING, duration_ms=args.stop_ms, force=True)
            time.sleep(args.stop_ms / 1000.0)
    finally:
        camera.set_detector(None)
        dashboard.set_runtime("idle", "Terrain Auto stopped")
        print("[terrain] Terrain Auto exited.")
