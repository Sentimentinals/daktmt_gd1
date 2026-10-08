from __future__ import annotations

import sys
import time

from .config import Config, STANDING
from .person_follow import (
    PersonFollowController,
    PersonFrame,
    PersonObstaclePlanner,
    PersonDepthAssociation,
)
from .walking_engine import DynamicWalkingEngine


def run_follow(
    args: Config,
    dashboard,
    camera,
    backend,
    sensor_hub,
    fall_safety,
) -> None:
    dashboard.set_runtime("follow", "Starting Person Follow")
    follow = PersonFollowController(
        turn_deadband=args.person_follow_turn_deadband,
        target_distance_mm=args.tof_obstacle_stop_mm,
        crawl_band_mm=args.person_follow_crawl_band_mm,
        slow_range_mm=args.person_follow_slow_range_mm,
        tof_filter_alpha=args.person_follow_tof_filter_alpha,
    )
    engine = DynamicWalkingEngine(
        dt=args.update_ms / 1000.0,
        step_time_s=args.auto_step_time_s,
        settle_time_s=args.auto_settle_time_s,
        max_step_len=args.person_follow_step_length_mm,
        max_turn_step_len=args.person_follow_turn_length_mm,
        step_height=args.walk_step_height_mm,
        hip_out_deg=args.walk_hip_out_deg,
        crouch_depth_mm=args.walk_crouch_depth_mm,
        forward_lean_deg=args.walk_forward_lean_deg,
        zmp_support_ratio=args.zmp_support_ratio,
        ankle_roll_gain=args.ankle_roll_gain,
        lift_start_phase=args.walk_lift_start_phase,
        swing_advance_end_phase=args.walk_swing_advance_end_phase,
        lift_end_phase=args.walk_lift_end_phase,
    )
    obstacle_planner = PersonObstaclePlanner(
        stop_distance_mm=args.tof_obstacle_stop_mm,
        clear_margin_mm=args.tof_obstacle_clear_margin_mm,
        stable_frames=args.tof_obstacle_stable_frames,
    )
    association = PersonDepthAssociation(
        height_mm=args.person_camera_tof_height_mm,
        camera_hfov_deg=args.person_camera_hfov_deg,
        camera_vfov_deg=args.person_camera_vfov_deg,
        tof_fov_deg=args.person_tof_fov_deg,
        flip_vertical=args.person_tof_flip_vertical,
        stable_frames=args.person_detect_stable_frames,
    )
    previous_fall_active = fall_safety.active
    next_tof_log = 0.0
    previous_hold = None
    previous_depth_logging = sensor_hub.log_depth if sensor_hub is not None else False

    try:
        if sensor_hub is not None:
            sensor_hub.log_depth = False
        with backend:
            try:
                dashboard.set_runtime("follow", "Follow ready")
                while True:
                    loop_started = time.monotonic()
                    control = dashboard.control_state("follow")
                    if not control.armed or control.mode != "follow":
                        break

                    if control.follow:
                        frame = camera.person_frame() or PersonFrame()
                        person = frame.single_person
                        if camera.person_ready() and person is not None:
                            follow.enable(person.track_id)
                            engine.reset()
                            obstacle_planner.reset()
                            association.reset()
                            print(f"[follow] Target #{person.track_id} locked.")
                        else:
                            print("[follow] Follow rejected: one stable person is required.")
                    if control.ignore_person:
                        if follow.enabled:
                            follow.disable()
                            engine.reset()
                            obstacle_planner.reset()
                            association.reset()
                            print("[follow] Person follow stopped.")
                        else:
                            camera.ignore_person()
                            print("[follow] Detected person ignored.")
                    if control.stop:
                        follow.disable()
                        engine.reset()
                        obstacle_planner.reset()
                        association.reset()
                        if not fall_safety.active:
                            backend.send(STANDING, duration_ms=args.stop_ms, force=True)
                            print("[follow] Stopped at STANDING.")
                    snapshot = sensor_hub.read() if sensor_hub is not None else None
                    depth = snapshot.depth if snapshot is not None else None
                    distance_mm = depth.front_distance_mm if depth is not None else None

                    forward = 0.0
                    turn = 0.0
                    status = "FOLLOW READY"
                    kind = "UNKNOWN"
                    stop_reason = "FOLLOW_DISABLED" if not follow.enabled else None
                    if follow.enabled:
                        frame = camera.person_frame() or PersonFrame()
                        perception_at = time.monotonic()
                        distance_sample_id = depth.sensor_time_ms if depth is not None else None
                        kind = association.update(frame, depth, now_s=perception_at)
                        forward, turn, status = follow.command(
                            frame,
                            distance_mm=distance_mm,
                            distance_sample_id=distance_sample_id,
                            now_s=perception_at,
                        )
                        if status == "TARGET LOST":
                            obstacle_planner.reset()
                            association.reset()
                            forward = 0.0
                            turn = 0.0
                            status = f"TARGET #{follow.target_id} WAIT CAMERA/TARGET"
                            stop_reason = "CAMERA/TARGET_LOST"
                        elif distance_mm is None:
                            obstacle_planner.reset()
                            forward = turn = 0.0
                            status = f"TARGET #{follow.target_id} TOF WAIT"
                            stop_reason = "TOF_UNAVAILABLE"
                        elif (distance_mm <= args.tof_obstacle_stop_mm
                              and kind != "OBJECT"):
                            obstacle_planner.reset()
                            forward = turn = 0.0
                            status = f"TARGET #{follow.target_id} WAIT {kind} | TOF {distance_mm} MM"
                            stop_reason = f"{kind}_TOO_CLOSE"
                        else:
                            # Only confirmed non-person evidence may override a close-range HOLD.
                            if " HOLD " in status and kind == "OBJECT":
                                forward = 1.0
                            forward, turn, avoid_status = obstacle_planner.update(
                                depth,
                                forward,
                                turn,
                            )
                            if avoid_status is not None:
                                status = f"TARGET #{follow.target_id} | {avoid_status}"
                            if avoid_status == "AVOID BLOCKED":
                                stop_reason = "NO_CLEAR_CORRIDOR"
                            elif avoid_status == "AVOID TOF WAIT":
                                stop_reason = "TOF_CORRIDOR_INVALID"
                            elif (
                                forward == 0.0 and obstacle_planner.direction
                                and distance_mm <= obstacle_planner.stop_distance_mm
                            ):
                                stop_reason = "OBSTACLE_TOO_CLOSE"
                            elif forward == 0.0 and turn == 0.0:
                                stop_reason = "DISTANCE_HOLD"

                    if camera.detection_error:
                        forward = turn = 0.0
                        status = f"PERSON DETECT UNAVAILABLE: {camera.detection_error}"
                        stop_reason = "CAMERA_UNAVAILABLE"

                    if follow.enabled or not engine.is_idle_ready():
                        pose = engine.update(forward, turn_cmd=turn)
                    else:
                        pose = dict(STANDING)

                    fall_active = fall_safety.active
                    if fall_active:
                        if not previous_fall_active:
                            follow.disable()
                            engine.reset()
                            obstacle_planner.reset()
                            association.reset()
                        pose = backend.current_pose
                        status = f"FALL: {fall_safety.reason}"
                        stop_reason = "FALL_SAFETY"
                    elif previous_fall_active:
                        pose = dict(STANDING)
                        status = "UPRIGHT - ARMS RETURNED"
                    previous_fall_active = fall_active
                    pose[25] = STANDING[25]
                    backend.send(pose, duration_ms=args.update_ms)
                    if fall_active:
                        motion = "FALL HOLD"
                    elif not follow.enabled:
                        motion = "IDLE"
                    elif forward == 0.0 and turn == 0.0:
                        motion = "STOPPED" if engine.is_idle_ready() else "HOLD"
                    elif forward == 0.0:
                        motion = "AVOID TURN" if obstacle_planner.direction else "ALIGN TURN"
                    else:
                        motion = "MOVING"
                    hold = (motion, stop_reason) if stop_reason and motion != "IDLE" else None
                    now = time.monotonic()
                    if now >= next_tof_log or hold != previous_hold:
                        corridors = obstacle_planner._corridors(depth) if depth is not None else (None,) * 3
                        ranges = "/".join(str(value) if value is not None else "--" for value in corridors)
                        # Angular grid coverage is not a calibrated physical obstacle width.
                        near_cols = sum(
                            any(20 <= depth.distances_mm[row * 8 + col] <= obstacle_planner.stop_distance_mm
                                for row in range(1, 7))
                            for col in range(8)
                        ) if depth is not None else "--"
                        resuming = previous_hold is not None and hold is None and follow.enabled
                        print(
                            f"[follow] {'RESUME ' if resuming else ''}{motion} "
                            f"reason={stop_reason or 'NONE'} | target={follow.target_id} kind={kind} "
                            f"tof={distance_mm if distance_mm is not None else '--'}mm "
                            f"stop(person/object)={follow.target_distance_mm}/{obstacle_planner.stop_distance_mm}mm "
                            f"corridors(L/C/R)={ranges}mm clear>={obstacle_planner.side_clear_mm}mm "
                            f"near-cols={near_cols}/8 | {status}",
                            flush=True,
                        )
                        next_tof_log = now + 2.0
                    previous_hold = hold
                    dashboard.publish(
                        pose=backend.current_pose,
                        gait=engine.telemetry_snapshot(),
                        sensor_snapshot=snapshot,
                        status=status,
                        active=fall_active or follow.enabled or not engine.is_idle_ready(),
                        balance_status=fall_safety.status,
                    )
                    dashboard.set_runtime("follow", status)
                    remaining = args.update_ms / 1000.0 - (time.monotonic() - loop_started)
                    if remaining > 0.0:
                        time.sleep(remaining)
            finally:
                handling_error = sys.exc_info()[0] is not None
                try:
                    exit_pose = (
                        backend.current_pose if fall_safety.active else STANDING
                    )
                    backend.send(exit_pose, duration_ms=args.stop_ms, force=True)
                    time.sleep(args.stop_ms / 1000.0)
                except Exception as exc:
                    print(f"[follow] Failed to send exit pose: {exc}")
                    if not handling_error:
                        raise
    finally:
        if sensor_hub is not None:
            sensor_hub.log_depth = previous_depth_logging
        dashboard.set_runtime("idle", "Person follow stopped")
        print("[follow] Person Follow exited.")
