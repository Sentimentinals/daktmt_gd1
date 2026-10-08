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

    try:
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

                    forward = 0.0
                    turn = 0.0
                    status = "FOLLOW READY"
                    if follow.enabled:
                        frame = camera.person_frame() or PersonFrame()
                        perception_at = time.monotonic()
                        distance_mm = depth.front_distance_mm if depth is not None else None
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
                        elif distance_mm is None:
                            obstacle_planner.reset()
                            forward = turn = 0.0
                            status = f"TARGET #{follow.target_id} TOF WAIT"
                        elif (distance_mm <= args.tof_obstacle_stop_mm
                              and kind != "OBJECT"):
                            obstacle_planner.reset()
                            forward = turn = 0.0
                            status = f"TARGET #{follow.target_id} WAIT {kind} | TOF {distance_mm} MM"
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

                    if camera.detection_error:
                        forward = turn = 0.0
                        status = f"PERSON DETECT UNAVAILABLE: {camera.detection_error}"

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
                    elif previous_fall_active:
                        pose = dict(STANDING)
                        status = "UPRIGHT - ARMS RETURNED"
                    previous_fall_active = fall_active
                    pose[25] = STANDING[25]
                    backend.send(pose, duration_ms=args.update_ms)
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
        dashboard.set_runtime("idle", "Person follow stopped")
        print("[follow] Person Follow exited.")
