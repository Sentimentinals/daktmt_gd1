from __future__ import annotations

import time
from pathlib import Path

from .config import Config, STANDING
from .person_follow import PersonDetector, PersonFollowController, PersonFrame
from .sensors import DepthObstacleGuard
from .walking_engine import DynamicWalkingEngine


def run_follow(
    args: Config,
    dashboard,
    camera,
    camera_ready: bool,
    backend,
    sensor_hub,
    fall_safety,
) -> None:
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
    previous_follow = False
    previous_ignore = False
    previous_stop = False
    previous_fall_active = fall_safety.active
    last_pose = dict(STANDING)

    try:
        with backend:
            last_pose = backend.current_pose
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
                        if not fall_safety.active:
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

                    fall_active = fall_safety.active
                    if fall_active:
                        if not previous_fall_active:
                            follow.disable()
                            engine.reset()
                        pose = backend.current_pose
                        status = f"FALL: {fall_safety.reason}"
                    elif previous_fall_active:
                        pose = dict(STANDING)
                        status = "UPRIGHT - ARMS RETURNED"
                    previous_fall_active = fall_active
                    pose[25] = STANDING[25]
                    backend.send(pose, duration_ms=args.update_ms)
                    last_pose = backend.current_pose
                    dashboard.publish(
                        pose=last_pose,
                        gait=engine.telemetry_snapshot(),
                        sensor_snapshot=snapshot,
                        status=status,
                        active=fall_active or follow.enabled or not engine.is_idle_ready(),
                        camera_ready=camera_ready,
                        balance_status=fall_safety.status,
                    )
                    dashboard.set_runtime("follow", status)
                    remaining = args.update_ms / 1000.0 - (time.monotonic() - loop_started)
                    if remaining > 0.0:
                        time.sleep(remaining)
            finally:
                try:
                    exit_pose = (
                        backend.current_pose if fall_safety.active else STANDING
                    )
                    backend.send(exit_pose, duration_ms=args.stop_ms, force=True)
                    time.sleep(args.stop_ms / 1000.0)
                except Exception as exc:
                    print(f"[follow] Failed to return to STANDING: {exc}")
    finally:
        camera.set_detector(None)
        dashboard.set_runtime("idle", "Person follow stopped")
        print("[follow] Person Follow exited.")
