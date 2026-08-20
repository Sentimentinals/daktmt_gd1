from __future__ import annotations

import time
from pathlib import Path

from .backends import make_backend
from .config import Config, STANDING
from .person_follow import PersonDetector, PersonFollowController, PersonFrame
from .sensors import DepthObstacleGuard, RobotSensorHub
from .walking_engine import DynamicWalkingEngine


def run_follow(args: Config, dashboard, camera, camera_ready: bool) -> None:
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
        max_leg_pwm_per_s=args.walk_max_leg_pwm_per_s,
        command_rate_limit=args.command_rate_limit,
        arm_swing_pwm=args.arm_swing_pwm,
        arm_right_dir=args.arm_right_dir,
        arm_left_dir=args.arm_left_dir,
        arm_smooth_tau=args.arm_smooth_tau,
        arm_min_pwm=args.arm_min_pwm,
        arm_quantum_pwm=args.arm_quantum_pwm,
        prepare_hold_s=args.walk_prepare_hold_s,
        prepare_step_s=args.walk_prepare_step_s,
        prepare_lift_mm=args.walk_prepare_lift_mm,
    )
    obstacle_guard = DepthObstacleGuard(
        stop_distance_mm=args.tof_obstacle_stop_mm,
        clear_margin_mm=args.tof_obstacle_clear_margin_mm,
        stable_frames=args.tof_obstacle_stable_frames,
    )
    sensor_hub = None
    if args.sensor_feedback and args.sensor_use_depth:
        sensor_hub = RobotSensorHub(
            port=args.sensor_port,
            baudrate=args.sensor_baudrate,
            timeout_s=args.sensor_timeout_s,
            depth_timeout_s=args.sensor_depth_timeout_s,
            use_imu=False,
            use_foot_fsr=False,
            use_depth=True,
        )
        try:
            sensor_hub.open(wait_for_connection=False)
        except Exception as exc:
            sensor_hub = None
            print(f"[follow] ToF unavailable: {exc}")

    backend = make_backend(
        mode=args.backend,
        port=args.port,
        baudrate=args.baudrate,
        csv_path=args.csv,
    )
    previous_follow = False
    previous_ignore = False
    previous_stop = False
    head_pwm = float(STANDING[25])
    last_head_at = time.monotonic()

    try:
        with backend:
            backend.send(STANDING, duration_ms=900, force=True)
            time.sleep(0.9)
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
                        backend.send(STANDING, duration_ms=args.stop_ms, force=True)
                        print("[follow] Stopped at STANDING.")
                    previous_stop = stop_pressed

                    snapshot = sensor_hub.read() if sensor_hub is not None else None
                    depth = snapshot.depth if snapshot is not None else None
                    obstacle_blocked, obstacle_mm = obstacle_guard.update(depth)
                    depth_missing = bool(
                        args.sensor_feedback and args.sensor_use_depth and depth is None
                    )

                    forward = 0.0
                    turn = 0.0
                    status = "FOLLOW READY"
                    if follow.enabled:
                        frame = camera.person_frame() or PersonFrame()
                        forward, turn, status = follow.command(frame)
                        if status in ("TARGET LOST", "MULTIPLE PEOPLE"):
                            follow.disable()
                            engine.reset()
                            forward = 0.0
                            turn = 0.0
                            print(f"[follow] Stopped: {status.lower()}.")
                        elif forward > 0.0 and (obstacle_blocked or depth_missing):
                            forward = 0.0
                            status = (
                                f"OBJECT {obstacle_mm} MM"
                                if obstacle_blocked and obstacle_mm is not None
                                else "TOF WAIT"
                            )

                    if follow.enabled or not engine.is_idle_ready():
                        pose = engine.update(forward, turn_cmd=turn)
                    else:
                        pose = dict(STANDING)

                    head_target = STANDING[25] + args.head_pan_direction * args.head_pan_pwm * (
                        1 if turn > 0.0 else -1 if turn < 0.0 else 0
                    )
                    now = time.monotonic()
                    max_head_delta = args.head_pan_rate_pwm_s * max(0.0, now - last_head_at)
                    head_pwm += max(-max_head_delta, min(max_head_delta, head_target - head_pwm))
                    last_head_at = now
                    pose[25] = max(500, min(2500, round(head_pwm)))
                    backend.send(pose, duration_ms=args.update_ms)
                    dashboard.publish(
                        pose=pose,
                        gait=engine.telemetry_snapshot(),
                        sensor_snapshot=snapshot,
                        status=status,
                        active=follow.enabled or not engine.is_idle_ready(),
                        camera_ready=camera_ready,
                        balance_status="FOLLOW ON" if follow.enabled else "FOLLOW READY",
                    )
                    dashboard.set_runtime("follow", status)
                    remaining = args.update_ms / 1000.0 - (time.monotonic() - loop_started)
                    if remaining > 0.0:
                        time.sleep(remaining)
            finally:
                try:
                    backend.send(STANDING, duration_ms=args.stop_ms, force=True)
                    time.sleep(args.stop_ms / 1000.0)
                except Exception as exc:
                    print(f"[follow] Failed to return to STANDING: {exc}")
    finally:
        if sensor_hub is not None:
            sensor_hub.close()
        camera.set_detector(None)
        dashboard.set_runtime("idle", "Person follow stopped")
        print("[follow] Person Follow exited.")
