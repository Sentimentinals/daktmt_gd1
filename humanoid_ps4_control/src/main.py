from __future__ import annotations

from contextlib import ExitStack
from pathlib import Path
import time

from .backends import make_backend
from .config import Config


def run_manual(
    args: Config,
    dashboard,
    backend,
    sensor_hub,
    fall_safety,
) -> None:
    from .arm_dance import ArmDanceEngine
    from .balance import angle_error_deg
    from .gait_dashboard import stationary_gait
    from .getup import GetupEngine
    from .sensors import DepthObstacleGuard
    from .walking_engine import DynamicWalkingEngine, SquatEngine, STANDING

    engine = DynamicWalkingEngine(
        dt=args.update_ms / 1000.0,
        t_step=args.t_step,
        t_dbl=args.t_dbl,
        max_step_len=args.walk_step_length_mm,
        max_turn_step_len=args.max_turn_step_len,
        max_side_step_len=args.max_side_step_len,
        step_height=args.walk_step_height_mm,
        hip_out_deg=args.walk_hip_out_deg,
        crouch_depth_mm=args.walk_crouch_depth_mm,
        forward_lean_deg=args.walk_forward_lean_deg,
        zmp_support_ratio=args.zmp_support_ratio,
        ankle_roll_gain=args.ankle_roll_gain,
        lift_start_phase=args.walk_lift_start_phase,
        swing_advance_end_phase=args.walk_swing_advance_end_phase,
        lift_end_phase=args.walk_lift_end_phase,
        landing_roll_release_start=args.walk_landing_roll_release_start,
        arm_swing_pwm=args.arm_swing_pwm,
        arm_right_dir=args.arm_right_dir,
        arm_left_dir=args.arm_left_dir,
    )
    arm_dance = ArmDanceEngine(
        dt=args.update_ms / 1000.0,
        transition_s=args.dance_transition,
        shoulder_pwm=args.dance_shoulder_pwm,
        elbow_pwm=args.dance_elbow_pwm,
        lift_pwm=args.dance_lift_pwm,
        head_pwm=args.dance_head_pwm,
    )
    squat = SquatEngine(
        dt=args.update_ms / 1000.0,
        depth_mm=args.manual_squat_depth_mm,
        forward_mm=args.manual_squat_forward_mm,
        arm_forward_pwm=args.manual_squat_arm_forward_pwm,
        arm_raise_s=args.manual_squat_arm_raise_s,
        transition_s=args.manual_squat_transition_s,
    )
    getup = GetupEngine(dt=args.update_ms / 1000.0, speed=args.getup_speed)
    obstacle_guard = DepthObstacleGuard(
        stop_distance_mm=args.tof_obstacle_stop_mm,
        clear_margin_mm=args.tof_obstacle_clear_margin_mm,
        stable_frames=args.tof_obstacle_stable_frames,
    )
    previous_getup = False
    previous_dance = False
    previous_fall = fall_safety.active
    recovery_active = False
    reset_until = 0.0
    dashboard.set_runtime("manual", "Manual control ready")

    try:
        with backend:
            try:
                while True:
                    started = time.monotonic()
                    state = dashboard.control_state("manual")
                    if not state.armed or state.mode != "manual":
                        break
                    snapshot = sensor_hub.read() if sensor_hub is not None else None
                    forward = state.forward * args.walk_speed
                    turn = state.turn * args.turn_speed
                    side = state.side * args.side_speed
                    locomotion_requested = bool(forward or turn or side)
                    reset_requested = state.stop or state.reset
                    if reset_requested:
                        engine.reset()
                        arm_dance.reset()
                        squat.reset()
                        getup.reset()
                        if recovery_active:
                            fall_safety.end_recovery()
                            recovery_active = False
                        reset_until = started + args.stop_ms / 1000.0
                    resetting = started < reset_until
                    if resetting:
                        forward = turn = side = 0.0
                        locomotion_requested = False
                    elif state.getup and not previous_getup:
                        protected_pose = backend.current_pose
                        fall_safety.begin_recovery()
                        recovery_active = True
                        engine.reset()
                        arm_dance.reset()
                        squat.reset()
                        getup.start(protected_pose)
                        previous_fall = False
                        print("[main] G: starting stand-up.")
                    elif (
                        state.squat
                        and not getup.running
                        and not fall_safety.active
                    ):
                        engine.reset()
                        arm_dance.reset()
                        squat.toggle()
                        forward = turn = side = 0.0
                        locomotion_requested = False
                    elif (
                        state.dance
                        and not previous_dance
                        and not getup.running
                        and not squat.active
                        and not fall_safety.active
                    ):
                        arm_dance.toggle()
                        engine.reset()
                    previous_getup = state.getup
                    previous_dance = state.dance
                    gait = stationary_gait()
                    if getup.running:
                        reading = snapshot.imu if snapshot is not None else None
                        reference = fall_safety.reference
                        tilt = None
                        if reading is not None and reference is not None and reading.balance_ready(
                            args.imu_min_gyro_cal, args.imu_min_accel_cal,
                        ):
                            tilt = (
                                angle_error_deg(reading.roll_deg, reference[0]),
                                angle_error_deg(reading.pitch_deg, reference[1]),
                            )
                        pose = getup.update(tilt)
                        releasing = getup.label == "release-arms" and pose != getup.steps[-2].pose
                        if recovery_active and (releasing or getup.blocked or not getup.running):
                            fall_safety.end_recovery()
                            recovery_active = False
                        status = f"GET-UP: {getup.label.upper()}" if getup.running else "STANDING"
                        gait = stationary_gait(getup.label)
                    elif resetting:
                        pose = dict(STANDING)
                        status = "Reset / standing"
                    elif squat.active:
                        pose = squat.update()
                        status = squat.phase.upper()
                        gait = stationary_gait(squat.phase)
                        gait["crouch_mm"] = squat.depth_mm
                    elif arm_dance.running and not locomotion_requested:
                        pose = arm_dance.update()
                        status = f"ARM DANCE: {arm_dance.section.upper()}"
                        gait = stationary_gait(f"dance-{arm_dance.section}")
                    else:
                        if arm_dance.running:
                            arm_dance.reset()
                        pose = engine.update(forward, turn_cmd=turn, side_cmd=side)
                        pose[25] = round(STANDING[25] + args.head_pan_direction * args.head_pan_pwm * (
                            1 if turn > 0.0 else -1 if turn < 0.0 else 0
                        ))
                        gait = engine.telemetry_snapshot()
                        idle_ready = engine.is_idle_ready()
                        directions = []
                        if forward:
                            directions.append("FORWARD" if forward > 0 else "BACKWARD")
                        if turn:
                            directions.append("TURN LEFT" if turn > 0 else "TURN RIGHT")
                        if side:
                            directions.append("SIDE LEFT" if side > 0 else "SIDE RIGHT")
                        status = " + ".join(directions) if directions else (
                            "SETTLING" if not idle_ready else "WALK READY"
                        )

                    fall_active = fall_safety.active
                    if fall_active:
                        if not previous_fall:
                            engine.reset()
                            arm_dance.reset()
                            squat.reset()
                            getup.reset()
                            if recovery_active:
                                fall_safety.end_recovery()
                                recovery_active = False
                        pose = backend.current_pose
                        status = "FALL DETECTED - ARMS FORWARD"
                        if resetting:
                            status += " | RESET BLOCKED BY FALL"
                        gait = stationary_gait("fall")
                    elif previous_fall:
                        engine.reset()
                        squat.reset()
                        pose = dict(STANDING)
                        status = "UPRIGHT - STANDING"
                        gait = stationary_gait()
                    previous_fall = fall_active

                    depth = snapshot.depth if snapshot is not None else None
                    blocked, distance = obstacle_guard.update(depth)
                    if depth is not None and blocked and not fall_active:
                        status += f" | TOF NEAR {distance} MM (MANUAL)"
                    hold_standing = not fall_active and (
                        reset_requested or pose == STANDING and backend.current_pose != STANDING
                    )
                    duration = args.stop_ms if hold_standing else args.update_ms
                    if not resetting or reset_requested or fall_active:
                        backend.send(pose, duration_ms=duration, force=hold_standing)
                    dashboard.publish(
                        pose=backend.current_pose,
                        gait=gait,
                        sensor_snapshot=snapshot,
                        status=status,
                        active=(
                            fall_active
                            or getup.running
                            or squat.active
                            or arm_dance.running
                            or not engine.is_idle_ready()
                        ),
                        balance_status=fall_safety.status,
                    )
                    dashboard.set_runtime("manual", status)
                    remaining = args.update_ms / 1000.0 - (time.monotonic() - started)
                    if remaining > 0:
                        time.sleep(remaining)
            finally:
                if recovery_active:
                    fall_safety.end_recovery()
                exit_pose = backend.current_pose if fall_safety.active or getup.running else STANDING
                backend.send(exit_pose, duration_ms=args.stop_ms, force=True)
                time.sleep(args.stop_ms / 1000.0)
    finally:
        dashboard.set_runtime("idle", "Manual control stopped")
        print("[main] Manual web control exited.")


def main() -> None:
    from .camera import HeadlessCamera
    from .fall_safety import FallSafety, PriorityBackend
    from .gait_anomaly import GaitHealthMonitor
    from .gait_dashboard import GaitDashboard, stationary_gait
    from .sensors import RobotSensorHub
    from .walking_engine import STANDING

    args = Config()
    project_root = Path(__file__).resolve().parent.parent
    camera = HeadlessCamera(
        width=args.vision_camera_width,
        height=args.vision_camera_height,
        fps=args.vision_fps,
    )
    health_monitor = GaitHealthMonitor(
        enabled=args.gait_anomaly_enabled,
        model_path=project_root / args.gait_anomaly_model,
        history_path=project_root / args.gait_anomaly_history,
        window_s=args.gait_anomaly_window_s,
        min_samples=args.gait_anomaly_min_samples,
        warning_windows=args.gait_anomaly_warning_windows,
    )
    dashboard = GaitDashboard(
        host=args.gait_dashboard_host,
        port=args.gait_dashboard_port,
        stream_hz=args.gait_dashboard_stream_hz,
        command_timeout_s=args.gait_dashboard_command_timeout_s,
        camera=camera,
        health_monitor=health_monitor,
    )
    dashboard.start()
    camera.start()
    sensor_hub = None
    if args.sensor_feedback or args.fall_detection_enabled:
        sensor_hub = RobotSensorHub(
            port=args.sensor_port,
            baudrate=args.sensor_baudrate,
            timeout_s=args.sensor_timeout_s,
            depth_timeout_s=args.sensor_depth_timeout_s,
            use_imu=args.sensor_use_imu or args.fall_detection_enabled,
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
            print("[main] ESP32 sensor connection started in background.")
        except Exception as exc:
            sensor_hub = None
            print(f"[main] Sensors unavailable: {exc}. Web control remains available.")

    raw_backend = make_backend(
        mode=args.backend,
        port=args.port,
        baudrate=args.baudrate,
        csv_path=args.csv,
    )

    try:
        with ExitStack() as stack:
            stack.enter_context(raw_backend)
            backend = PriorityBackend(raw_backend, args.fall_arm_forward_pwm)
            backend.send(STANDING, duration_ms=1200, force=True)
            time.sleep(1.2)
            fall_safety = FallSafety(args, sensor_hub, backend)
            fall_safety.start()
            stack.callback(fall_safety.close)
            dashboard.publish(
                pose=STANDING,
                gait=stationary_gait(),
                sensor_snapshot=sensor_hub.read() if sensor_hub is not None else None,
                status="WEB CONTROL READY",
                active=False,
                balance_status=fall_safety.status,
            )
            print("[main] Open the dashboard from a laptop on the same LAN, then enable control.")

            last_idle_publish = 0.0
            while True:
                state = dashboard.control_payload()
                if not state["armed"]:
                    now = time.monotonic()
                    if now - last_idle_publish >= 0.10:
                        status = "FALL DETECTED - ARMS FORWARD" if fall_safety.active else "WEB CONTROL READY"
                        dashboard.publish(
                            pose=backend.current_pose,
                            gait=stationary_gait("fall" if fall_safety.active else "idle"),
                            sensor_snapshot=sensor_hub.read() if sensor_hub is not None else None,
                            status=status,
                            active=fall_safety.active,
                            balance_status=fall_safety.status,
                        )
                        dashboard.set_runtime(state["mode"], status)
                        last_idle_publish = now
                    time.sleep(0.05)
                    continue
                try:
                    if state["mode"] == "manual":
                        run_manual(
                            args, dashboard, backend, sensor_hub, fall_safety,
                        )
                    elif state["mode"] == "terrain":
                        from .stair_main import run_terrain_auto

                        run_terrain_auto(
                            args, dashboard, camera, backend, sensor_hub, fall_safety,
                        )
                    elif state["mode"] == "follow":
                        from .follow_main import run_follow

                        run_follow(
                            args, dashboard, camera, backend, sensor_hub, fall_safety,
                        )
                except Exception as exc:
                    dashboard.disarm(f"{state['mode']} unavailable: {exc}")
                    print(f"[main] {state['mode']} unavailable: {exc}")
    except KeyboardInterrupt:
        print("\n[main] Ctrl+C received. Stopping web control.")
    finally:
        dashboard.disarm("Server stopped")
        dashboard.close()
        camera.close()
        if sensor_hub is not None:
            sensor_hub.close()

if __name__ == "__main__":
    main()
