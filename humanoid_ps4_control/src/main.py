from __future__ import annotations

from contextlib import ExitStack
import time

from .backends import make_backend
from .config import Config


def run_manual(
    args: Config,
    dashboard,
    camera_ready: bool,
    backend,
    sensor_hub,
    fall_safety,
) -> None:
    from .arm_dance import ArmDanceEngine
    from .balance import angle_error_deg
    from .gait_dashboard import stationary_gait
    from .getup import GetupEngine
    from .sensors import DepthObstacleGuard
    from .walking_engine import DynamicWalkingEngine, STANDING

    engine = DynamicWalkingEngine(
        dt=args.update_ms / 1000.0,
        t_step=args.t_step,
        t_dbl=args.t_dbl,
        max_step_len=args.walk_step_length_mm,
        max_turn_step_len=args.max_turn_step_len,
        max_side_step_len=args.max_side_step_len,
        step_height=args.walk_step_height_mm,
        crouch_depth_mm=args.walk_crouch_depth_mm,
        forward_lean_deg=args.walk_forward_lean_deg,
        zmp_support_ratio=args.zmp_support_ratio,
        ankle_roll_gain=args.ankle_roll_gain,
        landing_gap_mm=0.0,
        lift_start_phase=args.walk_lift_start_phase,
        swing_advance_end_phase=args.walk_swing_advance_end_phase,
        lift_end_phase=args.walk_lift_end_phase,
        landing_roll_release_start=args.walk_landing_roll_release_start,
        arm_swing_pwm=args.arm_swing_pwm,
        arm_right_dir=args.arm_right_dir,
        arm_left_dir=args.arm_left_dir,
        crouch_transition_s=args.walk_crouch_transition_s,
    )
    arm_dance = ArmDanceEngine(
        dt=args.update_ms / 1000.0,
        period_s=args.dance_period,
        transition_s=args.dance_transition,
        shoulder_pwm=args.dance_shoulder_pwm,
        elbow_pwm=args.dance_elbow_pwm,
        lift_pwm=args.dance_lift_pwm,
        head_pwm=args.dance_head_pwm,
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
    dashboard.set_runtime("manual", "Manual control ready")

    try:
        with backend:
            try:
                while True:
                    started = time.monotonic()
                    state = dashboard.control_state()
                    if not state.armed or state.mode != "manual":
                        break
                    snapshot = sensor_hub.read() if sensor_hub is not None else None
                    forward = state.forward * args.walk_speed
                    turn = state.turn * args.turn_speed
                    side = state.side * args.side_speed
                    reset_requested = state.stop or state.reset
                    if reset_requested and not fall_safety.active:
                        engine.reset()
                        arm_dance.reset()
                        getup.reset()
                        if recovery_active:
                            fall_safety.end_recovery()
                            recovery_active = False
                    elif state.getup and not previous_getup:
                        protected_pose = backend.current_pose
                        fall_safety.begin_recovery()
                        recovery_active = True
                        engine.reset()
                        arm_dance.reset()
                        getup.start(protected_pose)
                        previous_fall = False
                        print("[main] G: starting stand-up.")
                    elif state.dance and not previous_dance and not getup.running and not fall_safety.active:
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
                    elif reset_requested:
                        pose = dict(STANDING)
                        status = "Stop / standing" if state.stop else "Reset / standing"
                    elif arm_dance.running:
                        pose = arm_dance.update()
                        status = "ARM DANCE"
                        gait = stationary_gait("dance")
                    else:
                        pose = engine.update(forward, turn_cmd=turn, side_cmd=side)
                        pose[25] = round(STANDING[25] + args.head_pan_direction * args.head_pan_pwm * (
                            1 if turn > 0.0 else -1 if turn < 0.0 else 0
                        ))
                        gait = engine.telemetry_snapshot()
                        directions = []
                        if forward:
                            directions.append("FORWARD" if forward > 0 else "BACKWARD")
                        if turn:
                            directions.append("TURN LEFT" if turn > 0 else "TURN RIGHT")
                        if side:
                            directions.append("SIDE LEFT" if side > 0 else "SIDE RIGHT")
                        status = " + ".join(directions) if directions else (
                            "SETTLING" if not engine.is_idle_ready() else "WALK READY"
                        )

                    fall_active = fall_safety.active
                    if fall_active:
                        if not previous_fall:
                            engine.reset()
                            arm_dance.reset()
                            getup.reset()
                            if recovery_active:
                                fall_safety.end_recovery()
                                recovery_active = False
                        pose = backend.current_pose
                        status = "FALL DETECTED - ARMS FORWARD"
                        gait = stationary_gait("fall")
                    elif previous_fall:
                        engine.reset()
                        pose = dict(STANDING)
                        status = "UPRIGHT - STANDING"
                        gait = stationary_gait()
                    previous_fall = fall_active

                    depth = snapshot.depth if snapshot is not None else None
                    blocked, distance = obstacle_guard.update(depth)
                    if depth is not None and blocked and not fall_active:
                        status += f" | TOF NEAR {distance} MM (MANUAL)"
                    duration = args.stop_ms if reset_requested and not fall_active else args.update_ms
                    backend.send(pose, duration_ms=duration, force=reset_requested)
                    dashboard.publish(
                        pose=backend.current_pose,
                        gait=gait,
                        sensor_snapshot=snapshot,
                        status=status,
                        active=fall_active or getup.running or arm_dance.running or not engine.is_idle_ready(),
                        camera_ready=camera_ready,
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
    from .gait_dashboard import GaitDashboard, stationary_gait
    from .sensors import RobotSensorHub
    from .walking_engine import STANDING

    args = Config()
    camera = HeadlessCamera(
        width=args.vision_camera_width,
        height=args.vision_camera_height,
        fps=args.vision_fps,
    )
    dashboard = GaitDashboard(
        host=args.gait_dashboard_host,
        port=args.gait_dashboard_port,
        stream_hz=args.gait_dashboard_stream_hz,
        command_timeout_s=args.gait_dashboard_command_timeout_s,
        camera=camera,
    )
    dashboard.start()
    camera_ready = camera.start()
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
                camera_ready=camera_ready,
                balance_status=fall_safety.status,
            )
            print("[main] Open the dashboard from a laptop on the same LAN, then enable control.")

            last_idle_publish = 0.0
            while True:
                state = dashboard.control_state()
                if not state.armed:
                    now = time.monotonic()
                    if now - last_idle_publish >= 0.10:
                        status = "FALL DETECTED - ARMS FORWARD" if fall_safety.active else "WEB CONTROL READY"
                        dashboard.publish(
                            pose=backend.current_pose,
                            gait=stationary_gait("fall" if fall_safety.active else "idle"),
                            sensor_snapshot=sensor_hub.read() if sensor_hub is not None else None,
                            status=status,
                            active=fall_safety.active,
                            camera_ready=camera_ready,
                            balance_status=fall_safety.status,
                        )
                        dashboard.set_runtime(state.mode, status)
                        last_idle_publish = now
                    time.sleep(0.05)
                    continue
                try:
                    if state.mode == "manual":
                        run_manual(
                            args, dashboard, camera_ready, backend, sensor_hub, fall_safety,
                        )
                    elif state.mode == "terrain":
                        from .stair_main import run_terrain_auto

                        run_terrain_auto(
                            args, dashboard, camera, camera_ready,
                            backend, sensor_hub, fall_safety,
                        )
                    elif state.mode == "follow":
                        from .follow_main import run_follow

                        run_follow(
                            args, dashboard, camera, camera_ready,
                            backend, sensor_hub, fall_safety,
                        )
                except Exception as exc:
                    dashboard.disarm(f"{state.mode} unavailable: {exc}")
                    print(f"[main] {state.mode} unavailable: {exc}")
    except KeyboardInterrupt:
        print("\n[main] Ctrl+C received. Stopping web control.")
    finally:
        if sensor_hub is not None:
            sensor_hub.close()
        dashboard.disarm("Server stopped")
        camera.close()
        dashboard.close()

if __name__ == "__main__":
    main()
