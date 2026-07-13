from __future__ import annotations

import time

from .backends import make_backend
from .config import Config, STANDING
from .vision_control import ARM_IDS, VisionBodyController


def run_vision(args: Config) -> None:
    """Run bounded full-body mimic mode and return when Options/O is pressed."""
    try:
        import cv2
        import mediapipe as mp
        import pygame
        from picamera2 import Picamera2
    except ImportError as exc:
        raise RuntimeError(
            "Camera Mimic dependencies are missing. Install Picamera2/OpenCV from apt "
            "and mediapipe==0.10.18 from requirements-vision.txt."
        ) from exc

    from .ps4_pygame import PS4Reader

    reader = PS4Reader(
        joystick_index=args.joystick_index,
        fallback_keys=True,
        poll_rate_hz=args.vision_fps,
        deadzone=args.input_deadzone,
    )
    reader.init()
    screen = pygame.display.set_mode((args.vision_camera_width, args.vision_camera_height))
    pygame.display.set_caption("Camera Mimic")
    font = pygame.font.Font(None, 28)

    camera = Picamera2()
    camera.configure(
        camera.create_preview_configuration(
            main={
                "format": "RGB888",
                "size": (args.vision_camera_width, args.vision_camera_height),
            },
            controls={"FrameRate": args.vision_fps},
        )
    )
    controller = VisionBodyController(
        confidence=args.vision_confidence,
        lost_timeout_s=args.vision_lost_timeout_s,
        smooth_tau_s=args.vision_smooth_tau_s,
        max_pwm_per_s=args.vision_max_pwm_per_s,
        lift_pwm=args.vision_lift_pwm,
        shoulder_pwm=args.vision_shoulder_pwm,
        elbow_pwm=args.vision_elbow_pwm,
        head_pwm=args.vision_head_pwm,
        squat_deg=args.vision_squat_deg,
        leg_lift_threshold_m=args.vision_leg_lift_threshold_m,
    )
    from .sensors import RobotSensorHub
    from .walking_engine import SingleSupportTestEngine, clamp_pose_rate

    single_support = SingleSupportTestEngine(
        dt=1.0 / args.vision_fps,
        lift_height=args.vision_leg_lift_height_mm,
        zmp_support_ratio=args.zmp_support_ratio,
        hip_abduct_gain=args.hip_abduct_gain,
        ankle_roll_gain=args.ankle_roll_gain,
        arm_pwm=0,
        ramp_s=args.single_support_ramp_s,
    )
    sensor_hub = None
    if args.sensor_feedback and args.sensor_use_fsr:
        try:
            sensor_hub = RobotSensorHub(
                port=args.sensor_port,
                baudrate=args.sensor_baudrate,
                timeout_s=args.sensor_timeout_s,
                use_imu=False,
                use_fsr=True,
                fsr_invert=args.fsr_invert,
                fsr_filter_alpha=args.fsr_filter_alpha,
                fsr_left_zero_raw=args.fsr_left_zero_raw,
                fsr_left_full_raw=args.fsr_left_full_raw,
                fsr_right_zero_raw=args.fsr_right_zero_raw,
                fsr_right_full_raw=args.fsr_right_full_raw,
            )
            sensor_hub.open()
        except Exception as exc:
            sensor_hub = None
            print(f"[vision] FSR unavailable; leg lifting disabled: {exc}")
    backend = make_backend(mode=args.backend, port=args.port, baudrate=args.baudrate, csv_path=args.csv)
    pose_api = mp.solutions.pose
    drawing = mp.solutions.drawing_utils
    armed = False
    previous_toggle = False
    last_pose = dict(STANDING)
    active_lifted_leg = None

    try:
        camera.start()
        time.sleep(0.5)
        with pose_api.Pose(
            static_image_mode=False,
            model_complexity=0,
            smooth_landmarks=True,
            min_detection_confidence=args.vision_confidence,
            min_tracking_confidence=args.vision_confidence,
        ) as estimator, backend:
            backend.send(STANDING, duration_ms=600, force=True)
            try:
                for state in reader.poll():
                    if state.quit or state.button(reader.BTN_OPTIONS):
                        print("[vision] Returning to function menu.")
                        break

                    toggle = state.button(reader.BTN_SQUARE)
                    if toggle and not previous_toggle:
                        armed = not armed
                        if not armed:
                            controller.reset()
                            single_support.stop()
                            active_lifted_leg = None
                            backend.send(STANDING, duration_ms=500, force=True)
                        print("[vision] Mimic ON." if armed else "[vision] Mimic OFF.")
                    previous_toggle = toggle

                    frame = camera.capture_array("main")
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    result = estimator.process(rgb)
                    world = result.pose_world_landmarks.landmark if result.pose_world_landmarks else None
                    body_pose = controller.update(world, time.monotonic(), armed=armed)
                    requested_leg = controller.lifted_leg if armed and controller.tracked else None
                    if requested_leg != active_lifted_leg:
                        single_support.stop()
                        active_lifted_leg = None
                        if requested_leg is not None:
                            support_leg = "right" if requested_leg == "left" else "left"
                            single_support.start(support_leg=support_leg, current_pose=last_pose)
                            active_lifted_leg = requested_leg

                    leg_ready = False
                    if single_support.running and sensor_hub is not None:
                        snapshot = sensor_hub.read()
                        load = snapshot.foot_load
                        if load is not None and load.total >= args.fsr_min_total_load:
                            if single_support.support_leg == "left":
                                leg_ready = load.left_ratio >= args.fsr_support_ratio
                            else:
                                leg_ready = load.right_ratio >= args.fsr_support_ratio
                    single_support.lift_height = args.vision_leg_lift_height_mm if leg_ready else 0.0

                    if single_support.running:
                        pose = single_support.update()
                        for sid in (*ARM_IDS, 16):
                            pose[sid] = body_pose[sid]
                    else:
                        pose = body_pose
                    pose = clamp_pose_rate(last_pose, pose, args.vision_max_pwm_per_s / args.vision_fps)
                    backend.send(pose, duration_ms=args.update_ms)
                    last_pose = dict(pose)

                    if result.pose_landmarks:
                        drawing.draw_landmarks(frame, result.pose_landmarks, pose_api.POSE_CONNECTIONS)
                    preview = cv2.cvtColor(cv2.flip(frame, 1), cv2.COLOR_BGR2RGB)
                    surface = pygame.surfarray.make_surface(preview.swapaxes(0, 1))
                    screen.blit(surface, (0, 0))
                    if single_support.running and not leg_ready:
                        status = "SHIFTING WEIGHT"
                    else:
                        status = "FULL BODY" if armed and controller.tracked else ("SEARCHING" if armed else "MIMIC OFF")
                    color = (58, 210, 148) if armed and controller.tracked else (245, 190, 72)
                    label = font.render(status, True, color)
                    background = pygame.Surface((label.get_width() + 24, label.get_height() + 12), pygame.SRCALPHA)
                    background.fill((10, 14, 18, 205))
                    screen.blit(background, (14, 14))
                    screen.blit(label, (26, 20))
                    pygame.display.flip()
            finally:
                backend.send(STANDING, duration_ms=args.stop_ms, force=True)
                time.sleep(args.stop_ms / 1000.0)
    except KeyboardInterrupt:
        print("\n[vision] Interrupted. Returning to function menu.")
    finally:
        try:
            camera.stop()
        except Exception:
            pass
        try:
            camera.close()
        except Exception:
            pass
        if sensor_hub is not None:
            sensor_hub.close()
        reader.quit()
