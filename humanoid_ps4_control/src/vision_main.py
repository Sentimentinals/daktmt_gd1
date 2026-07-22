from __future__ import annotations

import time

from .backends import make_backend
from .config import Config, STANDING
from .vision_control import ARM_IDS, VisionBodyController


def run_vision(args: Config) -> None:
    """Run bounded full-body mimic mode and return when Options/O is pressed."""
    try:
        import cv2
        import pygame
        from picamera2 import Picamera2
        from .movenet_pose import MoveNetPoseEstimator
    except ImportError as exc:
        raise RuntimeError(
            f"Camera Mimic dependency missing: {exc.name}. Install Picamera2/OpenCV from apt "
            "and ai-edge-litert from requirements-vision.txt."
        ) from exc

    from .ps4_pygame import PS4Reader

    cv2.setNumThreads(1)
    estimator = MoveNetPoseEstimator(args.vision_model_path, num_threads=args.vision_threads)
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
        leg_lift_threshold_ratio=args.vision_leg_lift_threshold_ratio,
        min_body_scale=args.vision_min_body_scale,
    )
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
    backend = make_backend(mode=args.backend, port=args.port, baudrate=args.baudrate, csv_path=args.csv)
    armed = False
    previous_toggle = False
    last_pose = dict(STANDING)
    active_lifted_leg = None
    frame_duration_ms = max(args.update_ms, round(1000.0 / max(1, args.vision_fps)))

    try:
        camera.start()
        time.sleep(0.5)
        with backend:
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
                    landmarks = None
                    if armed:
                        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        landmarks = estimator.infer(rgb)
                    body_pose = controller.update(landmarks, time.monotonic(), armed=armed)
                    requested_leg = controller.lifted_leg if armed and controller.tracked else None
                    if requested_leg != active_lifted_leg:
                        single_support.stop()
                        active_lifted_leg = None
                        if requested_leg is not None:
                            support_leg = "right" if requested_leg == "left" else "left"
                            single_support.start(support_leg=support_leg, current_pose=last_pose)
                            active_lifted_leg = requested_leg

                    single_support.lift_height = args.vision_leg_lift_height_mm

                    if single_support.running:
                        pose = single_support.update()
                        for sid in (*ARM_IDS, 16):
                            pose[sid] = body_pose[sid]
                    else:
                        pose = body_pose
                    pose = clamp_pose_rate(last_pose, pose, args.vision_max_pwm_per_s / args.vision_fps)
                    backend.send(pose, duration_ms=frame_duration_ms)
                    last_pose = dict(pose)

                    if landmarks is not None:
                        estimator.draw(frame, landmarks, args.vision_confidence)
                    preview = cv2.cvtColor(cv2.flip(frame, 1), cv2.COLOR_BGR2RGB)
                    surface = pygame.surfarray.make_surface(preview.swapaxes(0, 1))
                    screen.blit(surface, (0, 0))
                    full_body = all(
                        part in controller.visible_parts
                        for part in ("left_arm", "right_arm", "legs")
                    )
                    status = (
                        "FULL BODY"
                        if armed and controller.tracked and full_body
                        else (
                            "TRACKING"
                            if armed and controller.tracked
                            else (
                                "SEARCHING" if armed else "MIMIC OFF"
                            )
                        )
                    )
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
        reader.quit()
