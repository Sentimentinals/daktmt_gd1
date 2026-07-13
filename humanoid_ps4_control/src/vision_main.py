from __future__ import annotations

import time

from .backends import make_backend
from .config import Config, STANDING
from .vision_control import VisionArmController


def run_vision(args: Config) -> None:
    """Run standing upper-body mimic mode and return when Options/O is pressed."""
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
    controller = VisionArmController(
        confidence=args.vision_confidence,
        lost_timeout_s=args.vision_lost_timeout_s,
        smooth_tau_s=args.vision_smooth_tau_s,
        max_pwm_per_s=args.vision_max_pwm_per_s,
        lift_pwm=args.vision_lift_pwm,
        shoulder_pwm=args.vision_shoulder_pwm,
        elbow_pwm=args.vision_elbow_pwm,
    )
    backend = make_backend(mode=args.backend, port=args.port, baudrate=args.baudrate, csv_path=args.csv)
    pose_api = mp.solutions.pose
    drawing = mp.solutions.drawing_utils
    armed = False
    previous_toggle = False

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
                            backend.send(STANDING, duration_ms=500, force=True)
                        print("[vision] Mimic ON." if armed else "[vision] Mimic OFF.")
                    previous_toggle = toggle

                    frame = camera.capture_array("main")
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    result = estimator.process(rgb)
                    world = result.pose_world_landmarks.landmark if result.pose_world_landmarks else None
                    pose = controller.update(world, time.monotonic(), armed=armed)
                    backend.send(pose, duration_ms=args.update_ms)

                    if result.pose_landmarks:
                        drawing.draw_landmarks(frame, result.pose_landmarks, pose_api.POSE_CONNECTIONS)
                    preview = cv2.cvtColor(cv2.flip(frame, 1), cv2.COLOR_BGR2RGB)
                    surface = pygame.surfarray.make_surface(preview.swapaxes(0, 1))
                    screen.blit(surface, (0, 0))
                    status = "MIMIC ON" if armed and controller.tracked else ("SEARCHING" if armed else "MIMIC OFF")
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
