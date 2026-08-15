from __future__ import annotations

import time

from .backends import make_backend
from .config import Config, STANDING
from .keyboard_input import KeyboardReader, LiveCameraPreview
from .walking_engine import AdaptiveSquatEngine


def run_pickup(args: Config) -> None:
    reader = KeyboardReader(
        poll_rate_hz=max(10, round(1000.0 / args.update_ms)),
        caption="Humanoid Pick Up Positioning",
        controls="Hold R to squat, C stop, O/Esc menu, Q quit.",
    )
    camera = LiveCameraPreview(
        width=args.vision_camera_width,
        height=args.vision_camera_height,
        fps=args.vision_fps,
        detector=None,
    )
    squat = AdaptiveSquatEngine(
        dt=args.update_ms / 1000.0,
        min_depth_mm=args.squat_min_depth_mm,
        max_depth_mm=args.squat_max_depth_mm,
        depth_rate_mm_s=args.squat_depth_rate_mm_s,
        max_pwm_per_frame=args.squat_max_pwm_per_frame,
    )
    backend = make_backend(
        mode=args.backend,
        port=args.port,
        baudrate=args.baudrate,
        csv_path=args.csv,
    )
    previous_stop = False

    try:
        with backend:
            backend.send(STANDING, duration_ms=900, force=True)
            time.sleep(0.9)
            try:
                if not reader.init():
                    raise RuntimeError("pygame keyboard control is unavailable")
                camera.start()
                squat.reset(STANDING)
                for keyboard in reader.poll():
                    if keyboard.quit or keyboard.menu:
                        break

                    stop_pressed = keyboard.stop
                    if stop_pressed and not previous_stop:
                        squat.reset(STANDING)
                        backend.send(STANDING, duration_ms=args.stop_ms, force=True)
                        print("[pickup] Stopped at STANDING.")
                    previous_stop = stop_pressed

                    ratio = 1.0 if keyboard.squat and not stop_pressed else 0.0
                    pose = squat.update(ratio)
                    backend.send(pose, duration_ms=args.update_ms)
                    depth_ratio = squat.depth_mm / max(1.0, args.squat_max_depth_mm)
                    status = (
                        f"PICK-UP POSITION {round(depth_ratio * 100)}%"
                        if ratio > 0.0 or not squat.is_idle()
                        else "PICK-UP READY"
                    )
                    camera.render(status)
            finally:
                try:
                    backend.send(STANDING, duration_ms=args.stop_ms, force=True)
                    time.sleep(args.stop_ms / 1000.0)
                except Exception as exc:
                    print(f"[pickup] Failed to return to STANDING: {exc}")
    finally:
        camera.close()
        reader.quit()
        print("[pickup] Pick Up Positioning exited.")
