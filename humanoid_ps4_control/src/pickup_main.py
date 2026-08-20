from __future__ import annotations

import time

from .backends import make_backend
from .config import Config, STANDING
from .gait_dashboard import stationary_gait
from .walking_engine import AdaptiveSquatEngine


def run_pickup(args: Config, dashboard, camera_ready: bool) -> None:
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
                squat.reset(STANDING)
                dashboard.set_runtime("pickup", "Pickup positioning ready")
                while True:
                    loop_started = time.monotonic()
                    control = dashboard.control_state()
                    if not control.armed or control.mode != "pickup":
                        break

                    stop_pressed = control.stop
                    if stop_pressed and not previous_stop:
                        squat.reset(STANDING)
                        backend.send(STANDING, duration_ms=args.stop_ms, force=True)
                        print("[pickup] Stopped at STANDING.")
                    previous_stop = stop_pressed

                    ratio = 1.0 if control.squat and not stop_pressed else 0.0
                    pose = squat.update(ratio)
                    backend.send(pose, duration_ms=args.update_ms)
                    depth_ratio = squat.depth_mm / max(1.0, args.squat_max_depth_mm)
                    status = (
                        f"PICK-UP POSITION {round(depth_ratio * 100)}%"
                        if ratio > 0.0 or not squat.is_idle()
                        else "PICK-UP READY"
                    )
                    dashboard.publish(
                        pose=pose,
                        gait=stationary_gait("squat" if not squat.is_idle() else "idle"),
                        sensor_snapshot=None,
                        status=status,
                        active=not squat.is_idle(),
                        camera_ready=camera_ready,
                        balance_status="PICKUP",
                    )
                    dashboard.set_runtime("pickup", status)
                    remaining = args.update_ms / 1000.0 - (time.monotonic() - loop_started)
                    if remaining > 0.0:
                        time.sleep(remaining)
            finally:
                try:
                    backend.send(STANDING, duration_ms=args.stop_ms, force=True)
                    time.sleep(args.stop_ms / 1000.0)
                except Exception as exc:
                    print(f"[pickup] Failed to return to STANDING: {exc}")
    finally:
        dashboard.set_runtime("idle", "Pickup positioning stopped")
        print("[pickup] Pick Up Positioning exited.")
