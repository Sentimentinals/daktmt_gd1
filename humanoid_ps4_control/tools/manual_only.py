from __future__ import annotations

import argparse
import time

from src.backends import MockBackend
from src.config import Config, STANDING
from src.fall_safety import PriorityBackend
from src.gait_dashboard import GaitDashboard, stationary_gait
from src.main import run_manual


class ManualOnlySafety:
    active = False
    reference = None
    status = "FALL OFF - MANUAL TEST"

    def begin_recovery(self) -> None:
        pass

    def end_recovery(self) -> None:
        pass


def main() -> None:
    parser = argparse.ArgumentParser(description="Manual dashboard without hardware, sensors, or camera.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    options = parser.parse_args()

    config = Config()
    dashboard = GaitDashboard(
        host=options.host,
        port=options.port,
        stream_hz=config.gait_dashboard_stream_hz,
        command_timeout_s=config.gait_dashboard_command_timeout_s,
    )
    raw_backend = MockBackend(verbose=False)
    backend = PriorityBackend(raw_backend, config.fall_arm_forward_pwm)
    safety = ManualOnlySafety()
    dashboard.start()
    try:
        with raw_backend:
            dashboard.publish(
                pose=STANDING,
                gait=stationary_gait(),
                sensor_snapshot=None,
                status="MANUAL TEST READY - NO HARDWARE",
                active=False,
                camera_ready=False,
                balance_status=safety.status,
            )
            while True:
                state = dashboard.control_state()
                if state.armed and state.mode == "manual":
                    run_manual(config, dashboard, False, backend, None, safety)
                    continue
                if state.armed:
                    dashboard.disarm("Manual-only test")
                dashboard.publish(
                    pose=backend.current_pose,
                    gait=stationary_gait(),
                    sensor_snapshot=None,
                    status="MANUAL TEST READY - NO HARDWARE",
                    active=False,
                    camera_ready=False,
                    balance_status=safety.status,
                )
                dashboard.set_runtime("manual", "MANUAL TEST READY - NO HARDWARE")
                time.sleep(0.08)
    except KeyboardInterrupt:
        pass
    finally:
        dashboard.disarm("Manual-only test stopped")
        dashboard.close()


if __name__ == "__main__":
    main()
