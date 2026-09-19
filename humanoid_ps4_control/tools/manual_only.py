from __future__ import annotations

import argparse
import json
import mimetypes
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

from src.backends import MockBackend, SerialRTBackend
from src.config import Config, STANDING
from src.walking_engine import DynamicWalkingEngine


STATIC_ROOT = Path(__file__).resolve().parent.parent / "web" / "manual_test"


class ManualController:
    def __init__(self, config: Config, backend, port_label: str) -> None:
        self.config = config
        self.backend = backend
        self.port_label = port_label
        self.engine = DynamicWalkingEngine(
            dt=config.update_ms / 1000.0,
            t_step=config.t_step,
            t_dbl=config.t_dbl,
            max_step_len=config.walk_step_length_mm,
            max_turn_step_len=config.max_turn_step_len,
            max_side_step_len=config.max_side_step_len,
            step_height=config.walk_step_height_mm,
            crouch_depth_mm=config.walk_crouch_depth_mm,
            forward_lean_deg=config.walk_forward_lean_deg,
            zmp_support_ratio=config.zmp_support_ratio,
            ankle_roll_gain=config.ankle_roll_gain,
            landing_gap_mm=0.0,
            lift_start_phase=config.walk_lift_start_phase,
            swing_advance_end_phase=config.walk_swing_advance_end_phase,
            lift_end_phase=config.walk_lift_end_phase,
            landing_roll_release_start=config.walk_landing_roll_release_start,
            arm_swing_pwm=config.arm_swing_pwm,
            arm_right_dir=config.arm_right_dir,
            arm_left_dir=config.arm_left_dir,
            crouch_transition_s=config.walk_crouch_transition_s,
        )
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._armed = False
        self._axes = {"forward": 0.0, "turn": 0.0, "side": 0.0}
        self._last_sequence = -1
        self._last_command_at = 0.0
        self._reset_requested = False
        self._pose = dict(STANDING)
        self._gait = self.engine.telemetry_snapshot()
        self._status = "OUTPUT DISABLED"
        self._frames = 0
        self._error = ""

    @staticmethod
    def _axis(value: object) -> float:
        try:
            return max(-1.0, min(1.0, float(value)))
        except (TypeError, ValueError):
            return 0.0

    def start(self) -> None:
        self.backend.open()
        self.backend.send(STANDING, duration_ms=800, force=True)
        time.sleep(0.8)
        self._thread = threading.Thread(target=self._run, name="manual-servo", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)
        try:
            self.backend.send(STANDING, duration_ms=400, force=True)
        finally:
            self.backend.close()

    def apply(self, request: dict[str, object]) -> dict[str, object]:
        try:
            sequence = int(request.get("sequence", -1))
        except (TypeError, ValueError):
            sequence = -1
        with self._lock:
            if sequence <= self._last_sequence:
                return self._snapshot_locked()
            was_moving = any(abs(value) > 0.0 for value in self._axes.values())
            self._last_sequence = sequence
            self._last_command_at = time.monotonic()
            self._armed = bool(request.get("armed", self._armed))
            axes = request.get("axes", {})
            if self._armed and isinstance(axes, dict):
                self._axes = {
                    name: self._axis(axes.get(name, 0.0))
                    for name in ("forward", "turn", "side")
                }
            else:
                self._axes = {"forward": 0.0, "turn": 0.0, "side": 0.0}
            moving = any(abs(value) > 0.0 for value in self._axes.values())
            if moving and not was_moving:
                self._reset_requested = True
            if bool(request.get("stop", False)) or bool(request.get("reset", False)):
                self._axes = {"forward": 0.0, "turn": 0.0, "side": 0.0}
                self._reset_requested = True
            if not self._armed:
                self._reset_requested = True
            return self._snapshot_locked()

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            return self._snapshot_locked()

    def _snapshot_locked(self) -> dict[str, object]:
        return {
            "sequence": self._last_sequence,
            "armed": self._armed,
            "port": self.port_label,
            "status": self._status,
            "axes": dict(self._axes),
            "frames": self._frames,
            "pose": dict(self._pose),
            "gait": dict(self._gait),
            "error": self._error,
        }

    def _run(self) -> None:
        period_s = self.config.update_ms / 1000.0
        try:
            while not self._stop.is_set():
                started = time.monotonic()
                with self._lock:
                    if self._armed and started - self._last_command_at > 0.7:
                        self._armed = False
                        self._axes = {"forward": 0.0, "turn": 0.0, "side": 0.0}
                        self._reset_requested = True
                    armed = self._armed
                    axes = dict(self._axes)
                    reset_requested = self._reset_requested
                    self._reset_requested = False

                if reset_requested:
                    self.engine.reset()
                    pose = dict(STANDING)
                    self.backend.send(pose, duration_ms=self.config.stop_ms, force=True)
                    status = "READY" if armed else "OUTPUT DISABLED"
                else:
                    forward = axes["forward"] * self.config.walk_speed
                    turn = axes["turn"] * self.config.turn_speed
                    side = axes["side"] * self.config.side_speed
                    pose = self.engine.update(forward, turn_cmd=turn, side_cmd=side)
                    self.backend.send(pose, duration_ms=self.config.update_ms)
                    directions = []
                    if forward:
                        directions.append("FORWARD" if forward > 0.0 else "BACKWARD")
                    if turn:
                        directions.append("TURN LEFT" if turn > 0.0 else "TURN RIGHT")
                    if side:
                        directions.append("SIDE LEFT" if side > 0.0 else "SIDE RIGHT")
                    status = " + ".join(directions) if directions else (
                        "READY" if self.engine.is_idle_ready() else "SETTLING"
                    )

                with self._lock:
                    self._pose = dict(pose)
                    self._gait = self.engine.telemetry_snapshot()
                    self._status = status
                    self._frames += 1

                remaining = period_s - (time.monotonic() - started)
                if remaining > 0.0:
                    self._stop.wait(remaining)
        except Exception as exc:
            with self._lock:
                self._armed = False
                self._axes = {"forward": 0.0, "turn": 0.0, "side": 0.0}
                self._status = "SERIAL ERROR"
                self._error = str(exc)


class ManualTestServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True
    controller: ManualController

    def handle_error(self, request: object, client_address: object) -> None:
        _type, error, _traceback = sys.exc_info()
        if isinstance(error, (BrokenPipeError, ConnectionAbortedError, ConnectionResetError)):
            return
        super().handle_error(request, client_address)


class ManualTestHandler(BaseHTTPRequestHandler):
    server: ManualTestServer
    protocol_version = "HTTP/1.1"

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/state":
            self._send_json(self.server.controller.snapshot())
            return
        self._serve_static(path)

    def do_POST(self) -> None:
        if urlparse(self.path).path != "/api/control":
            self.send_error(404, "Not found")
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = 0
        if length <= 0 or length > 2048:
            self.send_error(400, "Invalid request size")
            return
        try:
            request = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self.send_error(400, "Invalid JSON")
            return
        if not isinstance(request, dict):
            self.send_error(400, "JSON object required")
            return
        self._send_json(self.server.controller.apply(request))

    def _serve_static(self, request_path: str) -> None:
        relative = "index.html" if request_path in ("", "/") else unquote(request_path.lstrip("/"))
        path = (STATIC_ROOT / relative).resolve()
        if not path.is_relative_to(STATIC_ROOT.resolve()) or not path.is_file():
            self.send_error(404, "File not found")
            return
        content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        if path.suffix == ".js":
            content_type = "text/javascript"
        self._send_bytes(path.read_bytes(), content_type)

    def _send_json(self, value: object) -> None:
        payload = json.dumps(value, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self._send_bytes(payload, "application/json")

    def _send_bytes(self, payload: bytes, content_type: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, _format: str, *_args: object) -> None:
        return


def main() -> None:
    parser = argparse.ArgumentParser(description="Standalone laptop manual-control dashboard.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument("--servo-port")
    parser.add_argument("--servo-baudrate", type=int, default=115200)
    parser.add_argument("--dry-run", action="store_true")
    options = parser.parse_args()
    if not options.dry_run and not options.servo_port:
        parser.error("--servo-port is required unless --dry-run is used")

    config = Config()
    backend = (
        MockBackend(verbose=False)
        if options.dry_run
        else SerialRTBackend(options.servo_port, options.servo_baudrate)
    )
    port_label = "DRY RUN" if options.dry_run else str(options.servo_port)
    controller = ManualController(config, backend, port_label)
    server = ManualTestServer((options.host, options.port), ManualTestHandler)
    server.controller = controller

    controller.start()
    print(f"[manual-test] Open http://{options.host}:{server.server_address[1]}")
    print(f"[manual-test] Servo: {port_label} @ {options.servo_baudrate}")
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        controller.close()


if __name__ == "__main__":
    main()
