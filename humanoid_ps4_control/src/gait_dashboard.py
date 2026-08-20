from __future__ import annotations

import json
import mimetypes
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, unquote, urlparse

from .config import DIR, PWM_PER_DEG, ROBOT, STAND_ANG, STANDING


STATIC_ROOT = Path(__file__).resolve().parent.parent / "web" / "gait_dashboard"
SESSION_ROOT = Path(__file__).resolve().parent.parent / "out" / "gait_sessions"

JOINT_NAMES = {
    9: "left_elbow",
    10: "left_upper_arm",
    11: "left_shoulder_swing",
    12: "left_hip_roll",
    13: "left_hip_pitch",
    14: "left_knee",
    15: "left_ankle_pitch",
    16: "left_ankle_roll",
    17: "right_ankle_roll",
    18: "right_ankle_pitch",
    19: "right_knee",
    20: "right_hip_pitch",
    21: "right_hip_roll",
    22: "right_shoulder_swing",
    23: "right_upper_arm",
    24: "right_elbow",
    25: "head_yaw",
}

LEG_BASE_ANGLES = {
    12: STAND_ANG["L_hip_abduct"],
    13: STAND_ANG["L_hip_pitch"],
    14: STAND_ANG["L_knee"],
    15: STAND_ANG["L_ankle"],
    16: STAND_ANG["hip_roll"],
    17: STAND_ANG["hip_roll"],
    18: STAND_ANG["R_ankle"],
    19: STAND_ANG["R_knee"],
    20: STAND_ANG["R_hip_pitch"],
    21: STAND_ANG["R_hip_abduct"],
}


def _json_ready(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_ready(item) for item in value]
    if hasattr(value, "item"):
        return value.item()
    return value


def _model_payload() -> dict[str, object]:
    return {
        "dimensions_mm": ROBOT,
        "pwm_per_deg": PWM_PER_DEG,
        "standing_pwm": {str(sid): pwm for sid, pwm in STANDING.items()},
        "directions": {str(sid): direction for sid, direction in DIR.items()},
        "base_angles_deg": {str(sid): angle for sid, angle in LEG_BASE_ANGLES.items()},
        "joints": [
            {"id": sid, "name": name}
            for sid, name in sorted(JOINT_NAMES.items())
        ],
    }


class GaitDashboard:
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8765,
        stream_hz: int = 12,
        history_seconds: int = 120,
        camera=None,
    ) -> None:
        self.host = host
        self.port = port
        self.stream_period_s = 1.0 / max(2, stream_hz)
        self.camera = camera
        self._started_at = time.monotonic()
        self._stop = threading.Event()
        self._condition = threading.Condition()
        self._sequence = 0
        self._latest_frame: dict[str, object] = {}
        self._latest_payload = b"{}"
        self._history = deque(maxlen=max(250, history_seconds * 30))
        self._active_session: list[dict[str, object]] | None = None
        self._session_idle_since: float | None = None
        self._server: _DashboardServer | None = None
        self._server_thread: threading.Thread | None = None
        self._writer_threads: list[threading.Thread] = []

    @property
    def url(self) -> str:
        return f"http://{self.host}:{self.port}"

    def start(self) -> None:
        if self._server is not None:
            return
        self._stop.clear()
        self._server = _DashboardServer((self.host, self.port), _DashboardHandler)
        self._server.dashboard = self
        self.port = int(self._server.server_address[1])
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name="gait-dashboard",
            daemon=True,
        )
        self._server_thread.start()
        print(f"[dashboard] Gait Lab ready at {self.url}")

    def close(self) -> None:
        self._stop.set()
        with self._condition:
            self._finish_session_locked()
            self._condition.notify_all()
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
        if self._server_thread is not None:
            self._server_thread.join(timeout=1.5)
        for writer in self._writer_threads:
            writer.join(timeout=1.5)
        self._writer_threads.clear()
        self._server = None
        self._server_thread = None

    def publish(
        self,
        pose: dict[int, int],
        gait: dict[str, object],
        sensor_snapshot,
        status: str,
        active: bool,
        camera_ready: bool,
        balance_status: str,
    ) -> None:
        now = time.monotonic()
        imu = None
        feet = None
        depth = None
        if sensor_snapshot is not None:
            reading = sensor_snapshot.imu
            if reading is not None:
                imu = {
                    "roll_deg": reading.roll_deg,
                    "pitch_deg": reading.pitch_deg,
                    "yaw_deg": reading.yaw_deg,
                    "calibration": [
                        reading.system_cal,
                        reading.gyro_cal,
                        reading.accel_cal,
                        reading.mag_cal,
                    ],
                }
            force = sensor_snapshot.feet
            if force is not None:
                feet = {
                    "left": force.left_force,
                    "right": force.right_force,
                    "left_raw": force.left_raw,
                    "right_raw": force.right_raw,
                }
            distance = sensor_snapshot.depth
            if distance is not None:
                depth = {
                    "center_mm": distance.center_distance_mm,
                    "obstacle_mm": distance.obstacle_distance_mm,
                    "grid_mm": distance.distances_mm,
                }

        frame = _json_ready(
            {
                "time_s": round(now - self._started_at, 4),
                "wall_time": datetime.now().isoformat(timespec="milliseconds"),
                "status": status,
                "active": bool(active),
                "camera_ready": bool(camera_ready),
                "balance_status": balance_status,
                "pose_pwm": pose,
                "gait": gait,
                "imu": imu,
                "fsr": feet,
                "depth": depth,
            }
        )
        payload = json.dumps(frame, separators=(",", ":"), allow_nan=False).encode("utf-8")

        with self._condition:
            self._sequence += 1
            self._latest_frame = frame
            self._latest_payload = payload
            self._history.append(frame)
            if active:
                if self._active_session is None:
                    pre_roll = list(self._history)[:-1][-15:]
                    self._active_session = pre_roll
                self._active_session.append(frame)
                self._session_idle_since = None
            elif self._active_session is not None:
                self._active_session.append(frame)
                if self._session_idle_since is None:
                    self._session_idle_since = now
                elif now - self._session_idle_since >= 1.0:
                    self._finish_session_locked()
            self._condition.notify_all()

    def wait_for_update(self, sequence: int, timeout_s: float) -> tuple[int, bytes]:
        with self._condition:
            self._condition.wait_for(
                lambda: self._sequence != sequence or self._stop.is_set(),
                timeout=timeout_s,
            )
            return self._sequence, self._latest_payload

    def latest_payload(self) -> bytes:
        with self._condition:
            return self._latest_payload

    def history_payload(self) -> bytes:
        with self._condition:
            frames = list(self._history)
        return json.dumps(frames, separators=(",", ":"), allow_nan=False).encode("utf-8")

    def camera_frame(self) -> bytes | None:
        return None if self.camera is None else self.camera.jpeg_frame()

    def session_list(self) -> list[dict[str, object]]:
        if not SESSION_ROOT.exists():
            return []
        sessions = []
        for path in sorted(SESSION_ROOT.glob("gait_*.jsonl"), reverse=True):
            stat = path.stat()
            sessions.append(
                {
                    "name": path.name,
                    "size": stat.st_size,
                    "modified": datetime.fromtimestamp(stat.st_mtime).isoformat(timespec="seconds"),
                }
            )
        return sessions[:30]

    def session_path(self, name: str) -> Path | None:
        if not name or Path(name).name != name or not name.endswith(".jsonl"):
            return None
        path = SESSION_ROOT / name
        return path if path.is_file() else None

    def _finish_session_locked(self) -> None:
        if not self._active_session:
            self._active_session = None
            self._session_idle_since = None
            return
        frames = self._active_session
        self._active_session = None
        self._session_idle_since = None
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = SESSION_ROOT / f"gait_{timestamp}.jsonl"
        writer = threading.Thread(
            target=self._write_session,
            args=(path, frames),
            name="gait-session-writer",
            daemon=True,
        )
        self._writer_threads.append(writer)
        writer.start()

    @staticmethod
    def _write_session(path: Path, frames: list[dict[str, object]]) -> None:
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("w", encoding="utf-8") as output:
                for frame in frames:
                    output.write(json.dumps(frame, separators=(",", ":"), allow_nan=False))
                    output.write("\n")
            print(f"[dashboard] Saved gait session: {path}")
        except OSError as exc:
            print(f"[dashboard] Cannot save gait session: {exc}")


class _DashboardServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True
    dashboard: GaitDashboard


class _DashboardHandler(BaseHTTPRequestHandler):
    server: _DashboardServer

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/events":
            self._serve_events()
        elif parsed.path == "/api/latest":
            self._send_bytes(self.server.dashboard.latest_payload(), "application/json")
        elif parsed.path == "/api/history":
            self._send_bytes(self.server.dashboard.history_payload(), "application/json")
        elif parsed.path == "/api/model":
            self._send_json(_model_payload())
        elif parsed.path == "/api/sessions":
            self._send_json(self.server.dashboard.session_list())
        elif parsed.path == "/api/session":
            name = parse_qs(parsed.query).get("name", [""])[0]
            self._serve_session(name)
        elif parsed.path == "/camera.mjpg":
            self._serve_camera()
        else:
            self._serve_static(parsed.path)

    def _serve_events(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        sequence = -1
        try:
            while not self.server.dashboard._stop.is_set():
                next_sequence, payload = self.server.dashboard.wait_for_update(sequence, 1.0)
                if next_sequence == sequence:
                    self.wfile.write(b": keepalive\n\n")
                else:
                    self.wfile.write(b"data: " + payload + b"\n\n")
                    sequence = next_sequence
                self.wfile.flush()
                time.sleep(self.server.dashboard.stream_period_s)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def _serve_camera(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            while not self.server.dashboard._stop.is_set():
                jpeg = self.server.dashboard.camera_frame()
                if jpeg is None:
                    time.sleep(0.2)
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def _serve_session(self, name: str) -> None:
        path = self.server.dashboard.session_path(name)
        if path is None:
            self.send_error(404, "Session not found")
            return
        try:
            payload = path.read_bytes()
        except OSError:
            self.send_error(500, "Cannot read session")
            return
        self._send_bytes(payload, "application/x-ndjson")

    def _serve_static(self, request_path: str) -> None:
        relative = "index.html" if request_path in ("", "/") else unquote(request_path.lstrip("/"))
        path = (STATIC_ROOT / relative).resolve()
        if not path.is_relative_to(STATIC_ROOT.resolve()) or not path.is_file():
            self.send_error(404, "File not found")
            return
        content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        if path.suffix == ".js":
            content_type = "text/javascript"
        self._send_bytes(path.read_bytes(), content_type, cache=path.name != "index.html")

    def _send_json(self, value: object) -> None:
        payload = json.dumps(value, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self._send_bytes(payload, "application/json")

    def _send_bytes(self, payload: bytes, content_type: str, cache: bool = False) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "public, max-age=86400" if cache else "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, _format: str, *_args: object) -> None:
        return
