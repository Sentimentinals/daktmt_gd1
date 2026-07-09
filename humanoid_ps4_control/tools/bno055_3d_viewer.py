from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class ImuFrame:
    quaternion: Tuple[float, float, float, float]
    heading: float
    roll: float
    pitch: float
    calibration: Tuple[int, int, int, int]
    received_at: float


class FrameStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frame: Optional[ImuFrame] = None
        self.error: Optional[str] = None

    def set(self, frame: ImuFrame) -> None:
        with self._lock:
            self._frame = frame

    def get(self) -> Optional[ImuFrame]:
        with self._lock:
            return self._frame


def parse_frame(line: str) -> Optional[ImuFrame]:
    fields = line.strip().split(",")
    if len(fields) != 13 or fields[0] != "Q":
        return None

    try:
        quaternion = tuple(float(value) for value in fields[2:6])
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm < 1e-9:
            return None
        normalized = tuple(value / norm for value in quaternion)
        calibration = tuple(int(value) for value in fields[9:13])
        if any(value < 0 or value > 3 for value in calibration):
            return None
        return ImuFrame(
            quaternion=normalized,  # type: ignore[arg-type]
            heading=float(fields[6]),
            roll=float(fields[7]),
            pitch=float(fields[8]),
            calibration=calibration,  # type: ignore[arg-type]
            received_at=time.monotonic(),
        )
    except ValueError:
        return None


def find_serial_port(requested: str) -> str:
    if requested.lower() != "auto":
        return requested

    from serial.tools import list_ports

    ports = list(list_ports.comports())
    preferred = [
        port
        for port in ports
        if port.device.startswith(("/dev/ttyUSB", "/dev/ttyACM"))
    ]
    if len(preferred) == 1:
        return preferred[0].device
    if not preferred:
        raise RuntimeError("Khong tim thay /dev/ttyUSB* hoac /dev/ttyACM*.")

    details = ", ".join(
        f"{port.device} ({port.description})" for port in preferred
    )
    raise RuntimeError(
        "Co nhieu cong USB serial. Chay lai voi --port PORT: " + details
    )


def serial_worker(store: FrameStore, port: str, baud: int) -> None:
    try:
        import serial

        with serial.Serial(port, baud, timeout=0.25) as connection:
            connection.reset_input_buffer()
            while True:
                raw = connection.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                frame = parse_frame(line)
                if frame is not None:
                    store.set(frame)
                elif line.startswith("# ERROR"):
                    store.error = line
    except Exception as exc:
        store.error = str(exc)


def demo_frame(started_at: float) -> ImuFrame:
    elapsed = time.monotonic() - started_at
    yaw = elapsed * 0.45
    pitch = math.sin(elapsed * 0.7) * 0.35
    roll = math.sin(elapsed * 0.9) * 0.25

    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    quaternion = (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )
    return ImuFrame(
        quaternion=quaternion,
        heading=math.degrees(yaw) % 360,
        roll=math.degrees(roll),
        pitch=math.degrees(pitch),
        calibration=(3, 3, 3, 3),
        received_at=time.monotonic(),
    )


def quaternion_matrix(quaternion: Sequence[float]) -> list[float]:
    w, x, y, z = quaternion
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    rows = (
        (1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)),
        (2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)),
        (2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)),
    )
    return [
        rows[0][0],
        rows[1][0],
        rows[2][0],
        0.0,
        rows[0][1],
        rows[1][1],
        rows[2][1],
        0.0,
        rows[0][2],
        rows[1][2],
        rows[2][2],
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]


def draw_axes(gl, length: float = 2.2, width: float = 3.0) -> None:
    gl.glLineWidth(width)
    gl.glBegin(gl.GL_LINES)
    for color, endpoint in (
        ((1.0, 0.2, 0.2), (length, 0.0, 0.0)),
        ((0.2, 1.0, 0.3), (0.0, length, 0.0)),
        ((0.2, 0.5, 1.0), (0.0, 0.0, length)),
    ):
        gl.glColor3f(*color)
        gl.glVertex3f(0.0, 0.0, 0.0)
        gl.glVertex3f(*endpoint)
    gl.glEnd()


def draw_grid(gl) -> None:
    gl.glLineWidth(1.0)
    gl.glColor3f(0.26, 0.28, 0.31)
    gl.glBegin(gl.GL_LINES)
    for value in range(-4, 5):
        gl.glVertex3f(value, -4.0, 0.0)
        gl.glVertex3f(value, 4.0, 0.0)
        gl.glVertex3f(-4.0, value, 0.0)
        gl.glVertex3f(4.0, value, 0.0)
    gl.glEnd()


def draw_body(gl) -> None:
    vertices = (
        (-1.6, -0.8, -0.25),
        (1.6, -0.8, -0.25),
        (1.6, 0.8, -0.25),
        (-1.6, 0.8, -0.25),
        (-1.6, -0.8, 0.25),
        (1.6, -0.8, 0.25),
        (1.6, 0.8, 0.25),
        (-1.6, 0.8, 0.25),
    )
    edges = (
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    )
    gl.glColor3f(0.9, 0.9, 0.92)
    gl.glLineWidth(2.0)
    gl.glBegin(gl.GL_LINES)
    for start, end in edges:
        gl.glVertex3f(*vertices[start])
        gl.glVertex3f(*vertices[end])
    gl.glEnd()
    draw_axes(gl, length=2.7, width=4.0)


def run_viewer(store: FrameStore, demo: bool) -> int:
    if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
        raise RuntimeError(
            "Khong co DISPLAY. Hay chay trong Raspberry Pi Desktop qua HDMI/VNC."
        )

    import pygame
    from OpenGL import GL as gl
    from OpenGL import GLU as glu

    # Initializing every pygame subsystem also starts PulseAudio. On Raspberry
    # Pi OS Bullseye that can abort the process even though this viewer has no
    # audio, so initialize only the display subsystem.
    pygame.display.init()
    pygame.display.set_mode((960, 640), pygame.DOUBLEBUF | pygame.OPENGL)
    gl.glEnable(gl.GL_DEPTH_TEST)
    gl.glClearColor(0.07, 0.08, 0.09, 1.0)
    glu.gluPerspective(45.0, 960.0 / 640.0, 0.1, 100.0)
    gl.glTranslatef(0.0, 0.0, -10.0)
    gl.glRotatef(58.0, 1.0, 0.0, 0.0)
    gl.glRotatef(-20.0, 0.0, 0.0, 1.0)

    clock = pygame.time.Clock()
    started_at = time.monotonic()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key in (
                pygame.K_ESCAPE,
                pygame.K_q,
            ):
                running = False

        frame = demo_frame(started_at) if demo else store.get()
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        draw_grid(gl)
        draw_axes(gl)

        if frame is not None:
            gl.glPushMatrix()
            gl.glMultMatrixf(quaternion_matrix(frame.quaternion))
            draw_body(gl)
            gl.glPopMatrix()
            sys_cal, gyro_cal, accel_cal, mag_cal = frame.calibration
            age_ms = (time.monotonic() - frame.received_at) * 1000.0
            pygame.display.set_caption(
                "BNO055 | X=red Y=green Z=blue | "
                f"heading={frame.heading:6.1f} roll={frame.roll:6.1f} "
                f"pitch={frame.pitch:6.1f} | CAL "
                f"S{sys_cal} G{gyro_cal} A{accel_cal} M{mag_cal} | "
                f"age={age_ms:4.0f} ms"
            )
        else:
            message = store.error or "Dang cho du lieu BNO055..."
            pygame.display.set_caption("BNO055 | " + message)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Hien thi quaternion BNO055 tu ESP32 USB serial."
    )
    parser.add_argument("--port", default="auto", help="Vi du /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Chay hinh 3D gia lap, khong mo cong serial.",
    )
    args = parser.parse_args()

    store = FrameStore()
    if not args.demo:
        try:
            port = find_serial_port(args.port)
        except RuntimeError as exc:
            print(f"[ERROR] {exc}", file=sys.stderr)
            return 2
        print(f"Reading BNO055 from {port} at {args.baud} baud.")
        reader = threading.Thread(
            target=serial_worker,
            args=(store, port, args.baud),
            daemon=True,
        )
        reader.start()

    try:
        return run_viewer(store, args.demo)
    except ImportError as exc:
        print(
            "[ERROR] Thieu goi Python. Cai requirements-imu-viewer.txt.",
            file=sys.stderr,
        )
        print(exc, file=sys.stderr)
        return 3
    except RuntimeError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 4


if __name__ == "__main__":
    raise SystemExit(main())
