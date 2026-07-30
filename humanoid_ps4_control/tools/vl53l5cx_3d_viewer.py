from __future__ import annotations

import argparse
from typing import Optional

import numpy as np


GRID_SIZE = 8
FRAME_FIELDS = 2 + GRID_SIZE * GRID_SIZE
HORIZONTAL_FOV_DEG = 45.0
VERTICAL_FOV_DEG = 45.0


def parse_depth_line(line: str, max_mm: int) -> Optional[tuple[int, np.ndarray]]:
    fields = [field.strip() for field in line.split(",")]
    if len(fields) != FRAME_FIELDS or fields[0] != "D":
        return None
    try:
        timestamp_ms = int(fields[1])
        distances = np.asarray([int(value) for value in fields[2:]], dtype=float)
    except ValueError:
        return None

    valid = (distances > 0) & (distances <= max_mm)
    if not np.any(valid):
        return None
    distances[~valid] = np.nan
    return timestamp_ms, distances.reshape((GRID_SIZE, GRID_SIZE))


def resolve_port(requested: str) -> str:
    if requested.lower() != "auto":
        return requested

    from serial.tools import list_ports

    matches = []
    for info in list_ports.comports():
        description = " ".join(
            str(getattr(info, field, "") or "")
            for field in ("description", "manufacturer", "hwid")
        ).lower()
        if (getattr(info, "vid", None), getattr(info, "pid", None)) == (0x10C4, 0xEA60) or (
            "cp210" in description
        ):
            matches.append(str(info.device))
    if not matches:
        raise RuntimeError("No CP210x ESP32 serial port found.")
    return sorted(matches)[0]


def depth_coordinates(distances: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    horizontal = np.radians(
        np.linspace(-HORIZONTAL_FOV_DEG / 2.0, HORIZONTAL_FOV_DEG / 2.0, GRID_SIZE)
    )
    vertical = np.radians(
        np.linspace(VERTICAL_FOV_DEG / 2.0, -VERTICAL_FOV_DEG / 2.0, GRID_SIZE)
    )
    tan_x, tan_y = np.meshgrid(np.tan(horizontal), np.tan(vertical))
    depth = distances / np.sqrt(1.0 + tan_x * tan_x + tan_y * tan_y)
    return depth * tan_x, depth * tan_y, depth


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live VL53L5CX 8x8 USB depth surface")
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--max-mm", type=int, default=4000)
    return parser.parse_args()


def main() -> int:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.cm import ScalarMappable
        from matplotlib.colors import Normalize
        import serial
    except ImportError as exc:
        raise SystemExit(
            "Missing viewer dependency. Install pyserial, numpy and matplotlib."
        ) from exc

    args = parse_args()
    port = resolve_port(args.port)
    serial_port = serial.Serial(port, args.baudrate, timeout=0.10)
    serial_port.dtr = False
    serial_port.rts = False

    figure = plt.figure("VL53L5CX 8x8 Depth")
    axis = figure.add_subplot(111, projection="3d")
    axis.set_xlabel("X (mm)")
    axis.set_ylabel("Y (mm)")
    axis.set_zlabel("Depth (mm)")
    half_width = args.max_mm * np.tan(np.radians(HORIZONTAL_FOV_DEG / 2.0))
    half_height = args.max_mm * np.tan(np.radians(VERTICAL_FOV_DEG / 2.0))
    axis.set_xlim(-half_width, half_width)
    axis.set_ylim(-half_height, half_height)
    axis.set_zlim(0, args.max_mm)
    axis.view_init(elev=28, azim=-62)

    normalizer = Normalize(vmin=0, vmax=args.max_mm)
    color_map = plt.get_cmap("turbo")
    figure.colorbar(
        ScalarMappable(norm=normalizer, cmap=color_map),
        ax=axis,
        shrink=0.70,
        pad=0.10,
        label="Distance (mm)",
    )

    surface = [None]
    latest_frame = [None]

    def update(_frame):
        for _ in range(12):
            if serial_port.in_waiting <= 0:
                break
            parsed = parse_depth_line(
                serial_port.readline().decode("ascii", errors="ignore").strip(),
                args.max_mm,
            )
            if parsed is not None:
                latest_frame[0] = parsed

        if latest_frame[0] is None:
            axis.set_title(f"Waiting for VL53L5CX on {port}")
            return ()

        timestamp_ms, distances = latest_frame[0]
        x, y, depth = depth_coordinates(distances)
        if surface[0] is not None:
            surface[0].remove()
        surface[0] = axis.plot_surface(
            x,
            y,
            depth,
            cmap=color_map,
            norm=normalizer,
            vmin=0,
            vmax=args.max_mm,
            linewidth=0.4,
            edgecolor="#20242a",
            antialiased=True,
        )
        valid = distances[np.isfinite(distances)]
        axis.set_title(
            f"VL53L5CX 8x8 | frame {timestamp_ms} ms | "
            f"min {np.min(valid):.0f} mm | median {np.median(valid):.0f} mm"
        )
        return (surface[0],)

    animation = FuncAnimation(
        figure,
        update,
        interval=80,
        blit=False,
        cache_frame_data=False,
    )
    try:
        plt.show()
    finally:
        serial_port.close()
    del animation
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
