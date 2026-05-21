from __future__ import annotations

import math
import time
from typing import Dict, Generator, Tuple

from .rtrobot_xml import ActionGroup


def ease_sine_inout(t: float) -> float:
    return -(math.cos(math.pi * t) - 1.0) / 2.0


def ease_quintic_inout(t: float) -> float:
    if t < 0.5:
        return 16.0 * t ** 5
    p = -2.0 * t + 2.0
    return 1.0 - p ** 5 / 2.0


def ease_linear(t: float) -> float:
    return t


EASING_FUNCS = {
    "sine": ease_sine_inout,
    "quintic": ease_quintic_inout,
    "linear": ease_linear,
}


def interpolate_poses(
    start: Dict[int, int],
    end: Dict[int, int],
    t: float,
    easing: str = "sine",
) -> Dict[int, int]:
    """Interpolate two servo-pulse poses with the selected easing curve."""
    ease_fn = EASING_FUNCS.get(easing, ease_sine_inout)
    et = ease_fn(max(0.0, min(1.0, t)))
    all_ids = set(start) | set(end)
    return {
        sid: round(
            start.get(sid, end.get(sid, 1500))
            + (end.get(sid, start.get(sid, 1500)) - start.get(sid, end.get(sid, 1500))) * et
        )
        for sid in all_ids
    }


def _catmull_scalar(p0: float, p1: float, p2: float, p3: float, t: float) -> float:
    t2 = t * t
    t3 = t2 * t
    return 0.5 * (
        2 * p1
        + (-p0 + p2) * t
        + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
        + (-p0 + 3 * p1 - 3 * p2 + p3) * t3
    )


def catmull_rom_poses(
    p0: Dict[int, int],
    p1: Dict[int, int],
    p2: Dict[int, int],
    p3: Dict[int, int],
    t: float,
    pulse_min: int = 500,
    pulse_max: int = 2500,
) -> Dict[int, int]:
    """Catmull-Rom pose interpolation between p1 and p2."""
    all_ids = set(p0) | set(p1) | set(p2) | set(p3)
    result: Dict[int, int] = {}
    for sid in all_ids:
        v0 = float(p0.get(sid, p1.get(sid, 1500)))
        v1 = float(p1.get(sid, 1500))
        v2 = float(p2.get(sid, 1500))
        v3 = float(p3.get(sid, p2.get(sid, 1500)))
        val = _catmull_scalar(v0, v1, v2, v3, t)
        result[sid] = max(pulse_min, min(pulse_max, round(val)))
    return result


def playback(
    group: ActionGroup,
    loop: bool = False,
    update_ms: int = 20,
    easing: str = "catmull",
    start_pose: Dict[int, int] | None = None,
) -> Generator[Tuple[Dict[int, int], bool], None, None]:
    """
    Yield (pose_dict, frame_done) at update_ms intervals.

    easing="catmull" gives velocity-continuous keyframe playback.
    easing="sine" gives per-frame ease-in/ease-out playback.
    easing="linear" gives constant velocity per frame.
    """
    update_s = update_ms / 1000.0
    frames = group.frames
    n = len(frames)

    if not frames:
        return

    current_pose: Dict[int, int] = dict(start_pose) if start_pose else dict(frames[0].servos)
    poses = [dict(f.servos) for f in frames]
    frame_idx = 0

    while True:
        frame = frames[frame_idx]
        duration_s = frame.duration_ms / 1000.0
        elapsed_s = 0.0

        if easing == "catmull":
            if loop:
                p0 = poses[(frame_idx - 1) % n]
                p1 = poses[frame_idx % n]
                p2 = poses[(frame_idx + 1) % n]
                p3 = poses[(frame_idx + 2) % n]
            else:
                p0 = current_pose if frame_idx == 0 else poses[frame_idx - 1]
                p1 = poses[frame_idx]
                p2 = poses[min(n - 1, frame_idx + 1)]
                p3 = poses[min(n - 1, frame_idx + 2)]

            target_pose = p1
            while elapsed_s < duration_s:
                yield catmull_rom_poses(p0, p1, p2, p3, elapsed_s / duration_s), False
                time.sleep(update_s)
                elapsed_s += update_s
        else:
            target_pose = dict(frame.servos)
            while elapsed_s < duration_s:
                yield interpolate_poses(current_pose, target_pose, elapsed_s / duration_s, easing), False
                time.sleep(update_s)
                elapsed_s += update_s

        yield dict(target_pose), True

        if frame.delay_ms > 0:
            for _ in range(max(1, frame.delay_ms // update_ms)):
                yield dict(target_pose), False
                time.sleep(update_s)

        current_pose = dict(target_pose)
        frame_idx += 1

        if frame_idx >= n:
            if loop:
                frame_idx = 0
            else:
                return
