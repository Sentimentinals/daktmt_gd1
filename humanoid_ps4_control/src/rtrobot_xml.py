"""
Parse RTrobot-style XML action files into Python dataclasses.

Frame format:
    #1P1551#2P2050#3P1660T1000D100

Tokens:
    #<id>P<pulse>  servo channel id and pulse width in microseconds
    T<ms>          transition time in milliseconds
    D<ms>          delay after motion completes in milliseconds
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterator, List, Optional


@dataclass
class PoseFrame:
    """One keyframe: servo pulse dict plus timing."""

    servos: Dict[int, int]
    duration_ms: int = 1000
    delay_ms: int = 0

    def raw_command(self) -> str:
        parts = [f"#{sid}P{pulse}" for sid, pulse in sorted(self.servos.items())]
        return "".join(parts) + f"T{self.duration_ms}D0"


@dataclass
class ActionGroup:
    """One parsed <Group> entry."""

    group_id: int
    alias: str
    frames: List[PoseFrame] = field(default_factory=list)

    def __repr__(self) -> str:
        return f"ActionGroup(id={self.group_id}, alias={self.alias!r}, frames={len(self.frames)})"


_SERVO_RE = re.compile(r"#(\d+)P(\d+)")
_TIME_RE = re.compile(r"T(\d+)")
_DELAY_RE = re.compile(r"D(\d+)")


def parse_frame(line: str) -> Optional[PoseFrame]:
    """Parse one raw RTrobot frame string."""
    line = line.strip()
    if not line:
        return None

    servos: Dict[int, int] = {}
    for match in _SERVO_RE.finditer(line):
        servos[int(match.group(1))] = int(match.group(2))

    if not servos:
        return None

    t_match = _TIME_RE.search(line)
    d_match = _DELAY_RE.search(line)
    duration_ms = int(t_match.group(1)) if t_match else 1000
    delay_ms = int(d_match.group(1)) if d_match else 0
    return PoseFrame(servos=servos, duration_ms=duration_ms, delay_ms=delay_ms)


def parse_group_value(text: str) -> List[PoseFrame]:
    """Parse a <value> block into keyframes."""
    frames: List[PoseFrame] = []
    for line in text.splitlines():
        frame = parse_frame(line)
        if frame is not None:
            frames.append(frame)
    return frames


def load_xml(path: str | Path) -> Dict[int, ActionGroup]:
    """Load all non-empty action groups from an RTrobot XML file."""
    tree = ET.parse(str(path))
    root = tree.getroot()

    groups: Dict[int, ActionGroup] = {}
    for group_el in root.findall("Group"):
        gid = int(group_el.get("id", -1))
        alias = group_el.get("alias", "")
        value_el = group_el.find("value")
        text = value_el.text if value_el is not None and value_el.text else ""
        frames = parse_group_value(text)
        if frames:
            groups[gid] = ActionGroup(group_id=gid, alias=alias, frames=frames)

    return groups


def iter_frames(group: ActionGroup, loop: bool = False) -> Iterator[PoseFrame]:
    """Yield frames from an action group, optionally looping forever."""
    while True:
        yield from group.frames
        if not loop:
            break


def analyse_group(group: ActionGroup) -> dict:
    """Return a compact summary for quick XML inspection."""
    if not group.frames:
        return {}

    all_servos = sorted({sid for frame in group.frames for sid in frame.servos})
    ref = group.frames[0].servos

    changed: set[int] = set()
    for frame in group.frames[1:]:
        for sid, pulse in frame.servos.items():
            if pulse != ref.get(sid):
                changed.add(sid)

    return {
        "group_id": group.group_id,
        "alias": group.alias,
        "frame_count": len(group.frames),
        "all_servos": all_servos,
        "changed_servos": sorted(changed),
    }


if __name__ == "__main__":
    import sys

    xml_path = sys.argv[1] if len(sys.argv) > 1 else "actions/standing.xml"
    groups = load_xml(xml_path)
    print(f"Loaded {len(groups)} non-empty group(s) from: {xml_path}\n")
    for gid, grp in groups.items():
        info = analyse_group(grp)
        print(f"Group {gid} alias={grp.alias!r} frames={len(grp.frames)}")
        print(f"  All servos    : {info['all_servos']}")
        print(f"  Changed servos: {info['changed_servos']}")
        for i, frame in enumerate(grp.frames):
            print(f"  Frame {i}: T={frame.duration_ms}ms D={frame.delay_ms}ms servos={len(frame.servos)}")
        print()
