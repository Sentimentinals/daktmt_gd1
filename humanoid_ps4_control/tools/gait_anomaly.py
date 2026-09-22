from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
import urllib.request
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

from src.gait_anomaly import FEATURE_NAMES


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DATASET = PROJECT_ROOT / "out" / "gait_anomaly_baseline.csv"
DEFAULT_MODEL = PROJECT_ROOT / "deploy" / "models" / "gait_anomaly.json"
SCALE_FLOORS = (
    0.05,
    0.05,
    0.10,
    0.10,
    0.50,
    0.50,
    1.00,
    1.00,
    5.00,
    5.00,
    0.005,
    0.10,
    0.10,
    0.05,
)


def collect(args: argparse.Namespace) -> int:
    endpoint = args.url.rstrip("/") + "/api/events"
    request = urllib.request.Request(endpoint, headers={"Accept": "text/event-stream"})
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = ["captured_at", "profile", "window_id", *FEATURE_NAMES]
    write_header = not output.exists() or output.stat().st_size == 0
    deadline = time.monotonic() + max(1.0, args.seconds)
    rows = 0
    last_window_id = None
    print(f"[gait-anomaly] Reading completed IMU windows from {endpoint}")
    print("[gait-anomaly] Operate only normal walking; include forward, backward, turn, and side walk.")
    try:
        with urllib.request.urlopen(request, timeout=15.0) as response, output.open(
            "a", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            if write_header:
                writer.writeheader()
            while time.monotonic() < deadline:
                line = response.readline()
                if not line:
                    break
                if not line.startswith(b"data:"):
                    continue
                frame = json.loads(line[5:].decode("utf-8"))
                health = frame.get("gait_health") or {}
                window_id = health.get("window_id")
                features = health.get("features")
                profile = health.get("profile")
                if not window_id or window_id == last_window_id or not profile or not isinstance(features, dict):
                    continue
                values = {name: float(features[name]) for name in FEATURE_NAMES}
                if not all(math.isfinite(value) for value in values.values()):
                    continue
                writer.writerow(
                    {
                        "captured_at": frame.get("wall_time") or datetime.now(timezone.utc).isoformat(),
                        "profile": profile,
                        "window_id": window_id,
                        **values,
                    }
                )
                stream.flush()
                last_window_id = window_id
                rows += 1
                print(f"[gait-anomaly] window={rows} profile={profile}")
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"[gait-anomaly] Collection failed: {exc}")
        return 2
    print(f"[gait-anomaly] Saved {rows} windows to {output}")
    return 0 if rows else 2


def train(args: argparse.Namespace) -> int:
    rows = _read_dataset(args.dataset)
    if len(rows) < args.min_global_windows:
        raise RuntimeError(
            f"Need at least {args.min_global_windows} normal windows; found {len(rows)}"
        )
    groups: dict[str, list[list[float]]] = defaultdict(list)
    for profile, values in rows:
        groups[profile].append(values)

    profiles = {"global": _build_profile([values for _, values in rows])}
    for name, values in sorted(groups.items()):
        if len(values) >= args.min_profile_windows:
            profiles[name] = _build_profile(values)

    trained_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
    payload = {
        "version": 1,
        "model_id": trained_at,
        "trained_at": trained_at,
        "feature_names": list(FEATURE_NAMES),
        "profiles": profiles,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    included = ", ".join(f"{name}={profile['samples']}" for name, profile in profiles.items())
    skipped = ", ".join(
        f"{name}={len(values)}"
        for name, values in sorted(groups.items())
        if name not in profiles
    )
    print(f"[gait-anomaly] Model profiles: {included}")
    if skipped:
        print(f"[gait-anomaly] Need more windows for: {skipped}")
    print(f"[gait-anomaly] Wrote {args.output.resolve()}")
    return 0


def inspect(args: argparse.Namespace) -> int:
    payload = json.loads(args.model.read_text(encoding="utf-8"))
    print(f"model_id={payload.get('model_id', 'unknown')}")
    for name, profile in payload.get("profiles", {}).items():
        print(
            f"{name}: samples={profile.get('samples', 0)} "
            f"threshold={float(profile.get('threshold', 0.0)):.3f}"
        )
    return 0


def _read_dataset(path: Path) -> list[tuple[str, list[float]]]:
    rows = []
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"profile", *FEATURE_NAMES}
        if not required.issubset(reader.fieldnames or ()):
            raise RuntimeError("Dataset header does not match the gait feature format")
        for row in reader:
            try:
                profile = str(row["profile"])
                values = [float(row[name]) for name in FEATURE_NAMES]
            except (TypeError, ValueError, KeyError):
                continue
            if profile and all(math.isfinite(value) for value in values):
                rows.append((profile, values))
    return rows


def _build_profile(rows: list[list[float]]) -> dict[str, object]:
    columns = list(zip(*rows))
    center = [statistics.median(column) for column in columns]
    scale = []
    for index, column in enumerate(columns):
        mad = statistics.median(abs(value - center[index]) for value in column) * 1.4826
        fallback = statistics.pstdev(column) if len(column) >= 2 else 0.0
        scale.append(max(SCALE_FLOORS[index], mad, fallback * 0.5))
    scores = [_score(values, center, scale) for values in rows]
    threshold = max(2.5, _percentile(scores, 0.99) * 1.25)
    return {
        "center": [round(value, 7) for value in center],
        "scale": [round(value, 7) for value in scale],
        "threshold": round(threshold, 5),
        "samples": len(rows),
    }


def _score(values: list[float], center: list[float], scale: list[float]) -> float:
    return math.sqrt(
        sum(((value - mid) / width) ** 2 for value, mid, width in zip(values, center, scale))
        / len(FEATURE_NAMES)
    )


def _percentile(values: list[float], quantile: float) -> float:
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = max(0.0, min(1.0, quantile)) * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Collect and train the IMU gait anomaly monitor")
    commands = parser.add_subparsers(dest="command", required=True)

    collect_parser = commands.add_parser("collect", help="Collect normal gait windows from the dashboard")
    collect_parser.add_argument("--url", default="http://127.0.0.1:8765")
    collect_parser.add_argument("--seconds", type=float, default=180.0)
    collect_parser.add_argument("--output", type=Path, default=DEFAULT_DATASET)
    collect_parser.set_defaults(handler=collect)

    train_parser = commands.add_parser("train", help="Train a compact one-class baseline model")
    train_parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    train_parser.add_argument("--output", type=Path, default=DEFAULT_MODEL)
    train_parser.add_argument("--min-global-windows", type=int, default=24)
    train_parser.add_argument("--min-profile-windows", type=int, default=8)
    train_parser.set_defaults(handler=train)

    inspect_parser = commands.add_parser("inspect", help="Show model profiles and thresholds")
    inspect_parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    inspect_parser.set_defaults(handler=inspect)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    return args.handler(args)


if __name__ == "__main__":
    raise SystemExit(main())
