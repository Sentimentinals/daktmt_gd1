from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import statistics
import time
import urllib.request
import zipfile
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

from src.gait_anomaly import FEATURE_NAMES, _Sample, extract_features


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
    print(f"experimental={payload.get('experimental', False)}")
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
        / len(values)
    )


def train_public(args: argparse.Namespace) -> int:
    """Human walking proxy, never a validated robot fault classifier."""
    import numpy as np

    archive_bytes = args.dataset.read_bytes()
    source_hash = hashlib.sha256(archive_bytes).hexdigest()
    archive = zipfile.ZipFile(io.BytesIO(archive_bytes))
    nested = next((name for name in archive.namelist() if name.endswith(".zip")), None)
    if nested:
        archive = zipfile.ZipFile(io.BytesIO(archive.read(nested)))
    prefix = next(name[:-len("train/y_train.txt")] for name in archive.namelist()
                  if name.endswith("train/y_train.txt"))

    def read_array(name):
        return np.loadtxt(io.BytesIO(archive.read(prefix + name)))

    splits = {}
    injected_rows = []
    for split in ("train", "test"):
        labels = read_array(f"{split}/y_{split}.txt").astype(int)
        subjects = read_array(f"{split}/subject_{split}.txt").astype(int)
        gravity = []
        for axis in "xyz":
            folder = f"{split}/Inertial Signals/"
            gravity.append(read_array(folder + f"total_acc_{axis}_{split}.txt")
                           - read_array(folder + f"body_acc_{axis}_{split}.txt"))
        gx, gy, gz = gravity
        # Phone x points along gravity at the waist: map x to upright robot z.
        roll = np.degrees(np.arctan2(gy, gx))
        pitch = np.degrees(np.arctan2(-gz, np.hypot(gx, gy)))
        sample_times = np.arange(128) / 50.0
        target_times = np.arange(0.0, 2.54, 0.03)
        rows = []
        for i in range(len(labels)):
            r = np.degrees(np.unwrap(np.radians(roll[i])))
            p = np.degrees(np.unwrap(np.radians(pitch[i])))
            samples = [_Sample(float(t), float(a), float(b), None, "none", 0)
                       for t, a, b in zip(target_times,
                           np.interp(target_times, sample_times, r),
                           np.interp(target_times, sample_times, p))]
            features = extract_features(samples)
            rows.append([features[name] for name in FEATURE_NAMES[:10]])
            if split == "test" and labels[i] == 1:
                disturbed = [_Sample(s.time_s, s.roll_deg + (15.0 if 30 <= j < 38 else 0.0),
                                     s.pitch_deg, None, "none", 0) for j, s in enumerate(samples)]
                injected = extract_features(disturbed)
                injected_rows.append([injected[name] for name in FEATURE_NAMES[:10]])
        splits[split] = (labels, subjects, rows)

    labels, subjects, rows = splits["train"]
    calibration_subjects = set(sorted(set(subjects))[::5])
    normal = [row for label, subject, row in zip(labels, subjects, rows)
              if label == 1 and subject not in calibration_subjects]
    calibration = [row for label, subject, row in zip(labels, subjects, rows)
                   if label == 1 and subject in calibration_subjects]
    test_labels, test_subjects, test_rows = splits["test"]
    assert not set(subjects) & set(test_subjects)
    profile = _build_profile(normal)
    score = lambda row: _score(row, profile["center"], profile["scale"])
    profile["threshold"] = round(max(2.5, _percentile([score(r) for r in calibration], .99) * 1.25), 5)
    evaluation = {}
    for label, name in enumerate(("walking", "upstairs", "downstairs", "sitting", "standing", "laying"), 1):
        values = [score(row) for activity, row in zip(test_labels, test_rows) if activity == label]
        flagged = sum(value > profile["threshold"] for value in values)
        evaluation[name] = {"windows": len(values), "flagged": flagged,
                            "flag_rate": round(flagged / len(values), 5)}
    payload = {
        "version": 1, "model_id": "uci-har-public-" + source_hash[:12],
        "experimental": True, "feature_names": list(FEATURE_NAMES[:10]),
        "profiles": {"global": profile},
        "source": {"doi": "10.24432/C54S4K", "license": "CC BY 4.0",
                   "authors": "Reyes-Ortiz, Anguita, Ghio, Oneto, Parra (2013)",
                   "url": "https://archive.ics.uci.edu/dataset/240/human+activity+recognition+using+smartphones",
                   "archive_sha256": source_hash},
        "method": "Gravity proxy = total_acc - body_acc; roll=atan2(gy,gx), pitch=atan2(-gz,hypot(gx,gy)); 50Hz to 33.33Hz; first 10 gait features; median/MAD baseline.",
        "validation": {"fit_windows": len(normal), "calibration_windows": len(calibration),
                       "fit_subjects": sorted(map(int, set(subjects) - calibration_subjects)),
                       "calibration_subjects": sorted(map(int, calibration_subjects)),
                       "test_subjects": sorted(map(int, set(test_subjects))),
                       "test": evaluation,
                       "synthetic_sensitivity": {"injection": "15 degree roll pulse, samples 30..37 at 33.33Hz, on held-out walking only; NOT real faults",
                                                 "windows": len(injected_rows),
                                                 "flagged": sum(score(row) > profile["threshold"] for row in injected_rows)}},
        "limitations": "Human phone gravity (0.3Hz filtered) is not BNO055 robot orientation. Original windows overlap 50 percent. Activities other than walking are NOT fault labels. No robot accuracy or predictive-maintenance claim. Excludes unavailable gravity/asymmetry/step-count features. Requires robot baseline.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload["validation"], indent=2))
    print(f"Public-data experimental model: {args.output}")
    return 0


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
    public_parser = commands.add_parser("train-public", help="Train an experimental UCI HAR walking proxy")
    public_parser.add_argument("--dataset", type=Path, required=True, help="Official UCI HAR ZIP")
    public_parser.add_argument("--output", type=Path, default=DEFAULT_MODEL.with_name("gait_anomaly_public.json"))
    public_parser.set_defaults(handler=train_public)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    return args.handler(args)


if __name__ == "__main__":
    raise SystemExit(main())
