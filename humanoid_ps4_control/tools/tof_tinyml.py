from __future__ import annotations

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from src.config import Config
from src.sensors import DepthReading, RobotSensorHub


LABELS = ("clear", "obstacle", "stair_up", "stair_down")
LABEL_IDS = {label: index for index, label in enumerate(LABELS)}
FEATURE_COUNT = 67
PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATASET = PROJECT_ROOT / "out" / "tof_tinyml.csv"
DEFAULT_MODEL = PROJECT_ROOT / "firmware" / "esp32_bno055_usb" / "src" / "terrain_model_data.h"


def _trunc_div(value: int, divisor: int) -> int:
    return value // divisor if value >= 0 else -((-value) // divisor)


def extract_features(distances: tuple[int, ...]) -> np.ndarray:
    if len(distances) != 64:
        raise ValueError("VL53L5CX frame must contain 64 zones")
    clean = [value if 20 <= value <= 4000 else 4000 for value in distances]
    ordered = sorted(clean)
    frame_median = (ordered[31] + ordered[32]) // 2
    features = [max(-100, min(100, _trunc_div(value - frame_median, 20))) for value in clean]
    features.append(max(-100, min(100, frame_median // 20 - 100)))
    features.append(max(-100, min(100, min(clean) // 20 - 100)))
    row_medians = []
    for row in range(8):
        values = sorted(clean[row * 8 : (row + 1) * 8])
        row_medians.append((values[3] + values[4]) // 2)
    features.append(max(-100, min(100, (max(row_medians) - min(row_medians)) // 20)))
    return np.asarray(features, dtype=np.int16)


def collect(args: argparse.Namespace) -> int:
    settings = Config()
    hub = RobotSensorHub(
        port=args.port,
        baudrate=args.baudrate,
        timeout_s=settings.sensor_timeout_s,
        depth_timeout_s=settings.sensor_depth_timeout_s,
        use_imu=False,
        use_foot_fsr=False,
        use_depth=True,
    )
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    write_header = not output.exists() or output.stat().st_size == 0
    session = datetime.now().strftime("%Y%m%dT%H%M%S")
    deadline = time.monotonic() + max(1.0, args.seconds)
    last_timestamp = None
    rows = 0

    hub.open()
    print(f"[tof-tinyml] Collecting '{args.label}' from {hub.active_port or args.port}.")
    try:
        with output.open("a", newline="", encoding="ascii") as stream:
            writer = csv.writer(stream)
            if write_header:
                writer.writerow(["session", "label", "sensor_time_ms", *[f"z{i}" for i in range(64)]])
            while time.monotonic() < deadline:
                depth = hub.read().depth
                if depth is None or depth.sensor_time_ms == last_timestamp:
                    time.sleep(0.01)
                    continue
                last_timestamp = depth.sensor_time_ms
                writer.writerow([session, args.label, depth.sensor_time_ms, *depth.distances_mm])
                rows += 1
                if rows % 25 == 0:
                    print(f"[tof-tinyml] {rows} frames")
    finally:
        hub.close()
    print(f"[tof-tinyml] Saved {rows} frames to {output}")
    return 0 if rows else 2


def load_dataset(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    features = []
    labels = []
    sessions = []
    with path.open(newline="", encoding="ascii") as stream:
        for row in csv.DictReader(stream):
            label = row.get("label", "")
            if label not in LABEL_IDS:
                continue
            try:
                distances = tuple(int(row[f"z{i}"]) for i in range(64))
            except (KeyError, ValueError):
                continue
            features.append(extract_features(distances))
            labels.append(LABEL_IDS[label])
            sessions.append(row.get("session", "unknown"))
    if not features:
        raise RuntimeError(f"No valid ToF rows found in {path}")
    return np.stack(features), np.asarray(labels, dtype=np.uint8), np.asarray(sessions)


def fit_model(
    features: np.ndarray,
    labels: np.ndarray,
    prototypes_per_class: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    prototypes = []
    prototype_labels = []
    thresholds = []
    for label_id in range(len(LABELS)):
        samples = features[labels == label_id].astype(np.int32)
        if len(samples) < max(10, prototypes_per_class):
            raise RuntimeError(f"Class '{LABELS[label_id]}' needs at least 10 frames")
        unique_samples = np.unique(samples, axis=0)
        count = min(prototypes_per_class, len(unique_samples))
        class_mean = unique_samples.mean(axis=0)
        selected = [int(np.argmin(np.sum((unique_samples - class_mean) ** 2, axis=1)))]
        while len(selected) < count:
            chosen = unique_samples[selected]
            distances = np.min(
                np.sum((unique_samples[:, None, :] - chosen[None, :, :]) ** 2, axis=2),
                axis=1,
            )
            selected.append(int(np.argmax(distances)))
        centers = unique_samples[selected].copy()
        for _ in range(30):
            distances = np.sum((samples[:, None, :] - centers[None, :, :]) ** 2, axis=2)
            assignment = np.argmin(distances, axis=1)
            updated = centers.copy()
            for index in range(count):
                members = samples[assignment == index]
                if len(members):
                    updated[index] = np.rint(members.mean(axis=0)).astype(np.int32)
            if np.array_equal(updated, centers):
                break
            centers = updated

        distances = np.sum((samples[:, None, :] - centers[None, :, :]) ** 2, axis=2)
        assignment = np.argmin(distances, axis=1)
        for index, center in enumerate(centers):
            cluster_distances = distances[assignment == index, index]
            if not len(cluster_distances):
                continue
            threshold = max(100, int(np.percentile(cluster_distances, 97) * 1.25) + 1)
            prototypes.append(center.astype(np.int16))
            prototype_labels.append(label_id)
            thresholds.append(threshold)
    return (
        np.stack(prototypes),
        np.asarray(prototype_labels, dtype=np.uint8),
        np.asarray(thresholds, dtype=np.uint32),
    )


def predict_model(
    features: np.ndarray,
    prototypes: np.ndarray,
    prototype_labels: np.ndarray,
    thresholds: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    distances = np.sum(
        (features[:, None, :].astype(np.int32) - prototypes[None, :, :].astype(np.int32)) ** 2,
        axis=2,
    )
    best = np.argmin(distances, axis=1)
    predictions = prototype_labels[best].astype(np.int16)
    confidence = np.zeros(len(features), dtype=np.float64)
    for row, prototype in enumerate(best):
        best_distance = int(distances[row, prototype])
        threshold = int(thresholds[prototype])
        if best_distance > threshold:
            predictions[row] = -1
            continue
        other = distances[row, prototype_labels != prototype_labels[prototype]]
        second = int(other.min()) if len(other) else 0
        fit = 1.0 - best_distance / max(1, threshold)
        margin = 0.0 if second <= 0 else 1.0 - best_distance / second
        confidence[row] = max(0.0, min(0.99, 0.5 * fit + 0.5 * margin))
    return predictions, confidence


def write_model(
    path: Path,
    prototypes: np.ndarray,
    prototype_labels: np.ndarray,
    thresholds: np.ndarray,
) -> None:
    rows = ["    {" + ", ".join(str(int(value)) for value in prototype) + "}" for prototype in prototypes]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join(
            [
                "#pragma once",
                "",
                "#include <stdint.h>",
                "",
                "namespace terrain_model {",
                "",
                "constexpr bool kReady = true;",
                f"constexpr uint8_t kFeatureCount = {FEATURE_COUNT};",
                f"constexpr uint8_t kPrototypeCount = {len(prototypes)};",
                "constexpr int16_t kPrototypes[kPrototypeCount][kFeatureCount] = {",
                ",\n".join(rows),
                "};",
                "constexpr uint8_t kPrototypeLabels[kPrototypeCount] = {" +
                ", ".join(str(int(value)) for value in prototype_labels) + "};",
                "constexpr uint32_t kPrototypeThresholds[kPrototypeCount] = {" +
                ", ".join(str(int(value)) for value in thresholds) + "};",
                "",
                "}  // namespace terrain_model",
                "",
            ]
        ),
        encoding="ascii",
    )


def train(args: argparse.Namespace) -> int:
    features, labels, sessions = load_dataset(args.data.resolve())
    missing = [LABELS[index] for index in range(len(LABELS)) if not np.any(labels == index)]
    if missing:
        raise RuntimeError(f"Missing classes: {', '.join(missing)}")

    validation = np.zeros(len(labels), dtype=bool)
    for label_id in range(len(LABELS)):
        class_sessions = sorted(set(sessions[labels == label_id]))
        if len(class_sessions) >= 2:
            validation |= (labels == label_id) & (sessions == class_sessions[-1])
    if validation.any() and (~validation).any():
        model = fit_model(features[~validation], labels[~validation], args.prototypes)
        predicted, confidence = predict_model(features[validation], *model)
        accepted = confidence >= args.min_confidence
        accuracy = np.mean((predicted == labels[validation]) & accepted)
        print(
            f"[tof-tinyml] Session holdout: {accuracy * 100:.1f}% accepted-correct "
            f"({accepted.sum()}/{validation.sum()} accepted)"
        )
    else:
        print("[tof-tinyml] Add a second collection session per class for holdout validation.")

    model = fit_model(features, labels, args.prototypes)
    write_model(args.output.resolve(), *model)
    counts = ", ".join(
        f"{label}={int(np.sum(labels == label_id))}" for label_id, label in enumerate(LABELS)
    )
    print(f"[tof-tinyml] Trained {len(model[0])} prototypes from {counts}")
    print(f"[tof-tinyml] Wrote {args.output.resolve()}")
    return 0


def parse_args() -> argparse.Namespace:
    settings = Config()
    parser = argparse.ArgumentParser(description="Collect and train the VL53L5CX TinyML terrain model")
    commands = parser.add_subparsers(dest="command", required=True)

    collect_parser = commands.add_parser("collect", help="Append labeled 8x8 ToF frames")
    collect_parser.add_argument("--label", choices=LABELS, required=True)
    collect_parser.add_argument("--seconds", type=float, default=30.0)
    collect_parser.add_argument("--port", default=settings.sensor_port)
    collect_parser.add_argument("--baudrate", type=int, default=settings.sensor_baudrate)
    collect_parser.add_argument("--output", type=Path, default=DEFAULT_DATASET)
    collect_parser.set_defaults(handler=collect)

    train_parser = commands.add_parser("train", help="Train and export the firmware model")
    train_parser.add_argument("--data", type=Path, default=DEFAULT_DATASET)
    train_parser.add_argument("--output", type=Path, default=DEFAULT_MODEL)
    train_parser.add_argument("--prototypes", type=int, default=3)
    train_parser.add_argument("--min-confidence", type=float, default=0.60)
    train_parser.set_defaults(handler=train)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if hasattr(args, "prototypes"):
        args.prototypes = max(1, min(8, args.prototypes))
        args.min_confidence = max(0.0, min(1.0, args.min_confidence))
    return args.handler(args)


if __name__ == "__main__":
    raise SystemExit(main())
