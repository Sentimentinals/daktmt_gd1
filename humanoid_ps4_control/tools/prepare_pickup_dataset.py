from __future__ import annotations

import argparse
import csv
import json
import random
import shutil
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path


CLASS_IDS = {
    "/m/02jnhm": 0,  # Tin can
    "/m/01226z": 1,  # Football
    "/m/02ctlc": 1,  # Cricket ball
    "/m/02rgn06": 1,  # Volleyball
    "/m/044r5d": 1,  # Golf ball
    "/m/0wdt60w": 1,  # Rugby ball
    "/m/05ctyq": 1,  # Tennis ball
}
CLASS_NAMES = ("can", "ball", "rubik_cube")
OPEN_IMAGES_URL = "https://open-images-dataset.s3.amazonaws.com/{split}/{image_id}.jpg"


def box_iou(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    x1 = max(a[0], b[0])
    y1 = max(a[1], b[1])
    x2 = min(a[2], b[2])
    y2 = min(a[3], b[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = (
        (a[2] - a[0]) * (a[3] - a[1])
        + (b[2] - b[0]) * (b[3] - b[1])
        - intersection
    )
    return intersection / union if union > 0.0 else 0.0


def read_open_images(path: Path, min_area: float) -> tuple[dict[str, list[str]], list[str]]:
    boxes: dict[str, list[tuple[int, float, float, float, float]]] = {}
    all_image_ids = set()
    target_image_ids = set()
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            image_id = row["ImageID"]
            all_image_ids.add(image_id)
            class_id = CLASS_IDS.get(row["LabelName"])
            if class_id is None:
                continue
            target_image_ids.add(image_id)
            if row["IsGroupOf"] != "0" or row["IsDepiction"] != "0":
                continue
            x1 = float(row["XMin"])
            x2 = float(row["XMax"])
            y1 = float(row["YMin"])
            y2 = float(row["YMax"])
            width = x2 - x1
            height = y2 - y1
            if width * height < min_area:
                continue
            candidate = (class_id, x1, y1, x2, y2)
            existing = boxes.setdefault(image_id, [])
            duplicate = any(
                class_id == item[0] and box_iou(candidate[1:], item[1:]) >= 0.90
                for item in existing
            )
            if not duplicate:
                existing.append(candidate)

    labels = {
        image_id: [
            (
                f"{class_id} {(x1 + x2) * 0.5:.6f} {(y1 + y2) * 0.5:.6f} "
                f"{x2 - x1:.6f} {y2 - y1:.6f}"
            )
            for class_id, x1, y1, x2, y2 in image_boxes
        ]
        for image_id, image_boxes in boxes.items()
    }
    return labels, sorted(all_image_ids - target_image_ids)


def download_image(url: str, destination: Path) -> bool:
    for attempt in range(3):
        try:
            request = urllib.request.Request(url, headers={"User-Agent": "pickup-dataset/1.0"})
            with urllib.request.urlopen(request, timeout=30) as response:
                destination.write_bytes(response.read())
            return True
        except Exception:
            if attempt == 2:
                return False
            time.sleep(0.5 * (attempt + 1))
    return False


def add_open_images(
    source_split: str,
    output_split: str,
    csv_path: Path,
    root: Path,
    background_count: int,
    min_area: float,
    seed: int,
    workers: int,
) -> dict[str, int]:
    labels, background_ids = read_open_images(csv_path, min_area)
    random.Random(seed).shuffle(background_ids)
    selected = dict(labels)
    for image_id in background_ids[:background_count]:
        selected[image_id] = []

    images_dir = root / "images" / output_split
    labels_dir = root / "labels" / output_split
    images_dir.mkdir(parents=True, exist_ok=True)
    labels_dir.mkdir(parents=True, exist_ok=True)
    jobs = {}
    with ThreadPoolExecutor(max_workers=workers) as executor:
        for image_id in selected:
            destination = images_dir / f"oi_{source_split}_{image_id}.jpg"
            url = OPEN_IMAGES_URL.format(split=source_split, image_id=image_id)
            jobs[executor.submit(download_image, url, destination)] = (image_id, destination)
        failed = 0
        for future in as_completed(jobs):
            image_id, destination = jobs[future]
            if not future.result():
                failed += 1
                destination.unlink(missing_ok=True)
                continue
            label_path = labels_dir / f"oi_{source_split}_{image_id}.txt"
            label_path.write_text("\n".join(selected[image_id]), encoding="ascii")
    return {"images": len(selected) - failed, "failed": failed}


def rubik_box(label_path: Path, padding: float) -> str | None:
    xs = []
    ys = []
    for line in label_path.read_text(encoding="utf-8").splitlines():
        values = line.split()[1:]
        if len(values) < 6 or len(values) % 2:
            continue
        coordinates = [float(value) for value in values]
        xs.extend(coordinates[0::2])
        ys.extend(coordinates[1::2])
    if not xs or not ys:
        return None
    x1 = max(0.0, min(xs) - padding)
    x2 = min(1.0, max(xs) + padding)
    y1 = max(0.0, min(ys) - padding)
    y2 = min(1.0, max(ys) + padding)
    return f"2 {(x1 + x2) * 0.5:.6f} {(y1 + y2) * 0.5:.6f} {x2 - x1:.6f} {y2 - y1:.6f}"


def add_rubik(source_root: Path, root: Path, padding: float) -> dict[str, int]:
    counts = {}
    for source_split, output_split in (("train", "train"), ("valid", "val"), ("test", "test")):
        images_dir = root / "images" / output_split
        labels_dir = root / "labels" / output_split
        images_dir.mkdir(parents=True, exist_ok=True)
        labels_dir.mkdir(parents=True, exist_ok=True)
        count = 0
        for source_image in sorted((source_root / source_split / "images").glob("*")):
            source_label = source_root / source_split / "labels" / f"{source_image.stem}.txt"
            if not source_label.is_file():
                continue
            label = rubik_box(source_label, padding)
            if label is None:
                continue
            name = f"rubik_{source_split}_{source_image.name}"
            shutil.copy2(source_image, images_dir / name)
            (labels_dir / f"{Path(name).stem}.txt").write_text(label, encoding="ascii")
            count += 1
        counts[output_split] = count
    return counts


def oversample_class(root: Path, split: str, class_id: int, extra_copies: int) -> int:
    images_dir = root / "images" / split
    labels_dir = root / "labels" / split
    copied = 0
    source_labels = [
        path for path in labels_dir.glob("*.txt") if not path.name.startswith("repeat_")
    ]
    for label_path in source_labels:
        lines = label_path.read_text(encoding="ascii").splitlines()
        contains_class = any(
            line.split(maxsplit=1)[0] == str(class_id) for line in lines if line.strip()
        )
        if not contains_class:
            continue
        matches = list(images_dir.glob(f"{label_path.stem}.*"))
        if len(matches) != 1:
            raise RuntimeError(f"Expected one image for {label_path.name}, found {len(matches)}")
        source_image = matches[0]
        for copy_index in range(extra_copies):
            prefix = f"repeat_c{class_id}_{copy_index + 1}_"
            shutil.copy2(source_image, images_dir / f"{prefix}{source_image.name}")
            shutil.copy2(label_path, labels_dir / f"{prefix}{label_path.name}")
            copied += 1
    return copied


def main() -> int:
    parser = argparse.ArgumentParser(description="Build the three-class Pickup YOLO dataset.")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--oi-validation", type=Path, required=True)
    parser.add_argument("--oi-test", type=Path, required=True)
    parser.add_argument("--rubik-root", type=Path, required=True)
    parser.add_argument("--backgrounds", type=int, default=50)
    parser.add_argument("--min-area", type=float, default=0.001)
    parser.add_argument("--rubik-padding", type=float, default=0.02)
    parser.add_argument("--can-extra-copies", type=int, default=1)
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()

    root = args.output.resolve()
    if root.exists() and any(root.iterdir()):
        raise RuntimeError(f"Output directory is not empty: {root}")
    root.mkdir(parents=True, exist_ok=True)

    stats = {
        "open_images_train": add_open_images(
            "test", "train", args.oi_test, root, args.backgrounds, args.min_area, 42, args.workers
        ),
        "open_images_val": add_open_images(
            "validation", "val", args.oi_validation, root, args.backgrounds, args.min_area, 43, args.workers
        ),
        "rubik": add_rubik(args.rubik_root, root, args.rubik_padding),
    }
    stats["can_oversampling"] = {
        "extra_images": oversample_class(root, "train", 0, max(0, args.can_extra_copies))
    }
    yaml = (
        f"path: {root.as_posix()}\n"
        "train: images/train\nval: images/val\ntest: images/test\n"
        "names:\n  0: can\n  1: ball\n  2: rubik_cube\n"
    )
    (root / "data.yaml").write_text(yaml, encoding="ascii")
    sources = {
        "classes": CLASS_NAMES,
        "sources": [
            {
                "name": "Open Images V7",
                "url": "https://storage.googleapis.com/openimages/web/download_v7.html",
                "classes": [
                    "Tin can", "Football", "Cricket ball", "Volleyball",
                    "Golf ball", "Rugby ball", "Tennis ball",
                ],
            },
            {
                "name": "Rubiks Cube Segmentation",
                "url": "https://huggingface.co/datasets/seandavidreed/rubiks_cube_segmentation",
                "license": "CC BY 4.0",
            },
        ],
        "stats": stats,
    }
    (root / "sources.json").write_text(json.dumps(sources, indent=2), encoding="ascii")
    print(json.dumps(stats, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
