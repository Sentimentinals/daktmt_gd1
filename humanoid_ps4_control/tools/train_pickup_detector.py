from __future__ import annotations

import argparse
import json
import os
import shutil
import time
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Train and export the Pickup object detector.")
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--runs", type=Path, required=True)
    parser.add_argument("--name", default="pickup_objects")
    parser.add_argument("--model", default="yolo11n.pt")
    parser.add_argument("--epochs", type=int, default=120)
    parser.add_argument("--imgsz", type=int, default=416)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default="0")
    parser.add_argument("--workers", type=int, default=4)
    args = parser.parse_args()

    data_yaml = (args.data / "data.yaml").resolve()
    runs = args.runs.resolve()
    runs.mkdir(parents=True, exist_ok=True)
    config_root = runs / ".ultralytics"
    config_root.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("YOLO_CONFIG_DIR", str(config_root))

    from ultralytics import YOLO

    model = YOLO(args.model)
    model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        project=str(runs),
        name=args.name,
        exist_ok=True,
        cache="disk",
        patience=25,
        close_mosaic=10,
        seed=42,
        deterministic=True,
        plots=True,
    )
    epochs_completed = int(model.trainer.epoch + 1)
    best = Path(model.trainer.best)
    trained = YOLO(str(best))
    metrics = trained.val(data=str(data_yaml), imgsz=args.imgsz, device=args.device, plots=True)
    exported = Path(
        trained.export(
            format="onnx",
            imgsz=args.imgsz,
            opset=12,
            simplify=True,
            dynamic=False,
        )
    )

    import cv2
    import numpy as np

    net = cv2.dnn.readNetFromONNX(str(exported))
    sample = np.zeros((1, 3, args.imgsz, args.imgsz), dtype=np.float32)
    net.setInput(sample)
    output_shape = tuple(int(value) for value in net.forward().shape)
    if len(output_shape) != 3 or 4 + len(metrics.names) not in output_shape:
        raise RuntimeError(f"Unexpected ONNX output shape: {output_shape}")
    started = time.perf_counter()
    benchmark_runs = 10
    for _ in range(benchmark_runs):
        net.setInput(sample)
        net.forward()
    opencv_inference_ms = (time.perf_counter() - started) * 1000.0 / benchmark_runs

    per_class = {}
    for class_id, class_name in metrics.names.items():
        precision, recall, map50, map50_95 = metrics.class_result(class_id)
        per_class[class_name] = {
            "precision": float(precision),
            "recall": float(recall),
            "map50": float(map50),
            "map50_95": float(map50_95),
        }

    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(exported, output)
    summary = {
        "model": args.model,
        "image_size": args.imgsz,
        "epochs_requested": args.epochs,
        "epochs_completed": epochs_completed,
        "map50": float(metrics.box.map50),
        "map50_95": float(metrics.box.map),
        "precision": float(metrics.box.mp),
        "recall": float(metrics.box.mr),
        "classes": ["can", "ball", "rubik_cube"],
        "per_class": per_class,
        "onnx_output_shape": output_shape,
        "opencv_inference_ms": round(opencv_inference_ms, 2),
    }
    output.with_suffix(".json").write_text(json.dumps(summary, indent=2), encoding="ascii")
    print(json.dumps(summary, indent=2))
    print(f"Exported: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
