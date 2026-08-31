from __future__ import annotations

import argparse
import json
import os
import shutil
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Train and export the stair detector.")
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("deploy/models/stair_detector.onnx"))
    parser.add_argument("--runs", type=Path, default=Path("out/stair_training"))
    parser.add_argument("--model", default="yolo11n.pt")
    parser.add_argument("--epochs", type=int, default=120)
    parser.add_argument("--imgsz", type=int, default=416)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default="0")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--freeze", type=int, default=0)
    args = parser.parse_args()

    data_yaml = args.data.resolve()
    if data_yaml.is_dir():
        data_yaml /= "data.yaml"
    if not data_yaml.is_file():
        raise FileNotFoundError(f"YOLO data.yaml not found: {data_yaml}")

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
        name="stair_detector",
        exist_ok=True,
        cache="disk",
        patience=25,
        close_mosaic=10,
        seed=42,
        deterministic=True,
        plots=True,
        freeze=args.freeze,
        degrees=8.0,
        scale=0.3,
        mosaic=0.5,
    )
    best = Path(model.trainer.best)
    trained = YOLO(str(best))
    metrics = trained.val(
        data=str(data_yaml), imgsz=args.imgsz, device=args.device,
        workers=args.workers, plots=True, project=str(runs), name="validation",
    )
    import yaml

    data_config = yaml.safe_load(data_yaml.read_text(encoding="utf-8"))
    test_metrics = None
    if data_config.get("test"):
        test_metrics = trained.val(
            data=str(data_yaml), split="test", imgsz=args.imgsz,
            device=args.device, workers=args.workers, plots=True,
            project=str(runs), name="held_out_test",
        )
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
    net.setInput(np.zeros((1, 3, args.imgsz, args.imgsz), dtype=np.float32))
    prediction = net.forward()
    if prediction.ndim != 3 or prediction.shape[1] != 4 + len(metrics.names):
        raise RuntimeError(f"Unsupported stair ONNX output: {prediction.shape}")
    if not np.isfinite(prediction).all():
        raise RuntimeError("Stair ONNX produced non-finite predictions")

    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(exported, output)
    classes = [str(metrics.names[index]) for index in sorted(metrics.names)]
    summary = {
        "model": args.model,
        "image_size": args.imgsz,
        "epochs_requested": args.epochs,
        "epochs_completed": int(model.trainer.epoch + 1),
        "map50": float(metrics.box.map50),
        "map50_95": float(metrics.box.map),
        "precision": float(metrics.box.mp),
        "recall": float(metrics.box.mr),
        "classes": classes,
        "onnx_output_shape": list(prediction.shape),
        "freeze": args.freeze,
    }
    if test_metrics is not None:
        summary["held_out_test"] = {
            "map50": float(test_metrics.box.map50),
            "map50_95": float(test_metrics.box.map),
            "precision": float(test_metrics.box.mp),
            "recall": float(test_metrics.box.mr),
        }
    provenance = data_yaml.with_name("provenance.json")
    if provenance.is_file():
        summary["dataset"] = json.loads(provenance.read_text(encoding="utf-8"))
    output.with_suffix(".json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"Exported: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
