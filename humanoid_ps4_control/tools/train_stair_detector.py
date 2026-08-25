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
    )
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

    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(exported, output)
    classes = [str(metrics.names[index]) for index in sorted(metrics.names)]
    summary = {
        "model": args.model,
        "image_size": args.imgsz,
        "epochs_completed": int(model.trainer.epoch + 1),
        "map50": float(metrics.box.map50),
        "map50_95": float(metrics.box.map),
        "precision": float(metrics.box.mp),
        "recall": float(metrics.box.mr),
        "classes": classes,
    }
    output.with_suffix(".json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"Exported: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
