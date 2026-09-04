from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class YoloBox:
    box: tuple[int, int, int, int]
    confidence: float
    class_id: int
    area_ratio: float


def detect_yolo(
    cv2,
    net,
    frame,
    *,
    class_count: int,
    input_size: int,
    confidence: float,
    iou_threshold: float,
) -> tuple[YoloBox, ...]:
    height, width = frame.shape[:2]
    scale = min(input_size / width, input_size / height)
    resized_width = max(1, round(width * scale))
    resized_height = max(1, round(height * scale))
    resized = cv2.resize(frame, (resized_width, resized_height))
    pad_x = (input_size - resized_width) // 2
    pad_y = (input_size - resized_height) // 2
    padded = cv2.copyMakeBorder(
        resized,
        pad_y,
        input_size - resized_height - pad_y,
        pad_x,
        input_size - resized_width - pad_x,
        cv2.BORDER_CONSTANT,
        value=(114, 114, 114),
    )
    blob = cv2.dnn.blobFromImage(
        padded,
        scalefactor=1.0 / 255.0,
        size=(input_size, input_size),
        swapRB=True,
        crop=False,
    )
    net.setInput(blob)
    raw = net.forward()
    rows = raw[0]
    expected_columns = 4 + class_count
    if rows.ndim != 2:
        raise RuntimeError(f"Unsupported YOLO output shape: {raw.shape}")
    if rows.shape[0] == expected_columns:
        rows = rows.T
    if rows.shape[1] != expected_columns:
        raise RuntimeError(f"Unsupported YOLO output shape: {raw.shape}")

    boxes: list[list[int]] = []
    scores: list[float] = []
    class_ids: list[int] = []
    for row in rows:
        class_scores = row[4:]
        class_id = int(class_scores.argmax())
        score = float(class_scores[class_id])
        if score < confidence:
            continue
        center_x, center_y, box_width, box_height = (float(value) for value in row[:4])
        x = round((center_x - box_width * 0.5 - pad_x) / scale)
        y = round((center_y - box_height * 0.5 - pad_y) / scale)
        box_width = round(box_width / scale)
        box_height = round(box_height / scale)
        x1 = max(0, min(width - 1, x))
        y1 = max(0, min(height - 1, y))
        x2 = max(0, min(width, x + box_width))
        y2 = max(0, min(height, y + box_height))
        if x2 <= x1 or y2 <= y1:
            continue
        boxes.append([x1, y1, x2 - x1, y2 - y1])
        scores.append(score)
        class_ids.append(class_id)

    if not boxes:
        return ()
    indices = cv2.dnn.NMSBoxes(boxes, scores, confidence, iou_threshold)
    detections = []
    for raw_index in indices:
        index = int(raw_index[0]) if hasattr(raw_index, "__len__") else int(raw_index)
        x, y, box_width, box_height = boxes[index]
        detections.append(
            YoloBox(
                box=(x, y, x + box_width, y + box_height),
                confidence=scores[index],
                class_id=class_ids[index],
                area_ratio=(box_width * box_height) / float(max(1, width * height)),
            )
        )
    detections.sort(key=lambda item: (item.confidence, item.area_ratio), reverse=True)
    return tuple(detections)
