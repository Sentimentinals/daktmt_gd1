from __future__ import annotations

from pathlib import Path
from typing import Sequence

import cv2
import numpy as np
from ai_edge_litert.interpreter import Interpreter

from .vision_control import Landmark


MOVENET_TO_BODY = {
    0: 0,    # nose
    3: 7,    # left ear
    4: 8,    # right ear
    5: 11,   # left shoulder
    6: 12,   # right shoulder
    7: 13,   # left elbow
    8: 14,   # right elbow
    9: 15,   # left wrist
    10: 16,  # right wrist
    11: 23,  # left hip
    12: 24,  # right hip
    13: 25,  # left knee
    14: 26,  # right knee
    15: 27,  # left ankle
    16: 28,  # right ankle
}

BODY_CONNECTIONS = (
    (0, 7), (0, 8), (7, 11), (8, 12), (11, 12),
    (11, 13), (13, 15), (12, 14), (14, 16),
    (11, 23), (12, 24), (23, 24),
    (23, 25), (25, 27), (24, 26), (26, 28),
)


class MoveNetPoseEstimator:
    """Run MoveNet Lightning with LiteRT and expose body-controller landmarks."""

    def __init__(self, model_path: str, num_threads: int = 4) -> None:
        path = Path(model_path).expanduser()
        if not path.is_file():
            raise RuntimeError(
                f"MoveNet model not found: {path}. Download the INT8 model before starting Camera Mimic."
            )
        self.interpreter = Interpreter(model_path=str(path), num_threads=max(1, num_threads))
        self.interpreter.allocate_tensors()
        self.input = self.interpreter.get_input_details()[0]
        self.output = self.interpreter.get_output_details()[0]
        shape = self.input["shape"]
        if len(shape) != 4 or int(shape[0]) != 1 or int(shape[3]) != 3:
            raise RuntimeError(f"Unsupported MoveNet input shape: {tuple(int(value) for value in shape)}")
        self.input_height = int(shape[1])
        self.input_width = int(shape[2])
        self.canvas = np.zeros((self.input_height, self.input_width, 3), dtype=np.uint8)

    def infer(self, frame_rgb: np.ndarray) -> list[Landmark]:
        height, width = frame_rgb.shape[:2]
        scale = min(self.input_width / width, self.input_height / height)
        resized_width = max(1, round(width * scale))
        resized_height = max(1, round(height * scale))
        resized = cv2.resize(frame_rgb, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
        offset_x = (self.input_width - resized_width) // 2
        offset_y = (self.input_height - resized_height) // 2
        self.canvas.fill(0)
        self.canvas[offset_y:offset_y + resized_height, offset_x:offset_x + resized_width] = resized

        tensor = self._quantize(self.canvas, self.input)
        self.interpreter.set_tensor(self.input["index"], tensor[np.newaxis, ...])
        self.interpreter.invoke()
        output = self.interpreter.get_tensor(self.output["index"])
        output = self._dequantize(output, self.output)
        if output.size != 51:
            raise RuntimeError(f"Unsupported MoveNet output shape: {output.shape}")
        keypoints = output.reshape(17, 3)

        landmarks = [Landmark(0.0, 0.0, 0.0, 0.0) for _ in range(29)]
        for move_index, body_index in MOVENET_TO_BODY.items():
            y_value, x_value, score = keypoints[move_index]
            x_pixel = (float(x_value) * self.input_width - offset_x) / scale
            y_pixel = (float(y_value) * self.input_height - offset_y) / scale
            landmarks[body_index] = Landmark(
                max(0.0, min(1.0, x_pixel / width)),
                max(0.0, min(1.0, y_pixel / height)),
                0.0,
                max(0.0, min(1.0, float(score))),
            )
        return landmarks

    @staticmethod
    def draw(frame: np.ndarray, landmarks: Sequence[Landmark], confidence: float) -> None:
        height, width = frame.shape[:2]
        for first, second in BODY_CONNECTIONS:
            a, b = landmarks[first], landmarks[second]
            if a.visibility >= confidence and b.visibility >= confidence:
                cv2.line(
                    frame,
                    (round(a.x * width), round(a.y * height)),
                    (round(b.x * width), round(b.y * height)),
                    (80, 220, 150),
                    2,
                    cv2.LINE_AA,
                )
        for body_index in MOVENET_TO_BODY.values():
            point = landmarks[body_index]
            if point.visibility >= confidence:
                cv2.circle(
                    frame,
                    (round(point.x * width), round(point.y * height)),
                    4,
                    (245, 190, 72),
                    -1,
                    cv2.LINE_AA,
                )

    @staticmethod
    def _quantize(image: np.ndarray, detail: dict) -> np.ndarray:
        dtype = detail["dtype"]
        if np.issubdtype(dtype, np.floating):
            return image.astype(dtype)
        scale, zero_point = detail.get("quantization", (0.0, 0))
        if scale:
            limits = np.iinfo(dtype)
            values = np.rint(image.astype(np.float32) / scale + zero_point)
            return np.clip(values, limits.min, limits.max).astype(dtype)
        return image.astype(dtype)

    @staticmethod
    def _dequantize(tensor: np.ndarray, detail: dict) -> np.ndarray:
        if np.issubdtype(tensor.dtype, np.floating):
            return tensor.astype(np.float32, copy=False)
        scale, zero_point = detail.get("quantization", (0.0, 0))
        if scale:
            return (tensor.astype(np.float32) - zero_point) * scale
        return tensor.astype(np.float32)
