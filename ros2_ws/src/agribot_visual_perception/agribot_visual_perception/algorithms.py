"""AI visual inference behind a common frame-in/result-out interface."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np


SUPPORTED_MODES = ("object_detection", "instance_segmentation", "pose_estimation")
DEFAULT_MODEL_FILES = {
    "object_detection": "yolo26n.pt",
    "instance_segmentation": "yolo26n-seg.pt",
    "pose_estimation": "yolo26n-pose.pt",
}
METRIC_NAMES = {
    "object_detection": "detected_objects",
    "instance_segmentation": "segmented_instances",
    "pose_estimation": "detected_poses",
}


@dataclass
class VisualResult:
    image: np.ndarray
    metric_name: str
    metric_value: float
    objects: List[Dict[str, Any]]


class UltralyticsBackend:
    """Thin adapter that keeps the ROS node independent from model APIs."""

    def __init__(
        self,
        mode: str,
        model_path: Path,
        device: str,
        confidence: float,
        image_size: int,
    ) -> None:
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "Ultralytics is not installed. Run setup_ai_runtime.sh first."
            ) from exc

        if not model_path.is_file():
            raise FileNotFoundError(
                f"AI model not found: {model_path}. "
                "Run setup_ai_runtime.sh first."
            )
        self.mode = mode
        self.model = YOLO(str(model_path))
        self.device = device or None
        self.confidence = confidence
        self.image_size = image_size

    def infer(self, image: np.ndarray) -> tuple[np.ndarray, List[Dict[str, Any]]]:
        results = self.model.predict(
            source=image,
            conf=self.confidence,
            imgsz=self.image_size,
            device=self.device,
            verbose=False,
        )
        if not results:
            return image.copy(), []
        result = results[0]
        return result.plot(), self._serialize_result(result)

    def _serialize_result(self, result: Any) -> List[Dict[str, Any]]:
        boxes = result.boxes
        if boxes is None:
            return []
        xyxy = boxes.xyxy.detach().cpu().numpy()
        confidences = boxes.conf.detach().cpu().numpy()
        class_ids = boxes.cls.detach().cpu().numpy().astype(int)
        names = result.names
        objects: List[Dict[str, Any]] = []
        for index, (bounds, score, class_id) in enumerate(
            zip(xyxy, confidences, class_ids)
        ):
            item: Dict[str, Any] = {
                "class_id": int(class_id),
                "class_name": str(names.get(int(class_id), class_id)),
                "confidence": round(float(score), 5),
                "bbox_xyxy": [round(float(value), 2) for value in bounds],
            }
            if self.mode == "instance_segmentation" and result.masks is not None:
                polygons = result.masks.xy
                if index < len(polygons):
                    polygon = np.asarray(polygons[index])
                    stride = max(1, len(polygon) // 64)
                    item["mask_polygon"] = [
                        [round(float(x), 2), round(float(y), 2)]
                        for x, y in polygon[::stride]
                    ]
            if self.mode == "pose_estimation" and result.keypoints is not None:
                coordinates = result.keypoints.xy[index].detach().cpu().numpy()
                keypoint_confidence: Optional[np.ndarray] = None
                if result.keypoints.conf is not None:
                    keypoint_confidence = (
                        result.keypoints.conf[index].detach().cpu().numpy()
                    )
                keypoints = []
                for keypoint_index, (x, y) in enumerate(coordinates):
                    point = {"x": round(float(x), 2), "y": round(float(y), 2)}
                    if keypoint_confidence is not None:
                        point["confidence"] = round(
                            float(keypoint_confidence[keypoint_index]), 5
                        )
                    keypoints.append(point)
                item["keypoints"] = keypoints
            objects.append(item)
        return objects


class VisualProcessor:
    """Runs one AI model without publishing into the motion-control chain."""

    def __init__(
        self,
        mode: str,
        model_dir: str = "",
        model_path: str = "",
        device: str = "",
        confidence: float = 0.35,
        image_size: int = 640,
        inference_backend: Optional[Any] = None,
    ) -> None:
        if mode not in SUPPORTED_MODES:
            raise ValueError(f"Unsupported visual mode: {mode}")
        self.mode = mode
        resolved_path = self.resolve_model_path(mode, model_dir, model_path)
        self.backend = inference_backend or UltralyticsBackend(
            mode,
            resolved_path,
            device,
            min(max(confidence, 0.0), 1.0),
            max(32, image_size),
        )

    @staticmethod
    def resolve_model_path(mode: str, model_dir: str, model_path: str) -> Path:
        if model_path:
            return Path(model_path).expanduser().resolve()
        directory = (
            Path(model_dir).expanduser()
            if model_dir
            else Path.home() / ".local" / "share" / "agribot" / "vision_models"
        )
        return (directory / DEFAULT_MODEL_FILES[mode]).resolve()

    def process(self, image: np.ndarray) -> VisualResult:
        if image is None or image.size == 0:
            raise ValueError("Input image is empty")
        bgr = self._as_bgr(image)
        annotated, objects = self.backend.infer(bgr)
        annotated = self._as_bgr(annotated)
        return VisualResult(
            annotated,
            METRIC_NAMES[self.mode],
            float(len(objects)),
            objects,
        )

    @staticmethod
    def _as_bgr(image: np.ndarray) -> np.ndarray:
        if image.ndim == 2:
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"Unsupported image shape: {image.shape}")
        if image.dtype != np.uint8:
            return np.clip(image, 0, 255).astype(np.uint8)
        return image.copy()
