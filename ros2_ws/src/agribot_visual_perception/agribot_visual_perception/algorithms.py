"""OpenCV visual algorithms with a common frame-in/frame-out interface."""

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


SUPPORTED_MODES = ("canny", "orb", "optical_flow")


@dataclass
class VisualResult:
    image: np.ndarray
    metric_name: str
    metric_value: float


class VisualProcessor:
    """Processes frames without publishing into the navigation control chain."""

    def __init__(self, mode: str) -> None:
        if mode not in SUPPORTED_MODES:
            raise ValueError(f"Unsupported visual mode: {mode}")
        self.mode = mode
        self._orb = cv2.ORB_create(nfeatures=800) if mode == "orb" else None
        self._previous_gray: Optional[np.ndarray] = None

    def process(self, image: np.ndarray) -> VisualResult:
        if image is None or image.size == 0:
            raise ValueError("Input image is empty")
        bgr = self._as_bgr(image)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if self.mode == "canny":
            return self._process_canny(bgr, gray)
        if self.mode == "orb":
            return self._process_orb(bgr, gray)
        return self._process_optical_flow(bgr, gray)

    @staticmethod
    def _as_bgr(image: np.ndarray) -> np.ndarray:
        if image.ndim == 2:
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"Unsupported image shape: {image.shape}")
        return image.copy()

    @staticmethod
    def _process_canny(bgr: np.ndarray, gray: np.ndarray) -> VisualResult:
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.2)
        edges = cv2.Canny(blurred, 60, 150)
        annotated = (bgr.astype(np.float32) * 0.55).astype(np.uint8)
        annotated[edges > 0] = (0, 0, 255)
        return VisualResult(annotated, "edge_pixels", float(np.count_nonzero(edges)))

    def _process_orb(self, bgr: np.ndarray, gray: np.ndarray) -> VisualResult:
        keypoints = self._orb.detect(gray, None)
        annotated = cv2.drawKeypoints(
            bgr,
            keypoints,
            None,
            color=(0, 255, 0),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )
        return VisualResult(annotated, "keypoints", float(len(keypoints)))

    def _process_optical_flow(
        self, bgr: np.ndarray, gray: np.ndarray
    ) -> VisualResult:
        previous = self._previous_gray
        self._previous_gray = gray
        if previous is None or previous.shape != gray.shape:
            cv2.putText(
                bgr,
                "optical flow: waiting for next frame",
                (16, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            return VisualResult(bgr, "mean_flow_px", 0.0)

        flow = cv2.calcOpticalFlowFarneback(
            previous, gray, None, 0.5, 3, 21, 3, 5, 1.2, 0
        )
        magnitude = np.linalg.norm(flow, axis=2)
        annotated = bgr.copy()
        step = max(16, min(gray.shape) // 18)
        for y in range(step // 2, gray.shape[0], step):
            for x in range(step // 2, gray.shape[1], step):
                dx, dy = flow[y, x]
                if dx * dx + dy * dy < 0.25:
                    continue
                end = (int(round(x + dx)), int(round(y + dy)))
                cv2.arrowedLine(
                    annotated, (x, y), end, (0, 255, 255), 1, cv2.LINE_AA, 0, 0.3
                )
        return VisualResult(annotated, "mean_flow_px", float(np.mean(magnitude)))
