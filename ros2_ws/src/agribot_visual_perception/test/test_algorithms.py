from pathlib import Path

import cv2
import numpy as np
import pytest

from agribot_visual_perception.algorithms import VisualProcessor


def synthetic_frame():
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    cv2.rectangle(image, (20, 25), (80, 90), (255, 255, 255), 3)
    return image


class FakeBackend:
    def __init__(self, objects):
        self.objects = objects

    def infer(self, image):
        annotated = image.copy()
        annotated[0:4, 0:4] = (0, 255, 0)
        return annotated, self.objects


@pytest.mark.parametrize(
    ("mode", "metric_name"),
    [
        ("object_detection", "detected_objects"),
        ("instance_segmentation", "segmented_instances"),
        ("pose_estimation", "detected_poses"),
    ],
)
def test_every_ai_algorithm_returns_image_and_structured_results(mode, metric_name):
    objects = [
        {
            "class_id": 0,
            "class_name": "person",
            "confidence": 0.91,
            "bbox_xyxy": [10.0, 15.0, 80.0, 110.0],
        }
    ]
    processor = VisualProcessor(mode, inference_backend=FakeBackend(objects))
    result = processor.process(synthetic_frame())
    assert result.image.shape == (120, 160, 3)
    assert result.image.dtype == np.uint8
    assert result.metric_name == metric_name
    assert result.metric_value == 1.0
    assert result.objects == objects


def test_default_model_directory_is_outside_the_repository(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    path = VisualProcessor.resolve_model_path("object_detection", "", "")
    assert path.parent == Path(tmp_path) / ".local/share/agribot/vision_models"


def test_unknown_visual_algorithm_is_rejected():
    with pytest.raises(ValueError):
        VisualProcessor("unknown", inference_backend=FakeBackend([]))
