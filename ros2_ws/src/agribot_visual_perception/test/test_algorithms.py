import cv2
import numpy as np
import pytest

from agribot_visual_perception.algorithms import VisualProcessor


def synthetic_frame(offset=0):
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    cv2.rectangle(image, (20 + offset, 25), (80 + offset, 90), (255, 255, 255), 3)
    cv2.circle(image, (115 + offset, 55), 16, (180, 180, 180), -1)
    return image


@pytest.mark.parametrize("mode", ["canny", "orb", "optical_flow"])
def test_every_visual_algorithm_returns_an_annotated_bgr_frame(mode):
    processor = VisualProcessor(mode)
    result = processor.process(synthetic_frame())
    assert result.image.shape == (120, 160, 3)
    assert result.image.dtype == np.uint8
    assert result.metric_value >= 0.0


def test_optical_flow_uses_consecutive_frames():
    processor = VisualProcessor("optical_flow")
    first = processor.process(synthetic_frame())
    second = processor.process(synthetic_frame(offset=5))
    assert first.metric_value == 0.0
    assert second.metric_name == "mean_flow_px"
    assert second.metric_value > 0.0


def test_unknown_visual_algorithm_is_rejected():
    with pytest.raises(ValueError):
        VisualProcessor("unknown")
