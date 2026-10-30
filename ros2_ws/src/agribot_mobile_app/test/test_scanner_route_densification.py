from copy import deepcopy
import math

import numpy as np
import pytest

from agribot_mobile_app.scanner_route import densify_reference_route, ordered_reference_matches


def pose(x, y, yaw=0):
    return {"x": x, "y": y, "yaw": yaw}


def test_ordered_match_disambiguates_overlapping_return_leg():
    reference = np.array([[0, 0], [1, 0.05], [2, 0.05], [3, 0],
                          [3, 1], [2, 1], [1, 1], [0, 1],
                          [0, 0.05], [1, 0], [2, 0], [3, 0.05]])
    anchors = [[0, 0], [1, 0], [3, 1], [0, 1], [2, 0], [3, 0.05]]
    assert ordered_reference_matches(reference, anchors, 0.2) == [0, 1, 4, 7, 10, 11]


def test_manual_pose_and_heading_preserved_and_guides_are_real_samples():
    reference = np.array([[0, 0], [1, 0], [2, 0], [2, 1], [2, 2], [3, 2], [4, 2]])
    anchors = [pose(0, 0, 0.2), pose(2.02, 2, -1.7), pose(4, 2, 1.1)]
    original = deepcopy(anchors)
    expanded = densify_reference_route(reference, anchors, tolerance=0.1, maximum_spacing=1.1)
    assert anchors == original
    assert [expanded["poses"][i] for i in expanded["manual_anchor_pose_indices"]] == original
    assert expanded["source_sample_indices"] == list(range(7))
    assert expanded["poses"][0] == anchors[0] and expanded["poses"][-1] == anchors[-1]
    for i, point in enumerate(expanded["poses"]):
        if i not in expanded["manual_anchor_pose_indices"]:
            np.testing.assert_array_equal([point["x"], point["y"]], reference[expanded["source_sample_indices"][i]])
    assert expanded["poses"][3]["yaw"] == pytest.approx(math.pi / 2)


def test_hairpin_is_not_replaced_by_straight_interpolation():
    reference = np.array([[0, 0], [0, 1], [0, 2], [1, 2], [1, 1], [1, 0]])
    expanded = densify_reference_route(reference, [pose(0, 0), pose(1, 0)], tolerance=0.1)
    assert len(expanded["poses"]) >= 4
    assert any(point["y"] == 2 for point in expanded["poses"])


def test_bad_order_and_distant_anchors_are_rejected():
    with pytest.raises(ValueError, match="order"):
        ordered_reference_matches([[0, 0], [1, 0], [2, 0]], [[2, 0], [0, 0]], 0.1)
    with pytest.raises(ValueError, match="too far"):
        ordered_reference_matches([[0, 0], [1, 0]], [[0, 0], [20, 20]])


def test_limit_is_not_silently_relaxed_and_invalid_values_rejected():
    reference = np.column_stack([np.arange(20), np.zeros(20)])
    with pytest.raises(ValueError, match="point limit"):
        densify_reference_route(reference, [pose(0, 0), pose(19, 0)],
                                maximum_spacing=1, maximum_points=3)
    with pytest.raises(ValueError, match="positive"):
        densify_reference_route(reference, [pose(0, 0), pose(19, 0)], maximum_spacing=0)
    with pytest.raises(ValueError, match="finite"):
        densify_reference_route(reference, [pose(0, 0), pose(19, 0, float("nan"))])
