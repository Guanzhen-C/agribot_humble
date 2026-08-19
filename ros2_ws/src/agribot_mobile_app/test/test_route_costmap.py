import math

import numpy as np
import pytest

from agribot_mobile_app.catalog import GridData
from agribot_mobile_app.route_costmap import (
    RouteCostmapError,
    build_keepout_costmap,
    neutral_route_costmap,
    world_to_grid,
)


def grid(yaw=0.0):
    return GridData(
        width=20,
        height=20,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=yaw,
        data=bytes(400),
    )


def test_empty_semantic_task_does_not_change_the_original_costmap():
    mask = build_keepout_costmap(grid(), [])

    assert mask.dtype == np.uint8
    assert np.count_nonzero(mask) == 0


def test_semantic_avoidance_zone_is_the_only_lethal_area():
    mask = build_keepout_costmap(
        grid(),
        [
            {
                "selector": "place_blocked",
                "x": 10.0,
                "y": 10.0,
                "radius_m": 2.0,
            }
        ],
    )

    assert mask[10, 10] == 100
    assert mask[10, 12] == 100
    assert mask[10, 13] == 0
    assert set(np.unique(mask)) == {0, 100}


def test_world_to_grid_supports_rotated_map_origins_and_neutral_mask():
    rotated = grid(math.pi / 2.0)
    assert world_to_grid(rotated, 0.0, 3.0) == (3, 0)
    assert np.count_nonzero(neutral_route_costmap(rotated)) == 0


def test_duplicate_avoidance_ids_are_rejected():
    with pytest.raises(RouteCostmapError, match="unique"):
        build_keepout_costmap(
            grid(),
            [
                {"selector": "same", "x": 2.0, "y": 2.0, "radius_m": 1.0},
                {"selector": "same", "x": 3.0, "y": 3.0, "radius_m": 1.0},
            ],
        )
