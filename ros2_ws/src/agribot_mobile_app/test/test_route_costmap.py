import math

import numpy as np
import pytest

from agribot_mobile_app.catalog import GridData
from agribot_mobile_app.route_costmap import (
    RouteCostmapError,
    RouteCostmapPolicy,
    build_route_costmap,
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


def test_astar_centerline_is_free_and_cost_rises_outward():
    mask = build_route_costmap(
        grid(),
        [{"x": 2.0, "y": 10.0}, {"x": 17.0, "y": 10.0}],
        [],
        RouteCostmapPolicy(
            core_half_width_m=0.5,
            gradient_width_m=4.0,
            maximum_preference_cost=80,
        ),
    )

    assert mask.dtype == np.uint8
    assert mask[10, 10] == 0
    assert 0 < mask[12, 10] < mask[14, 10]
    assert mask[16, 10] == 80


def test_semantic_avoidance_zone_overrides_route_with_lethal_cost():
    mask = build_route_costmap(
        grid(),
        [{"x": 2.0, "y": 10.0}, {"x": 17.0, "y": 10.0}],
        [
            {
                "selector": "place_blocked",
                "x": 10.0,
                "y": 10.0,
                "radius_m": 2.0,
            }
        ],
        RouteCostmapPolicy(),
    )

    assert mask[10, 10] == 100
    assert mask[10, 12] == 100
    assert mask[10, 13] < 100


def test_world_to_grid_supports_rotated_map_origins_and_neutral_mask():
    rotated = grid(math.pi / 2.0)
    assert world_to_grid(rotated, 0.0, 3.0) == (3, 0)
    assert np.count_nonzero(neutral_route_costmap(rotated)) == 0


def test_rejects_route_outside_map_and_duplicate_avoidance_ids():
    with pytest.raises(RouteCostmapError, match="does not intersect"):
        build_route_costmap(
            grid(),
            [{"x": -20.0, "y": -20.0}, {"x": -10.0, "y": -10.0}],
            [],
            RouteCostmapPolicy(),
        )
    with pytest.raises(RouteCostmapError, match="unique"):
        build_route_costmap(
            grid(),
            [{"x": 2.0, "y": 10.0}, {"x": 17.0, "y": 10.0}],
            [
                {"selector": "same", "x": 2.0, "y": 2.0, "radius_m": 1.0},
                {"selector": "same", "x": 3.0, "y": 3.0, "radius_m": 1.0},
            ],
            RouteCostmapPolicy(),
        )
