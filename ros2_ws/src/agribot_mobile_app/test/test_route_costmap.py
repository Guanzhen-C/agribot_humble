import math

import numpy as np
import pytest

from agribot_mobile_app.catalog import GridData
from agribot_mobile_app.route_costmap import (
    RouteCorridorPolicy,
    RouteCostmapError,
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


def centerline():
    return [{"x": 2.5, "y": 10.5}, {"x": 17.5, "y": 10.5}]


def test_wide_corridor_is_free_and_outside_has_high_cost():
    mask = build_route_costmap(
        grid(),
        centerline(),
        [],
        RouteCorridorPolicy(
            half_width_m=2.0, transition_width_m=2.0, outside_cost=100
        ),
    )

    assert mask.dtype == np.uint8
    assert mask[10, 10] == 0
    assert mask[12, 10] == 0
    assert 45 <= mask[13, 10] <= 55
    assert mask[14, 10] == 100


def test_avoidance_cost_is_added_to_corridor_cost_and_saturates():
    zone = {
        "selector": "place_blocked",
        "x": 10.5,
        "y": 10.5,
        "influence_radius_m": 3.0,
        "decay_length_m": 1.0,
    }
    mask = build_route_costmap(
        grid(), centerline(), [zone], RouteCorridorPolicy()
    )

    assert mask[10, 10] == 100
    assert 35 <= mask[10, 11] <= 38
    assert mask[14, 10] == 100


def test_world_to_grid_supports_rotated_map_origins_and_neutral_mask():
    rotated = grid(math.pi / 2.0)
    assert world_to_grid(rotated, 0.0, 3.0) == (3, 0)
    assert np.count_nonzero(neutral_route_costmap(rotated)) == 0


def test_duplicate_avoidance_ids_are_rejected():
    zone = {
        "selector": "same",
        "x": 2.0,
        "y": 2.0,
        "influence_radius_m": 1.0,
        "decay_length_m": 0.5,
    }
    with pytest.raises(RouteCostmapError, match="unique"):
        build_route_costmap(
            grid(), centerline(), [zone, dict(zone)], RouteCorridorPolicy()
        )


@pytest.mark.parametrize(
    "policy",
    [
        RouteCorridorPolicy(half_width_m=0.0),
        RouteCorridorPolicy(transition_width_m=0.0),
        RouteCorridorPolicy(outside_cost=101),
    ],
)
def test_invalid_corridor_policy_is_rejected(policy):
    with pytest.raises(RouteCostmapError):
        build_route_costmap(grid(), centerline(), [], policy)
