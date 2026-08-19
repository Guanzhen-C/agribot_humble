"""Build semantic route-corridor and avoidance costs for Nav2."""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from .catalog import GridData


class RouteCostmapError(RuntimeError):
    pass


@dataclass(frozen=True)
class RouteCorridorPolicy:
    """Normalized costs used by the vehicle-side semantic cost layer."""

    half_width_m: float = 2.0
    transition_width_m: float = 1.0
    outside_cost: int = 100

    def validate(self) -> None:
        if not math.isfinite(self.half_width_m) or self.half_width_m <= 0.0:
            raise RouteCostmapError("route corridor half width must be positive")
        if (
            not math.isfinite(self.transition_width_m)
            or self.transition_width_m <= 0.0
        ):
            raise RouteCostmapError("route corridor transition width must be positive")
        if not 1 <= self.outside_cost <= 100:
            raise RouteCostmapError("route corridor outside cost must be between 1 and 100")


def _finite(value, description: str) -> float:
    if isinstance(value, bool):
        raise RouteCostmapError(f"{description} must be finite")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise RouteCostmapError(f"{description} must be finite") from error
    if not math.isfinite(result):
        raise RouteCostmapError(f"{description} must be finite")
    return result


def world_to_grid(grid: GridData, x: float, y: float) -> tuple[int, int]:
    dx = _finite(x, "world x") - grid.origin_x
    dy = _finite(y, "world y") - grid.origin_y
    cosine = math.cos(grid.origin_yaw)
    sine = math.sin(grid.origin_yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    return (
        int(math.floor(local_x / grid.resolution)),
        int(math.floor(local_y / grid.resolution)),
    )


def build_route_costmap(
    grid: GridData,
    centerline: list[dict],
    avoidance_zones: list[dict],
    policy: RouteCorridorPolicy,
) -> np.ndarray:
    """Return normalized additive costs for a wide A* route corridor."""
    policy.validate()
    if (
        grid.width <= 0
        or grid.height <= 0
        or not math.isfinite(grid.resolution)
        or grid.resolution <= 0.0
    ):
        raise RouteCostmapError("map geometry is invalid")
    if not isinstance(centerline, list) or len(centerline) < 2:
        raise RouteCostmapError("semantic A* centerline needs at least two points")
    if not isinstance(avoidance_zones, list):
        raise RouteCostmapError("semantic avoidance zones must be a list")

    route_pixels = np.zeros((grid.height, grid.width), dtype=np.uint8)
    pixels = []
    for point in centerline:
        if not isinstance(point, dict):
            raise RouteCostmapError("semantic A* centerline point is invalid")
        pixels.append(
            world_to_grid(
                grid,
                _finite(point.get("x"), "route x"),
                _finite(point.get("y"), "route y"),
            )
        )
    cv2.polylines(
        route_pixels,
        [np.asarray(pixels, dtype=np.int32).reshape((-1, 1, 2))],
        False,
        255,
        1,
        cv2.LINE_8,
    )
    if not np.any(route_pixels):
        raise RouteCostmapError("semantic A* centerline does not intersect the map")

    distance_m = cv2.distanceTransform(
        (route_pixels == 0).astype(np.uint8), cv2.DIST_L2, 5
    ) * grid.resolution
    transition = np.clip(
        (distance_m - policy.half_width_m) / policy.transition_width_m,
        0.0,
        1.0,
    )
    # Smoothstep avoids an abrupt cost derivative at both corridor boundaries.
    transition = transition * transition * (3.0 - 2.0 * transition)
    accumulated = transition * float(policy.outside_cost)

    selectors = set()
    for zone in avoidance_zones:
        if not isinstance(zone, dict):
            raise RouteCostmapError("semantic avoidance zone is invalid")
        selector = str(zone.get("selector", ""))
        if not selector or selector in selectors:
            raise RouteCostmapError("semantic avoidance selectors must be unique")
        selectors.add(selector)
        influence_radius = _finite(
            zone.get("influence_radius_m"), "avoidance influence radius"
        )
        decay_length = _finite(
            zone.get("decay_length_m"), "avoidance decay length"
        )
        if influence_radius <= 0.0 or decay_length <= 0.0:
            raise RouteCostmapError(
                "semantic avoidance influence radius and decay length must be positive"
            )
        center_x = _finite(zone.get("x"), "avoidance x")
        center_y = _finite(zone.get("y"), "avoidance y")

        # Work in the map-local frame so rotated OccupancyGrid origins remain valid.
        center_column, center_row = world_to_grid(grid, center_x, center_y)
        radius_cells = int(math.ceil(influence_radius / grid.resolution)) + 1
        column_min = max(0, center_column - radius_cells)
        column_max = min(grid.width, center_column + radius_cells + 1)
        row_min = max(0, center_row - radius_cells)
        row_max = min(grid.height, center_row + radius_cells + 1)
        if column_min >= column_max or row_min >= row_max:
            continue

        columns = np.arange(column_min, column_max, dtype=np.float64) + 0.5
        rows = np.arange(row_min, row_max, dtype=np.float64) + 0.5
        local_x = columns[None, :] * grid.resolution
        local_y = rows[:, None] * grid.resolution
        cosine = math.cos(grid.origin_yaw)
        sine = math.sin(grid.origin_yaw)
        world_x = grid.origin_x + cosine * local_x - sine * local_y
        world_y = grid.origin_y + sine * local_x + cosine * local_y
        distance = np.hypot(world_x - center_x, world_y - center_y)
        costs = np.where(
            distance <= influence_radius,
            100.0 * np.exp(-distance / decay_length),
            0.0,
        )
        accumulated[row_min:row_max, column_min:column_max] += costs

    return np.rint(np.clip(accumulated, 0.0, 100.0)).astype(np.uint8)


def neutral_route_costmap(grid: GridData) -> np.ndarray:
    if grid.width <= 0 or grid.height <= 0:
        raise RouteCostmapError("map geometry is invalid")
    return np.zeros((grid.height, grid.width), dtype=np.uint8)
