"""Build a Nav2 costmap-filter mask from a semantic A* route."""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np

from .catalog import GridData


class RouteCostmapError(RuntimeError):
    pass


@dataclass(frozen=True)
class RouteCostmapPolicy:
    core_half_width_m: float = 0.485974
    gradient_width_m: float = 2.0
    maximum_preference_cost: int = 80

    def validate(self) -> None:
        if (
            not math.isfinite(self.core_half_width_m)
            or self.core_half_width_m < 0.0
        ):
            raise RouteCostmapError("route core half width must be non-negative")
        if (
            not math.isfinite(self.gradient_width_m)
            or self.gradient_width_m <= 0.0
        ):
            raise RouteCostmapError("route gradient width must be positive")
        if not 1 <= self.maximum_preference_cost <= 99:
            raise RouteCostmapError("route preference cost must be between 1 and 99")


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
    policy: RouteCostmapPolicy,
) -> np.ndarray:
    policy.validate()
    if (
        grid.width <= 0
        or grid.height <= 0
        or not math.isfinite(grid.resolution)
        or grid.resolution <= 0.0
    ):
        raise RouteCostmapError("map geometry is invalid")
    if not isinstance(centerline, list) or len(centerline) < 2:
        raise RouteCostmapError("semantic route centerline needs at least two points")

    route_pixels = np.zeros((grid.height, grid.width), dtype=np.uint8)
    pixels = []
    for point in centerline:
        if not isinstance(point, dict):
            raise RouteCostmapError("semantic route centerline point is invalid")
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
        raise RouteCostmapError("semantic route does not intersect the active map")

    distance_m = cv2.distanceTransform(
        (route_pixels == 0).astype(np.uint8), cv2.DIST_L2, 5
    ) * grid.resolution
    normalized = np.clip(
        (distance_m - policy.core_half_width_m) / policy.gradient_width_m,
        0.0,
        1.0,
    )
    mask = np.rint(normalized * policy.maximum_preference_cost).astype(np.uint8)

    if not isinstance(avoidance_zones, list):
        raise RouteCostmapError("semantic avoidance zones must be a list")
    selectors = set()
    for zone in avoidance_zones:
        if not isinstance(zone, dict):
            raise RouteCostmapError("semantic avoidance zone is invalid")
        selector = str(zone.get("selector", ""))
        if not selector or selector in selectors:
            raise RouteCostmapError("semantic avoidance selectors must be unique")
        selectors.add(selector)
        radius = _finite(zone.get("radius_m"), "avoidance radius")
        if radius < 0.0:
            raise RouteCostmapError("semantic avoidance radius must be non-negative")
        center = world_to_grid(
            grid,
            _finite(zone.get("x"), "avoidance x"),
            _finite(zone.get("y"), "avoidance y"),
        )
        cv2.circle(
            mask,
            center,
            max(1, int(math.ceil(radius / grid.resolution))),
            100,
            -1,
            cv2.LINE_8,
        )
    return mask


def neutral_route_costmap(grid: GridData) -> np.ndarray:
    if grid.width <= 0 or grid.height <= 0:
        raise RouteCostmapError("map geometry is invalid")
    return np.zeros((grid.height, grid.width), dtype=np.uint8)
