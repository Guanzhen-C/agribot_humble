"""Build a Nav2 costmap-filter mask containing semantic keepout zones only."""

from __future__ import annotations

import math

import cv2
import numpy as np

from .catalog import GridData


class RouteCostmapError(RuntimeError):
    pass


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


def build_keepout_costmap(
    grid: GridData, avoidance_zones: list[dict]
) -> np.ndarray:
    if (
        grid.width <= 0
        or grid.height <= 0
        or not math.isfinite(grid.resolution)
        or grid.resolution <= 0.0
    ):
        raise RouteCostmapError("map geometry is invalid")
    if not isinstance(avoidance_zones, list):
        raise RouteCostmapError("semantic avoidance zones must be a list")
    mask = neutral_route_costmap(grid)
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
