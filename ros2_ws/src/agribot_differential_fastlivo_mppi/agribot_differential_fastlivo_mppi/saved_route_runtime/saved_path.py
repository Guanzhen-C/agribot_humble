"""Validated saved paths and bounded, forward-only route progress."""

import hashlib
import json
import math
from pathlib import Path

import numpy as np

from .catalog import CatalogError, validated_identifier
from .route_preflight import footprint_cells


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class SavedPathStore:
    def __init__(self, catalog, config_path):
        self.catalog, self.config_path = catalog, Path(config_path)

    def load(self, map_id):
        try:
            return self._load(map_id)
        except (OSError, ValueError, KeyError, TypeError) as error:
            raise CatalogError(f"保存路径文件无效: {error}") from error

    def _load(self, map_id):
        base = self.catalog.map_base(map_id)
        file = Path(f"{base}_saved_paths.json")
        if not file.is_file():
            return {"routes": []}
        if file.stat().st_size > 16_000_000:
            raise CatalogError("已保存路径文件过大")
        document = json.loads(file.read_text())
        if document.get("schema_version") != 1 or document.get("map_id") != map_id:
            raise CatalogError("已保存路径与地图不匹配")
        expected = document["fingerprints"]
        files = {"pgm": base.with_suffix(".pgm"), "yaml": base.with_suffix(".yaml"),
                 "georeference": Path(f"{base}_georeference.yaml"), "nav2": self.config_path,
                 "primitives": self.config_path.parent / "motion_primitives/diff_5cm.json"}
        if any(expected.get(key) != sha256(path) for key, path in files.items()):
            raise CatalogError("地图或导航参数已改变，请重新验证保存路径")
        ids = set()
        for route in document["routes"]:
            identifier = validated_identifier(route["id"], "路径编号")
            poses, anchors = route["path"], route["anchor_path_indices"]
            if identifier in ids or not 2 <= len(poses) <= 50000 or not 2 <= len(anchors) <= 110:
                raise CatalogError("保存路径数量或编号无效")
            ids.add(identifier)
            if (anchors != sorted(set(anchors)) or anchors[0] < 0 or anchors[-1] >= len(poses)
                    or any(type(index) is not int for index in anchors)):
                raise CatalogError("路径点索引无效")
            if any(not all(type(p[k]) in (int, float) and math.isfinite(p[k])
                           for k in ("x", "y", "yaw")) for p in poses):
                raise CatalogError("保存路径位姿无效")
        return document

    def public(self, map_id):
        return {"map_id": map_id, "routes": [
            {"id": r["id"], "label": r["label"], "vehicle_type": r["vehicle_type"],
             "anchor_count": len(r["anchor_path_indices"])} for r in self.load(map_id)["routes"]]}

    def select(self, map_id, route_id, start):
        route = next((r for r in self.load(map_id)["routes"] if r["id"] == route_id), None)
        if route is None or type(start) is not int or not 1 <= start < len(route["anchor_path_indices"]):
            raise CatalogError("路径或起点无效；终点不能作为起点")
        return route, route["path"][route["anchor_path_indices"][start - 1]:]


def arc_lengths(path):
    xy = np.array([[p["x"], p["y"]] for p in path])
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]


def numbered_markers(route, start):
    return [{"number": index + 1,
             "x": route["path"][path_index]["x"],
             "y": route["path"][path_index]["y"]}
            for index, path_index in enumerate(route["anchor_path_indices"])
            if index >= start - 1]


def handover_path_index(route, start, waypoint=100):
    """Index in the selected suffix, never a dense-point or proximity trigger."""
    anchors = route["anchor_path_indices"]
    if len(anchors) < waypoint:
        return None
    return max(0, anchors[waypoint-1] - anchors[start-1])


def forward_index(path, pose, previous, window=3.0, arc=None):
    arc = arc_lengths(path) if arc is None else arc
    last = min(len(path), int(np.searchsorted(arc, arc[previous] + window)) + 1)
    return min(range(previous, last), key=lambda i: math.hypot(
        path[i]["x"] - pose["x"], path[i]["y"] - pose["y"]))


def blocked_ahead(path, index, grid, footprint, distance=3.0, arc=None,
                  outside_is_blocked=True):
    arc = arc_lengths(path) if arc is None else arc
    end = min(len(path) - 1, int(np.searchsorted(arc, arc[index] + distance)))
    sample = index
    while sample <= end:
        cells = footprint_cells(grid, path[sample], footprint)
        if ((cells["out_of_bounds"] and outside_is_blocked)
                or cells["occupied_cells"]):
            return True
        sample = max(sample + 1, int(np.searchsorted(arc, arc[sample] + 0.20)))
    return False


def rejoin_indices(path, index):
    arc = arc_lengths(path)
    return sorted(set(min(len(path) - 1, int(np.searchsorted(arc, arc[index] + distance)))
                      for distance in (4.0, 7.0, 10.0)))
