#!/usr/bin/env python3
"""Keep raw scanner waypoints and create an explicit endpoint-corrected alternative."""

import argparse
import copy
import hashlib
import json
from pathlib import Path

import yaml

from agribot_mobile_app.catalog import MapCatalog
from agribot_mobile_app.route_preflight import footprint_cells, nearest_free_endpoint


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_base", type=Path)
    parser.add_argument("params_file", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--route-id", required=True)
    parser.add_argument("--max-endpoint-shift", type=float, default=2.0)
    args = parser.parse_args()
    base = args.map_base.expanduser().resolve()
    if args.output.resolve() == Path(f"{base}_routes.json") or args.output.exists():
        parser.error("Choose a new output path; original route presets must be preserved")
    catalog = MapCatalog(base.parent)
    original = catalog.route_presets(base.name)
    route = next(item for item in original["routes"] if item["id"] == args.route_id)
    geometry = yaml.safe_load(base.with_suffix(".yaml").read_text())
    image = Path(geometry["image"])
    if not image.is_absolute():
        image = base.parent / image
    image_hash = hashlib.sha256(image.read_bytes()).hexdigest()
    grid = catalog.grid(base.name)
    params = yaml.safe_load(args.params_file.read_text())
    config = params["global_costmap"]["global_costmap"]["ros__parameters"]
    footprint = json.loads(config["footprint"])
    checks = [footprint_cells(grid, pose, footprint) for pose in route["poses"]]
    if any(check["out_of_bounds"] or check["occupied_cells"] for check in checks[:-1]):
        raise ValueError("An intermediate point overlaps an obstacle; endpoint correction cannot repair this route")
    corrected, distance = nearest_free_endpoint(grid, route["poses"][-1], footprint, radius=args.max_endpoint_shift)
    candidate = copy.deepcopy(route)
    candidate["id"] += "_endpoint_safe"
    candidate["label"] = "扫描路线·终点避障版"
    candidate["poses"][-1] = corrected
    candidate["warning"] = f"第{len(route['poses'])}点相对扫描记录调整{distance:.2f}米，端点按整车轮廓及0.10米额外净空检查；请检查规划路径后执行。"
    candidate["endpoint_correction"] = {"waypoint": len(route["poses"]), "original": route["poses"][-1], "corrected": corrected, "shift_m": distance}
    candidate["static_preflight"] = {"map_pgm_sha256": image_hash, "footprint": footprint, "original_point_checks": checks,
                                     "corrected_endpoint_check": footprint_cells(grid, corrected, footprint, margin=0.1),
                                     "scope": "Static waypoint footprints only; final Smac path and real vehicle operation still require validation"}
    raw = copy.deepcopy(route)
    raw["label"] = "扫描原始路线·终点待确认"
    raw["warning"] = f"原始第{len(route['poses'])}点车体轮廓与障碍/未知区域重叠，原始坐标未修改；请调整终点后再执行。"
    result = {**original, "routes": [candidate, raw]}
    if hashlib.sha256(image.read_bytes()).hexdigest() != image_hash:
        raise ValueError("Map changed during preparation; retry on the saved map")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({"output": str(args.output), "point_count": len(candidate["poses"]), "endpoint_correction": candidate["endpoint_correction"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
