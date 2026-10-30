#!/usr/bin/env python3
"""Prepare a separate map-bound route with guides on the original scanner trace."""

import argparse
import csv
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np

from agribot_mobile_app.scanner_route import densify_reference_route


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manual", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--tolerance", type=float, default=0.25)
    parser.add_argument("--maximum-spacing", type=float, default=12.0)
    parser.add_argument("--maximum-distance", type=float, default=0.75)
    args = parser.parse_args()
    if args.output.exists() or args.output.resolve() == args.manual.resolve():
        raise ValueError("Refusing to replace original manual picks or an existing output")
    source_bytes = args.manual.read_bytes()
    document = json.loads(source_bytes)
    reference_path = Path(document["reference_csv"])
    reference_bytes = reference_path.read_bytes()
    if hashlib.sha256(reference_bytes).hexdigest() != document["reference_csv_sha256"]:
        raise ValueError("Original scanner reference has changed")
    with reference_path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    reference = np.array([[float(row[key]) for key in ("map_x_m", "map_y_m")] for row in rows])
    route = document["routes"][0]
    expanded = densify_reference_route(
        reference, route["poses"], tolerance=args.tolerance,
        maximum_spacing=args.maximum_spacing, maximum_distance=args.maximum_distance)
    output = deepcopy(document)
    output["ready_for_navigation"] = False
    candidate = output["routes"][0]
    candidate.update(expanded)
    candidate["id"] = route["id"] + "_guided"
    candidate["label"] = "人工选点加扫描轨迹引导点"
    candidate["manual_anchor_source_indices_before_rematch"] = route.get("source_sample_indices", [])
    candidate["manual_input_sha256"] = hashlib.sha256(source_bytes).hexdigest()
    candidate["densification"] = {"tolerance_m": args.tolerance,
                                "maximum_spacing_m": args.maximum_spacing,
                                "maximum_match_distance_m": args.maximum_distance,
                                "manual_poses_and_headings_unchanged": True}
    candidate["warning"] = "保留人工位姿并沿原始扫描轨迹补点；尚未完成真车通行验收。"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(output, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"output": str(args.output), "original_points": len(route["poses"]),
                      "total_points": len(expanded["poses"]),
                      "added_points": len(expanded["poses"]) - len(route["poses"]),
                      "maximum_anchor_rematch_distance_m": max(expanded["manual_anchor_reference_offsets_m"])},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
