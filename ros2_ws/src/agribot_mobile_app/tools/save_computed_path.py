#!/usr/bin/env python3
"""Freeze an already computed path and bind manual start markers monotonically."""
import argparse
import csv
import json
from pathlib import Path

from agribot_mobile_app.saved_path import sha256
from agribot_mobile_app.scanner_route import ordered_reference_matches


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manual", type=Path)
    parser.add_argument("path_csv", type=Path)
    parser.add_argument("map_base", type=Path)
    parser.add_argument("nav2_config", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    manual = json.loads(args.manual.read_text())
    with args.path_csv.open() as stream:
        path = [{k: float(row[k]) for k in ("x", "y", "yaw")} for row in csv.DictReader(stream)]
    anchors = manual["routes"][0]["poses"]
    indices = ordered_reference_matches([[p["x"], p["y"]] for p in path],
                                        [[p["x"], p["y"]] for p in anchors], 0.1)
    base = args.map_base
    document = {"schema_version": 1, "map_id": manual["map_id"],
                "fingerprints": {"pgm": sha256(base.with_suffix(".pgm")),
                                 "yaml": sha256(base.with_suffix(".yaml")),
                                 "georeference": sha256(Path(f"{base}_georeference.yaml")),
                                 "nav2": sha256(args.nav2_config),
                                 "primitives": sha256(args.nav2_config.parent / "motion_primitives/diff_5cm.json")},
                "routes": [{"id": "manual104_saved", "label": "人工104点已验证规划路径",
                            "vehicle_type": "differential", "path": path,
                            "anchor_path_indices": indices,
                            "manual_poses": anchors, "source_manual_sha256": sha256(args.manual),
                            "source_path_sha256": sha256(args.path_csv)}]}
    source_route = manual["routes"][0]
    if "localization_handover_waypoint" in source_route:
        saved = document["routes"][0]
        saved["id"] = source_route["id"] + "_saved"
        saved["label"] = source_route["label"] + "规划路径"
        saved["localization_handover_waypoint"] = source_route["localization_handover_waypoint"]
    with args.output.open("x") as stream:
        json.dump(document, stream, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
    print(f"Saved {len(path)} path poses, {len(indices)} start markers: {args.output}")


if __name__ == "__main__":
    main()
