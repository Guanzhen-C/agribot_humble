#!/usr/bin/env python3
"""Regenerate canonical metric original-tractor bundles and raised implements."""
import argparse
import json
from pathlib import Path

from agribot_vehicle_config_tools.generator import generate_bundle
from agribot_farm_sim.mounts import prepare_mount
from prepare_scene_tractors import prepare, PROFILES

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--workspace-src", type=Path, required=True)
parser.add_argument("--unity-project", type=Path, required=True)
parser.add_argument("--asset-root", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
variants = {"car1_cultivator": ("unity_car1", "mounted_cultivator", "ros_v2"),
            "car2_cultivator": ("unity_car2", "mounted_cultivator", "ros_v2"),
            "car4_chisel": ("unity_car4", "compact_chisel_plow", "ros_v1"),
            "car4_harrow": ("unity_car4", "mounted_disc_harrow", "ros_v1"),
            "car5_chisel": ("unity_car5", "compact_chisel_plow", "ros_v1"),
            "car5_harrow": ("unity_car5", "mounted_disc_harrow", "ros_v1")}
sources = {}
for vehicle in PROFILES:
    dest = args.output / vehicle
    prepare(args.unity_project / "ConfigExports" / vehicle,
            args.asset_root / "cab_tractor/ros_v7", dest, vehicle)
    sources[vehicle] = dest
for vehicle, (base, implement, normalized) in variants.items():
    dest = args.output / vehicle
    prepare_mount(sources[base] / "vehicle_config.json", args.asset_root / implement / normalized, dest, vehicle)
    sources[vehicle] = dest
for vehicle, source in sources.items():
    config = json.loads((source / "vehicle_config.json").read_text())
    generate_bundle(config, source / "vehicle_config.json",
                    args.workspace_src / "agribot_vehicle_description/generated" / vehicle,
                    workspace_src=args.workspace_src)
    canonical = args.workspace_src / "agribot_vehicle_description/config/vehicles" / vehicle / "vehicle_config.json"
    canonical.parent.mkdir(parents=True, exist_ok=True)
    canonical.write_text(json.dumps(config, ensure_ascii=False, indent=2)+"\n")
    print("GENERATED " + vehicle, flush=True)
