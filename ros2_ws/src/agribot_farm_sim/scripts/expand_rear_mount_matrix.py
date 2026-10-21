#!/usr/bin/env python3
"""Prepare additional inspected rear mounts and generate matching catalog proposals.

Existing canonical exports are not regenerated. Catalog proposals are published
separately after their geometry/previews have been checked.
"""
import argparse
import copy
import json
import shutil
from pathlib import Path

from agribot_farm_sim.mounts import prepare_mount, mounted_configuration
from agribot_vehicle_config_tools.generator import generate_bundle
from agribot_vehicle_config_tools.model import load_and_validate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace-src", required=True, type=Path)
    parser.add_argument("--unity-project", required=True, type=Path)
    parser.add_argument("--asset-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--replace-reviewed-existing", action="append", default=[])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=args.resume)
    if (args.output / "sources.json").exists():
        raise FileExistsError("Completed preparation is immutable; use its saved sources.json or a new output directory")
    package = args.workspace_src / "agribot_farm_sim"
    matrix = json.loads((package / "config/rear_mount_matrix.json").read_text())
    catalog = json.loads((package / "config/compatibility.json").read_text())
    presets = json.loads((args.unity_project / "Assets/AgribotConfig/Resources/farm_vehicle_presets.json").read_text())
    reviews = {(item["car"], item["implement"]) for item in matrix["designReviews"]}
    existing = {(item["car"], item["implement"]) for item in catalog["bindings"]}
    sources = {}
    for host in matrix["hosts"]:
        base = next(p for p in presets["presets"] if p.get("garageName") == host["car"])
        for tool in matrix["implements"]:
            pair = (host["car"], tool["id"])
            if pair in existing or pair in reviews:
                continue
            key = host["prefix"] + "_" + tool["suffix"]
            vehicle_path = args.asset_root / host["source"] / "vehicle_config.json"
            asset_path = args.asset_root / tool["source"]
            path = args.output / key / "vehicle_config.json"
            if path.exists() and args.resume:
                vehicle, _ = load_and_validate(vehicle_path)
                asset = json.loads((asset_path / "implement_asset.json").read_text())
                expected = json.loads(json.dumps(mounted_configuration(vehicle, asset, key)))
                if json.loads(path.read_text()) != expected:
                    raise ValueError("Staged inputs changed; use a new output directory: " + key)
            else:
                path = prepare_mount(vehicle_path, asset_path, args.output / key, key)
            config = json.loads(path.read_text())
            bundle = args.workspace_src / "agribot_vehicle_description/generated" / key
            canonical = args.workspace_src / "agribot_vehicle_description/config/vehicles" / key / "vehicle_config.json"
            same_bundle = ((bundle / "vehicle_config.json").is_file()
                           and json.loads((bundle / "vehicle_config.json").read_text()) == config)
            same_canonical = canonical.is_file() and json.loads(canonical.read_text()) == config
            if (bundle.exists() or canonical.exists()) and not (args.resume and same_bundle and same_canonical):
                if key not in args.replace_reviewed_existing:
                    raise FileExistsError("Unregistered existing export needs review: " + key)
                backup = args.output / "before" / key
                backup.mkdir(parents=True, exist_ok=False)
                if bundle.exists():
                    shutil.copytree(bundle, backup / "generated")
                if canonical.exists():
                    shutil.copy2(canonical, backup / "vehicle_config.json")
            if not (args.resume and same_bundle and same_canonical and (bundle / "manifest.json").is_file()):
                generate_bundle(config, path, bundle, workspace_src=args.workspace_src)
            canonical.parent.mkdir(parents=True, exist_ok=True)
            canonical.write_text(path.read_text())
            catalog["bindings"].append(dict(car=host["car"], preset=key, implement=tool["id"],
                unityType=tool["unityType"], name=tool["name"], capability="fixed_transport"))
            preset = copy.deepcopy(base)
            preset.pop("garageName", None)
            preset.update(id=key, baseVehicleId=host["baseVehicleId"], implementObjectName=tool["unityType"],
                implementId=tool["id"], implementName=tool["name"], tasks=["机具运输"],
                verification="mount_geometry_balance_only_route_pending")
            presets["presets"].append(preset)
            sources[key] = str(path.parent)
            print("GENERATED", key, config["metadata"]["implementWheelClearance"], flush=True)
    for name, data in (("compatibility.expanded.json", catalog), ("farm_vehicle_presets.expanded.json", presets),
                       ("sources.json", sources)):
        (args.output / name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
    print("ADDITIONAL_MOUNTS", len(sources), flush=True)


if __name__ == "__main__":
    main()
