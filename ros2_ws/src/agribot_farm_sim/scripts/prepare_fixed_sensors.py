#!/usr/bin/env python3
"""Reuse the garage sensor CAD at fixed surface-supported tractor mounts.

Run with Blender: --python this_file -- --vehicle DIR --reference DIR --output DIR
The input vehicle and original garage exports are never overwritten.
"""
import argparse
import json
import math
from pathlib import Path
import shutil
import sys

import bpy
from mathutils import Vector


def load_mesh(path):
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    bpy.ops.import_scene.obj(filepath=str(path), axis_forward="Y", axis_up="Z")
    bpy.context.view_layer.update()
    return [o.matrix_world @ v.co for o in bpy.context.scene.objects if o.type == "MESH" for v in o.data.vertices]


def main():
    parser = argparse.ArgumentParser()
    for name in ("vehicle", "reference", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--mount-profile", type=Path,
                        help="Optional fixed XY surface probes for imu/lidar/rtk_master")
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    if args.output.exists():
        raise ValueError("Output exists; use a new revision directory")
    config = json.loads((args.vehicle / "vehicle_config.json").read_text())
    reference_path = args.reference / "configuration.json"
    if not reference_path.exists():
        reference_path = args.reference / "vehicle_config.json"
    reference = json.loads(reference_path.read_text())
    bounds = {}
    for sensor_id in ("imu", "lidar", "right_camera", "rtk_master"):
        points = load_mesh(args.reference / f"sensor_{sensor_id}_visual.obj")
        bounds[sensor_id] = [[min(p[i] for p in points) for i in range(3)],
                             [max(p[i] for p in points) for i in range(3)]]
    load_mesh(args.vehicle / "vehicle_visual.obj")
    depsgraph = bpy.context.evaluated_depsgraph_get()

    def mount(x, y, sensor_id):
        hit, point, normal, *_ = bpy.context.scene.ray_cast(depsgraph, Vector((x, y, 10)), Vector((0, 0, -1)))
        if not hit or normal.z < .8:
            raise ValueError(f"No suitable mounting surface at {x}, {y}")
        return [x, y, point.z - bounds[sensor_id][0][2] + .001]

    probes = {"imu": [.95, -.10], "lidar": [1.60, 0.], "rtk_master": [.8, 0.]}
    standoffs = {}
    if args.mount_profile:
        profile = json.loads(args.mount_profile.read_text())
        probes = profile["surfaceProbes"]
        standoffs = profile.get("standoffHeightM", {})
        if set(probes) != {"imu", "lidar", "rtk_master"} or any(len(p) != 2 for p in probes.values()):
            raise ValueError("Expected three fixed XY surface probes")
        if any(key not in ("imu", "lidar", "rtk_master") or isinstance(value, bool) or
               not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0
               for key, value in standoffs.items()):
            raise ValueError("Standoffs require positive finite heights at supported sensor mounts")
    positions = {key: mount(*probes[key], key) for key in ("imu", "lidar")}
    positions["rtk_master"] = mount(*probes["rtk_master"], "rtk_master")
    for key, height in standoffs.items():
        x, y, z = positions[key]
        bottom = z + bounds[key][0][2]
        bpy.ops.mesh.primitive_cylinder_add(vertices=16, radius=.0125, depth=height,
            location=(x, y, bottom+height/2))
        bpy.context.object.name = key+"_support_post"
        mat = bpy.data.materials.new(key+"_support_steel")
        mat.diffuse_color = (.22, .25, .27, 1.)
        bpy.context.object.data.materials.append(mat)
        positions[key][2] += height
    lidar = positions["lidar"]
    positions["right_camera"] = [lidar[0] + .0253, lidar[1],
        lidar[2] + bounds["lidar"][1][2] - bounds["right_camera"][0][2] + .001]
    # The master visual contains both antennas and their complete mounting mast.
    old = {s["id"]: s["pose"]["xyz"] for s in reference["sensors"]}
    baseline = old["rtk_master"][1] - old["rtk_secondary"][1]
    positions["rtk_master"][1] = baseline / 2
    positions["rtk_secondary"] = [positions["rtk_master"][0], -baseline / 2, positions["rtk_master"][2]]
    for sensor in config["sensors"]:
        sensor["pose"]["xyz"] = positions[sensor["id"]]
    config["metadata"]["notes"] += " Fixed sensor CAD reused from the existing garage; mounting bases supported by the hood."
    shutil.copytree(args.vehicle, args.output)
    if standoffs:
        bpy.ops.object.select_all(action="SELECT")
        bpy.ops.export_scene.obj(filepath=str((args.output/"vehicle_visual.obj").resolve()),
            use_selection=True, axis_forward="Y", axis_up="Z", use_triangles=True, use_materials=True)
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from agribot_farm_sim.materials import normalize_ambient
        normalize_ambient(args.output/"vehicle_visual.mtl")
    for path in args.reference.glob("sensor_*_visual.*"):
        shutil.copy2(path, args.output / path.name)
    (args.output / "vehicle_config.json").write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n")
    (args.output / "sensor_mount_report.json").write_text(json.dumps({"positions": positions,
        "visualBounds": bounds, "standoffHeightM": standoffs}, indent=2) + "\n")
    print("FIXED_SENSOR_MOUNTS=" + json.dumps(positions), flush=True)


if __name__ == "__main__":
    main()
