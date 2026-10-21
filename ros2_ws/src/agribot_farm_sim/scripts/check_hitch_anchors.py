#!/usr/bin/env python3
"""Blender audit: pin centres must be within an eye radius of inspected CAD parts."""
import argparse
import json
from pathlib import Path
import sys

import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree


def distances(path, points):
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for mesh in list(bpy.data.meshes):
        if mesh.users == 0:
            bpy.data.meshes.remove(mesh)
    bpy.ops.import_scene.obj(filepath=str(path), axis_forward="Y", axis_up="Z")
    result = []
    for point, name in points:
        obj = bpy.data.objects.get(name)
        if obj is None:
            raise ValueError("missing inspected CAD part: " + name)
        tree = BVHTree.FromPolygons([obj.matrix_world @ v.co for v in obj.data.vertices],
                                   [list(poly.vertices) for poly in obj.data.polygons])
        location, normal, face, distance = tree.find_nearest(Vector(point))
        result.append(dict(part=name, point=point, distanceM=distance,
                           nearest=list(location) if location else None))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", type=Path, required=True)
    parser.add_argument("--tool", type=Path, required=True)
    parser.add_argument("--kind", required=True)
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "agribot_vehicle_config_tools"))
    from agribot_farm_sim.hitches import HOSTS, TOOLS, RECEIVER_FRAMES
    config = json.loads((args.vehicle / "vehicle_config.json").read_text())
    results = distances(args.vehicle / "vehicle_visual.obj", HOSTS[config["vehicleId"]][1])
    receiver = RECEIVER_FRAMES.get(args.kind)
    tool_points = receiver["footings"] if receiver else TOOLS[args.kind]
    results += distances(args.tool / "implement_visual.obj", tool_points)
    print(json.dumps(dict(vehicle=config["vehicleId"], tool=args.kind, anchors=results,
                         toolAnchorType="frame_receiver_footings" if receiver else "existing_structure"), indent=2))
    if any(item["distanceM"] is None or item["distanceM"] > .042 for item in results):
        raise ValueError("hitch pin/eye would be separated from the inspected CAD geometry")


if __name__ == "__main__":
    main()
