#!/usr/bin/env python3
"""Blender check of the nominal forward camera's central field against its body.

blender -b --python-exit-code 1 --python this_file -- --vehicle DIR
This is a geometric regression check, not a full image quality assessment.
"""
import argparse
import json
import math
from pathlib import Path
import sys

import bpy
from mathutils import Vector


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", type=Path, required=True)
    parser.add_argument("--check-lidar", action="store_true")
    parser.add_argument("--check-contacts", action="store_true")
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    config = json.loads((args.vehicle / "vehicle_config.json").read_text())
    camera = next(item for item in config["sensors"] if item["id"] == "right_camera")
    if any(abs(a) > 1e-6 for a in camera["pose"]["rpy"]):
        raise ValueError("This check requires the nominal FLU forward camera mount")
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    bpy.ops.import_scene.obj(filepath=str(args.vehicle / "vehicle_visual.obj"),
                             axis_forward="Y", axis_up="Z")
    bpy.context.view_layer.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    blocked = []
    sensors = [(camera, range(-15, 16, 5), range(-10, 11, 5))]
    if args.check_lidar:
        lidar = next(item for item in config["sensors"] if item["id"] == "lidar")
        if any(abs(a) > 1e-6 for a in lidar["pose"]["rpy"]):
            raise ValueError("This check requires the nominal FLU forward lidar mount")
        sensors.append((lidar, range(-60, 61, 10), range(-10, 11, 5)))
    checked = 0
    for sensor, yaws, pitches in sensors:
        for yaw in yaws:
            for pitch in pitches:
                checked += 1
                direction = Vector((1., math.tan(math.radians(yaw)),
                                    math.tan(math.radians(pitch)))).normalized()
                hit, point, _, _, obj, _ = bpy.context.scene.ray_cast(
                    depsgraph, Vector(sensor["pose"]["xyz"]), direction, distance=3.)
                if hit:
                    blocked.append(dict(sensor=sensor["id"], yaw=yaw, pitch=pitch, point=list(point), mesh=obj.name))
    contacts = []
    if args.check_contacts:
        feet = config.get("metadata", {}).get("sensorMountContacts", [])
        if not feet:
            raise ValueError("No inspected sensor mount contacts recorded")
        for obj in list(bpy.context.scene.objects):
            if obj.name.startswith("SensorMount_"):
                bpy.data.objects.remove(obj, do_unlink=True)
        bpy.context.view_layer.update()
        depsgraph = bpy.context.evaluated_depsgraph_get()
        for foot in feet:
            origin = Vector(foot["xyz"]) + Vector((0., 0., .02))
            hit, point, _, _, obj, _ = bpy.context.scene.ray_cast(
                depsgraph, origin, Vector((0., 0., -1.)), distance=.04)
            error = abs(point.z - foot["xyz"][2]) if hit else float("inf")
            if error > .001 or not obj.name.startswith(foot["sourcePart"]):
                raise RuntimeError("Sensor foot does not contact the inspected body part: " + foot["name"])
            contacts.append(dict(name=foot["name"], errorM=error))
    print(json.dumps(dict(vehicle=config["vehicleId"], checked=checked, blocked=blocked, contacts=contacts)), flush=True)
    if blocked:
        raise RuntimeError("Vehicle body obstructs the inspected forward sensor field")


if __name__ == "__main__":
    main()
