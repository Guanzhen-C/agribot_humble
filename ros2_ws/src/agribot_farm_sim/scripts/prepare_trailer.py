#!/usr/bin/env python3
"""Separate a locally supplied trailer's rigid body and four passive wheels."""

import argparse
import json
from pathlib import Path
import sys

import bpy
from mathutils import Matrix, Vector


def bounds(objects):
    points = [v.co for obj in objects for v in obj.data.vertices]
    return ([min(v[i] for v in points) for i in range(3)],
            [max(v[i] for v in points) for i in range(3)])


def export(objects, path):
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    bpy.ops.export_scene.obj(filepath=str(path.resolve()), use_selection=True,
        axis_forward="Y", axis_up="Z", use_materials=True, use_triangles=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--lod", type=Path)
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    if args.output.exists():
        parser.error("Use a new output directory")
    profile = json.loads(args.profile.read_text())
    info = json.loads((args.input/"mesh/mesh_info.json").read_text())
    if info["sourceSha256"] != profile["sourceSha256"]:
        parser.error("Trailer CAD digest does not match its inspected profile")
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    lod = args.lod or args.input/"lod/visual.obj"
    bpy.ops.import_scene.obj(filepath=str(lod.resolve()),
        axis_forward="Y", axis_up="Z", use_split_objects=True)
    objects = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    groups = {key: [] for key in profile["sourceWheelCenters"]}
    for obj in objects:
        low, high = bounds([obj])
        center = Vector([(a+b)/2 for a, b in zip(low, high)])
        for key, point in profile["sourceWheelCenters"].items():
            delta = center-Vector(point)
            if abs(delta.x) < .24 and abs(delta.y) < .18 and abs(delta.z) < .24 and max(high[i]-low[i] for i in range(3)) < 1.0:
                groups[key].append(obj)
                break
    if any(len(group) < 20 for group in groups.values()):
        raise ValueError("Trailer wheel regions do not match the inspected assembly")
    wheel_objects = [obj for group in groups.values() for obj in group]
    body = [obj for obj in objects if obj not in wheel_objects]
    rotation, origin = Matrix(profile["sourceToFluRotation"]), Vector(profile["sourceHitchM"])
    if abs(rotation.determinant()-1) > 1e-6:
        raise ValueError("Trailer transform must preserve metric scale")
    frame = bpy.data.materials.new("trailer_paint")
    frame.diffuse_color = (.18, .38, .23, 1)
    rubber = bpy.data.materials.new("trailer_rubber")
    rubber.diffuse_color = (.04, .045, .05, 1)
    for obj in objects:
        for vertex in obj.data.vertices:
            vertex.co = rotation @ (vertex.co-origin)
        obj.data.materials.clear()
        obj.data.materials.append(rubber if obj in wheel_objects else frame)
    faces = sum(len(obj.data.polygons) for obj in objects)
    if faces > profile["maxVisualFaces"]:
        raise ValueError("Trailer exceeds its visual budget")
    args.output.mkdir(parents=True)
    export(body, args.output/"implement_visual.obj")
    wheels = []
    for key, group in groups.items():
        center = rotation @ (Vector(profile["sourceWheelCenters"][key])-origin)
        for obj in group:
            for vertex in obj.data.vertices:
                vertex.co -= center
        filename = "trailer_"+key+".obj"
        export(group, args.output/filename)
        wheels.append(dict(id=key, position=list(center), radiusM=profile["wheelRadiusM"],
            widthM=profile["wheelWidthM"], massKg=20., meshFile=filename))
    low, high = bounds(body)
    report = dict(profile, coordinateConvention="ROS_FLU", units="metres", originDefinition="articulated_hitch",
        visualFaces=faces, wheels=wheels, minimumM=low, maximumM=high,
        groundBelowHitchM=profile["wheelRadiusM"]-wheels[0]["position"][2],
        collisionBoxes=[dict(name="trailer_body", center=[(a+b)/2 for a, b in zip(low, high)],
            size=[b-a+.002 for a, b in zip(low, high)])],
        wheelGroups={key: [obj.name for obj in group] for key, group in groups.items()},
        mountingVerified=False, simulationVerified=False)
    (args.output/"implement_asset.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n")
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from agribot_farm_sim.materials import normalize_ambient
    for path in args.output.glob("*.mtl"):
        normalize_ambient(path)
    print(json.dumps({k: v for k, v in report.items() if k != "wheelGroups"}, ensure_ascii=False))


if __name__ == "__main__":
    main()
