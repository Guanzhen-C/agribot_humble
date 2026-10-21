#!/usr/bin/env python3
"""Build a nominal front sickle-bar assembly, not a manufacturer CAD model.

blender -b --python prepare_front_cutter.py -- OUTPUT
The fixed transport mesh has no animated blade or grass-cutting physics.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import bpy
from mathutils import Vector


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    if args.output.exists():
        parser.error("Output must be a new directory")
    args.output.mkdir(parents=True)
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    materials = {}
    for name, rgb in {"red_guard": (.58, .035, .025), "steel": (.42, .46, .49),
                      "dark_frame": (.065, .075, .085), "cutting_edge": (.67, .69, .70)}.items():
        material = bpy.data.materials.new(name)
        material.diffuse_color = (*rgb, 1.)
        materials[name] = material

    def box(name, center, size, material, bevel=.004):
        bpy.ops.mesh.primitive_cube_add(size=1, location=center)
        obj = bpy.context.object
        obj.name = name
        obj.scale = size
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        obj.data.materials.append(materials[material])
        if bevel:
            mod = obj.modifiers.new("RoundedEdges", "BEVEL")
            mod.width, mod.segments = bevel, 2
            bpy.ops.object.modifier_apply(modifier=mod.name)
        return obj

    def tooth(name, y, x0, x1, half_width, z, thickness, material):
        vertices = [(x, v, h) for h in (z, z + thickness)
                    for x, v in ((x0, y - half_width), (x0, y + half_width), (x1, y))]
        mesh = bpy.data.meshes.new(name)
        mesh.from_pydata(vertices, [], [(0, 1, 2), (5, 4, 3), (0, 3, 4, 1), (1, 4, 5, 2), (2, 5, 3, 0)])
        mesh.update()
        obj = bpy.data.objects.new(name, mesh)
        bpy.context.collection.objects.link(obj)
        obj.data.materials.append(materials[material])
        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_all(action="SELECT")
        bpy.ops.mesh.normals_make_consistent(inside=False)
        bpy.ops.object.mode_set(mode="OBJECT")

    box("cutter_backbone", (.085, 0, .075), (.14, 1.60, .09), "dark_frame")
    box("red_safety_hood", (.06, 0, .145), (.16, 1.56, .045), "red_guard")
    box("sickle_slide", (.165, 0, .061), (.08, 1.49, .012), "steel", .001)
    for index in range(23):
        y = (index - 11) * .064
        tooth("finger_guard_%02d" % index, y, .13, .35, .026, .035, .014, "dark_frame")
        tooth("sickle_blade_%02d" % index, y + .015, .15, .305, .029, .051, .006, "cutting_edge")
        bpy.ops.mesh.primitive_cylinder_add(vertices=12, radius=.008, depth=.012, location=(.12, y, .101))
        bpy.context.object.name = "guard_fastener_%02d" % index
        bpy.context.object.data.materials.append(materials["steel"])
    for side in (-1, 1):
        box("end_skid", (.13, side * .79, .018), (.31, .05, .036), "steel")
        box("mount_arm", (-.22, side * .13, .185), (.68, .06, .06), "dark_frame")
        box("chassis_mount_plate", (-.525, side * .13, .255), (.07, .10, .09), "steel")
    box("mount_crossmember", (-.36, 0, .185), (.07, .32, .06), "dark_frame")
    box("drive_housing", (.025, 0, .205), (.15, .22, .10), "red_guard", .015)
    box("drive_cover", (.105, 0, .204), (.018, .18, .07), "dark_frame")

    objects = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    bpy.context.view_layer.update()
    points = [obj.matrix_world @ Vector(corner) for obj in objects for corner in obj.bound_box]
    low = [min(p[i] for p in points) for i in range(3)]
    high = [max(p[i] for p in points) for i in range(3)]
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.export_scene.obj(filepath=str((args.output / "implement_visual.obj").resolve()),
                            use_selection=True, axis_forward="Y", axis_up="Z", use_materials=True,
                            use_triangles=True, keep_vertex_order=True)
    boxes = [
        {"name": "cutter_head", "center": [.15, 0, .13], "size": [.404, 1.634, .264]},
        {"name": "mount_crossmember", "center": [-.36, 0, .185], "size": [.074, .324, .064]},
    ]
    for side, name in ((-1, "right"), (1, "left")):
        boxes.append({"name": name + "_mount_arm", "center": [-.22, side * .13, .185],
                      "size": [.684, .064, .064]})
        boxes.append({"name": name + "_mount_plate", "center": [-.525, side * .13, .255],
                      "size": [.074, .104, .094]})
    report = {"id": "front_sickle_bar", "displayName": "前置割草刀杆", "units": "metres",
              "coordinateConvention": "ROS_FLU", "originDefinition": "mount_ground_center",
              "minimumM": low, "maximumM": high, "collisionBoxes": boxes,
              "sourceSha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "redistributionLicense": "project_generated_nominal_geometry",
              "source": "Parametric simulation assembly; not manufacturer CAD or verified hardware.",
              "limitations": "Fixed transport only; no blade actuation, PTO, cutting, or lifting-capacity model."}
    (args.output / "implement_asset.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False))


if __name__ == "__main__":
    main()
