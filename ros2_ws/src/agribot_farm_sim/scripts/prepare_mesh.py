#!/usr/bin/env python3
"""Create a separate visual LOD and orthographic previews with Blender.

blender -b --python prepare_mesh.py -- INPUT_OBJ OUTPUT_DIR --faces 100000
The original CAD mesh is not changed. Object groups remain independently usable.
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
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--faces", type=int, default=100000)
    parser.add_argument("--large-part-size-m", type=float, default=0.0,
                        help="Optional minimum bounding-box diagonal for structural-part protection")
    parser.add_argument("--large-part-min-faces", type=int, default=500)
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    if args.faces < 1000:
        parser.error("Use a target of at least 1000 faces")
    if not math.isfinite(args.large_part_size_m) or args.large_part_size_m < 0 or args.large_part_min_faces < 12:
        parser.error("Structural-part protection needs a finite nonnegative size and at least 12 faces")
    if args.input.resolve().parent == args.output.resolve():
        parser.error("Output must be separate from the source mesh directory")
    args.output.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    bpy.ops.import_scene.obj(filepath=str(args.input.resolve()), axis_forward="Y", axis_up="Z",
                             use_split_objects=True, use_split_groups=False)
    objects = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    original = sum(len(obj.data.polygons) for obj in objects)
    ratio = min(1.0, args.faces / max(original, 1))
    protected_parts = []
    for obj in objects:
        bpy.context.view_layer.objects.active = obj
        if ratio < 1.0 and len(obj.data.polygons) > 32:
            corners = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
            diagonal = Vector([max(point[i] for point in corners) - min(point[i] for point in corners)
                               for i in range(3)]).length
            minimum_faces = 12
            if args.large_part_size_m > 0 and diagonal >= args.large_part_size_m:
                minimum_faces = args.large_part_min_faces
                protected_parts.append(obj.name)
            modifier = obj.modifiers.new("VisualLOD", "DECIMATE")
            modifier.ratio = min(1.0, max(ratio, minimum_faces / len(obj.data.polygons)))
            bpy.ops.object.modifier_apply(modifier=modifier.name)
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    bpy.ops.export_scene.obj(filepath=str((args.output / "visual.obj").resolve()),
                             use_selection=True, axis_forward="Y", axis_up="Z",
                             use_materials=True, use_triangles=True, keep_vertex_order=True)
    bounds = [obj.matrix_world @ Vector(corner) for obj in objects for corner in obj.bound_box]
    minimum = Vector([min(point[axis] for point in bounds) for axis in range(3)])
    maximum = Vector([max(point[axis] for point in bounds) for axis in range(3)])
    center = (maximum + minimum) / 2
    diagonal = (maximum - minimum).length
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.light = "STUDIO"
    scene.display.shading.color_type = "MATERIAL"
    scene.display.shading.show_shadows = True
    scene.display.shading.show_cavity = True
    scene.display.shading.cavity_type = "BOTH"
    scene.display.shading.background_type = "WORLD"
    scene.world.color = (0.8, 0.8, 0.8)
    scene.render.resolution_x = 1000
    scene.render.resolution_y = 800
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    camera_data = bpy.data.cameras.new("InspectionCamera")
    camera = bpy.data.objects.new("InspectionCamera", camera_data)
    scene.collection.objects.link(camera)
    camera_data.type = "ORTHO"
    camera_data.ortho_scale = diagonal * 1.12
    camera_data.clip_end = max(100, diagonal * 10)
    scene.camera = camera
    for name, direction in (("isometric", (1, -1, 0.8)), ("xy", (0, 0, 1)),
                            ("xz", (0, -1, 0)), ("yz", (1, 0, 0))):
        camera.location = center + Vector(direction).normalized() * diagonal * 2
        camera.rotation_euler = (center - camera.location).to_track_quat("-Z", "Y").to_euler()
        scene.render.filepath = str((args.output / f"preview_{name}.png").resolve())
        bpy.ops.render.render(write_still=True)
    report = dict(source=str(args.input.resolve()), sourceFaces=original,
                  lodFaces=sum(len(obj.data.polygons) for obj in objects), objectCount=len(objects),
                  minimumM=list(minimum), maximumM=list(maximum),
                  largePartSizeM=args.large_part_size_m, largePartMinFaces=args.large_part_min_faces,
                  protectedParts=protected_parts,
                  axes="source_CAD_unverified", mountingVerified=False)
    (args.output / "lod_info.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
