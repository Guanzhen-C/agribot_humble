#!/usr/bin/env python3
"""Normalize an inspected implement LOD without resizing the original CAD.

blender -b --python prepare_implement.py -- ASSET_ROOT OUTPUT --profile PROFILE
ASSET_ROOT must contain convert_step.py's mesh and prepare_mesh.py's lod folders.
"""

import argparse
import json
from pathlib import Path
import sys

import bpy
from mathutils import Matrix, Vector


def bounds(objects):
    points = [obj.matrix_world @ vertex.co for obj in objects for vertex in obj.data.vertices]
    return (Vector([min(point[i] for point in points) for i in range(3)]),
            Vector([max(point[i] for point in points) for i in range(3)]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--lod", type=Path, help="Inspected LOD OBJ; default is INPUT/lod/visual.obj")
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    if args.output.exists():
        parser.error("Use a new output directory; source assets are never overwritten")
    profile = json.loads(args.profile.read_text())
    source = json.loads((args.input / "mesh/mesh_info.json").read_text())
    if source["sourceSha256"] != profile["sourceSha256"]:
        parser.error("CAD digest does not match the inspected implement profile")
    rotation = Matrix(profile["sourceToFluRotation"])
    identity = rotation @ rotation.transposed()
    if abs(rotation.determinant() - 1) > 1e-6 or any(
        abs(identity[i][j] - float(i == j)) > 1e-6 for i in range(3) for j in range(3)
    ):
        parser.error("Expected a proper rotation; scaling and mirroring are forbidden")
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    lod = args.lod if args.lod is not None else args.input / "lod/visual.obj"
    bpy.ops.import_scene.obj(filepath=str(lod.resolve()),
        axis_forward="Y", axis_up="Z", use_split_objects=True)
    objects = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    for obj in objects:
        world = obj.matrix_world.copy()
        for vertex in obj.data.vertices:
            vertex.co = rotation @ (world @ vertex.co)
        obj.matrix_world = Matrix.Identity(4)
    minimum, maximum = bounds(objects)
    size = maximum - minimum
    for dimension, (low, high) in zip(size, profile["expectedSizeRangeM"]):
        if not low <= dimension <= high:
            raise ValueError(f"Unexpected implement size {list(size)}")
    origin = Vector((maximum.x, (minimum.y + maximum.y) / 2, minimum.z))
    for obj in objects:
        for vertex in obj.data.vertices:
            vertex.co -= origin
    faces = sum(len(obj.data.polygons) for obj in objects)
    if faces > profile["maxVisualFaces"]:
        raise ValueError(f"LOD has {faces} faces, exceeding the inspected budget")
    args.output.mkdir(parents=True)
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    bpy.ops.export_scene.obj(filepath=str((args.output / "implement_visual.obj").resolve()),
        use_selection=True, axis_forward="Y", axis_up="Z", use_triangles=True, use_materials=True)
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from agribot_farm_sim.materials import normalize_ambient
    normalize_ambient(args.output / "implement_visual.mtl")
    report = dict(profile, coordinateConvention="ROS_FLU", units="metres", visualFaces=faces,
                  originDefinition="front_ground_center", sourceOriginM=list(rotation.transposed() @ origin),
                  minimumM=list(minimum - origin), maximumM=list(maximum - origin),
                  sizeM=list(size), mountingVerified=False, simulationVerified=False)
    (args.output / "implement_asset.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.color_type = "MATERIAL"
    scene.display.shading.show_cavity = True
    scene.display.shading.light = "STUDIO"
    scene.world.color = (.8, .8, .8)
    scene.render.resolution_x, scene.render.resolution_y = 1200, 900
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    camera_data = bpy.data.cameras.new("ImplementPreview")
    camera = bpy.data.objects.new("ImplementPreview", camera_data)
    scene.collection.objects.link(camera)
    camera_data.type = "ORTHO"
    camera_data.ortho_scale = size.length * 1.05
    center = (minimum + maximum) / 2 - origin
    camera.location = center + Vector((4, -5, 3)).normalized() * size.length * 2
    camera.rotation_euler = (center - camera.location).to_track_quat("-Z", "Y").to_euler()
    scene.camera = camera
    scene.render.filepath = str((args.output / "preview.png").resolve())
    bpy.ops.render.render(write_still=True)
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
