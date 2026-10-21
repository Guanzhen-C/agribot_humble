#!/usr/bin/env python3
"""Prepare a flat full-size machinery headland from the existing licensed maize.

blender -b --python prepare_wide_headland.py -- SOURCE_ASSET NEW_OUTPUT
"""
import json
import math
from pathlib import Path
import shutil
import sys
import bpy
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from import_open_scene import import_mesh, field_surface, compact_textures, export, occupancy, world_file

source, output = map(Path, sys.argv[sys.argv.index("--")+1:])
if output.exists():
    raise FileExistsError(output)
output.mkdir(parents=True)
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
original = import_mesh(source / "scene.obj")
plants = sorted((obj for obj in original if obj.name.startswith("maize_")), key=lambda obj: obj.name)[:8]
if len(plants) != 8:
    raise ValueError("Expected the reviewed maize geometry")
soil = next(mat for mat in bpy.data.materials if "soil" in mat.name.lower() or "mud" in mat.name.lower())
objects = [field_surface("Flat_transport_headland", (0, 12, -.002), (86, 68), soil)]
objects[0]["navigation_surface"] = True
positions = [(x, y) for y in (-14, -12, -10) for x in range(-28, 25, 2)]
positions += [(x, y) for x in (34, 36, 38) for y in range(4, 35, 2)]
for index, (x, y) in enumerate(positions):
    original_plant = plants[index % len(plants)]
    points = [original_plant.matrix_world @ Vector(p) for p in original_plant.bound_box]
    low = Vector([min(p[i] for p in points) for i in range(3)])
    high = Vector([max(p[i] for p in points) for i in range(3)])
    center = Vector(((low.x+high.x)/2, (low.y+high.y)/2, low.z))
    plant = original_plant.copy()
    plant.data = original_plant.data.copy()
    bpy.context.scene.collection.objects.link(plant)
    plant.matrix_world = original_plant.matrix_world.copy()
    plant.location += Vector((x, y, .015))-center
    objects.append(plant)
metal = bpy.data.materials.new("headland_boundary_posts")
metal.diffuse_color = (.34, .38, .30, 1.)
for index, (x, y) in enumerate([(x, -19) for x in range(-40, 41, 5)] +
                              [(x, 43) for x in range(-40, 41, 5)] +
                              [(-40, y) for y in range(-14, 43, 5)] +
                              [(40, y) for y in range(-14, 43, 5)]):
    bpy.ops.mesh.primitive_cube_add(size=1, location=(x, y, 1.))
    post = bpy.context.object
    post.name = "Boundary_post_"+str(index)
    post.dimensions = (.16, .16, 2.)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    post.data.materials.append(metal)
    objects.append(post)
for obj in original:
    bpy.data.objects.remove(obj, do_unlink=True)
compact_textures(output, 1024)
export(objects, output / "scene.obj")
export(objects, output / "collision.obj")
map_info = occupancy(objects, output, "maize")
world_file(output, "maize")
metadata = json.loads((source / "source.json").read_text())
metadata.update(map=map_info, visualFaces=sum(len(o.data.polygons) for o in objects),
                collisionFaces=sum(len(o.data.polygons) for o in objects),
                surfaceHeights=[[0, 0, -.002]], objects=[])
metadata["modifications"] += ["Metric wide headland with flat ground; original plant geometry retained, rearranged planting strips", "Full-size machinery route, no vehicle or implement shrinking"]
for obj in objects:
    points = [obj.matrix_world @ Vector(p) for p in obj.bound_box]
    metadata["objects"].append(dict(name=obj.name, navigationSurface=bool(obj.get("navigation_surface", False)),
        minimum=[min(p[i] for p in points) for i in range(3)],
        maximum=[max(p[i] for p in points) for i in range(3)]))
(output / "source.json").write_text(json.dumps(metadata, indent=2)+"\n")
for path in source.glob("LICENSE*"):
    shutil.copy2(path, output / path.name)
print("WIDE_HEADLAND_READY", len(objects), map_info)
