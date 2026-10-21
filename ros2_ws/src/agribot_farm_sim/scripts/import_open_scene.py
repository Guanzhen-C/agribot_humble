#!/usr/bin/env python3
"""Build a portable Gazebo scene from pinned upstream agricultural assets.

Run using Blender 3.x: blender -b --python import_open_scene.py -- ...
No upstream ROS launch file or simulator plugin is executed. The tomato asset
generator is separately pinned and hash-checked before its geometry-only use.
"""

import argparse
import hashlib
import importlib.util
import json
import math
import random
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import bpy
from mathutils import Euler, Matrix, Vector
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from agribot_farm_sim.legacy_sdf import read_harvester_geometry, HARVESTER_SDF_SHA256


def transform(text):
    values = list(map(float, (text or "0 0 0 0 0 0").split()))
    return Matrix.Translation(Vector(values[:3])) @ Euler(values[3:], "XYZ").to_matrix().to_4x4()


def import_mesh(path):
    before = set(bpy.context.scene.objects)
    if path.suffix == ".dae":
        document = ET.parse(path)
        namespace = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
        missing_uv = document.findall(".//c:texture", namespace)
        normalized_material = False
        for effect in document.findall(".//c:effect", namespace):
            sampler_ids = {node.get("sid") for node in effect.findall(".//c:newparam", namespace)}
            image_ids = {node.get("id") for node in document.findall(".//c:image", namespace)}
            for parent in effect.iter():
                for node in list(parent):
                    if node.tag.endswith("}texture") and node.get("texture") not in sampler_ids | image_ids:
                        parent.remove(node)
                        ET.SubElement(parent, "{" + namespace["c"] + "}color").text = ".3 .28 .24 1"
                        normalized_material = True
        if normalized_material or any("texcoord" not in node.attrib for node in missing_uv):
            # Some Gazebo assets omit a required COLLADA attribute accepted by
            # Assimp. Normalize a temporary copy for Blender's strict importer.
            for node in missing_uv:
                node.attrib.setdefault("texcoord", "UVMap")
            for node in document.findall(".//c:library_images/c:image/c:init_from", namespace):
                node.text = str((path.parent / node.text).resolve())
            ET.register_namespace("", namespace["c"])
            with tempfile.TemporaryDirectory() as directory:
                normalized = Path(directory) / path.name
                document.write(normalized, encoding="utf-8", xml_declaration=True)
                bpy.ops.wm.collada_import(filepath=str(normalized))
        else:
            bpy.ops.wm.collada_import(filepath=str(path))
    elif path.suffix.lower() == ".stl":
        bpy.ops.import_mesh.stl(filepath=str(path))
    else:
        if path.read_bytes()[:80].startswith(b"version https://git-lfs.github.com"):
            raise ValueError(f"LFS mesh is not downloaded: {path}")
        bpy.ops.import_scene.obj(filepath=str(path), axis_forward="Y", axis_up="Z")
    meshes = [obj for obj in bpy.context.scene.objects if obj not in before and obj.type == "MESH"]
    if not meshes:
        raise ValueError(f"Mesh importer returned no geometry: {path}")
    return meshes


def load_harbot(repository):
    models = repository / "gazebo_classic/models"
    world = ET.parse(repository / "gazebo_classic/greenhouse.world").getroot().find("world")
    cache = {}
    objects = []
    for include in world.findall("include"):
        name = include.findtext("uri").removeprefix("model://")
        if name == "sun":
            continue
        # External decorative trees contain millions of disconnected leaf faces.
        # The navigation scene retains the complete greenhouse and crop meshes.
        if name in ("small_tree", "large_tree"):
            continue
        model = ET.parse(models / name / "model.sdf").getroot().find("model")
        model_tf = transform(include.findtext("pose")) @ transform(model.findtext("pose"))
        for link in model.findall("link"):
            for visual in link.findall("visual"):
                mesh = visual.find("geometry/mesh")
                if mesh is None:
                    raise ValueError("Unexpected non-mesh upstream visual")
                path = models / mesh.findtext("uri").removeprefix("model://")
                if path not in cache:
                    cache[path] = import_mesh(path)
                    for obj in cache[path]:
                        # Reduce repeated trees once, retaining material boundaries and UVs.
                        count = len(obj.data.polygons)
                        if count > 45000:
                            bpy.context.view_layer.objects.active = obj
                            modifier = obj.modifiers.new("VisualLOD", "DECIMATE")
                            modifier.ratio = 45000 / count
                            bpy.ops.object.modifier_apply(modifier=modifier.name)
                scale = Matrix.Diagonal(Vector([*map(float, mesh.findtext("scale", "1 1 1").split()), 1]))
                for original in cache[path]:
                    obj = original.copy()
                    obj.name = include.findtext("name", name) + "_" + link.attrib["name"]
                    obj["navigation_surface"] = name == "greenhouse" and link.attrib["name"] in (
                        "base_link", "greenhouse_link", "grass_link", "pathway_link")
                    bpy.context.scene.collection.objects.link(obj)
                    obj.matrix_world = model_tf @ transform(link.findtext("pose")) @ transform(visual.findtext("pose")) @ scale @ original.matrix_world
                    objects.append(obj)
    for templates in cache.values():
        for obj in templates:
            bpy.data.objects.remove(obj, do_unlink=True)
    # Retain the upstream shell but make it visible as milky greenhouse film.
    for material in {slot.material for obj in objects for slot in obj.material_slots
                     if slot.material and slot.material.name.split(".")[0] == "glass"}:
        material.diffuse_color = (.88, .92, .86, .28)
        material.roughness = .65
        material.specular_intensity = .25
        if material.use_nodes:
            shader = next((node for node in material.node_tree.nodes
                           if node.type == "BSDF_PRINCIPLED"), None)
            if shader:
                shader.inputs["Base Color"].default_value = (.88, .92, .86, 1)
                shader.inputs["Alpha"].default_value = .28
                shader.inputs["Roughness"].default_value = .65
                shader.inputs["Specular"].default_value = .25
                shader.inputs["Transmission"].default_value = 0
    return objects, repository / "LICENSE", "MIT"


def texture_material(name, path):
    material = bpy.data.materials.new(name)
    material.use_nodes = True
    shader = material.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Roughness"].default_value = .9
    image = material.node_tree.nodes.new("ShaderNodeTexImage")
    image.image = bpy.data.images.load(str(path))
    material.node_tree.links.new(image.outputs["Color"], shader.inputs["Base Color"])
    return material


def field_surface(name, center, size, material):
    bpy.ops.mesh.primitive_plane_add(size=2, location=center)
    obj = bpy.context.object
    obj.name = name
    obj.scale = (size[0] / 2, size[1] / 2, 1)
    obj.data.materials.append(material)
    for loop in obj.data.uv_layers.active.data:
        loop.uv.x *= size[0] / 2
        loop.uv.y *= size[1] / 2
    obj["navigation_surface"] = True
    return obj


def load_maize(repository, soil_texture):
    if soil_texture is None or not soil_texture.is_file():
        raise ValueError("Maize scene requires the Poly Haven brown_mud diffuse texture")
    templates = import_mesh(repository / "models/maize_02/meshes/maize_02.dae")
    bpy.context.view_layer.update()
    bounds = [obj.matrix_world @ Vector(corner) for obj in templates for corner in obj.bound_box]
    center = Vector(((min(v.x for v in bounds) + max(v.x for v in bounds)) / 2,
                     (min(v.y for v in bounds) + max(v.y for v in bounds)) / 2,
                     min(v.z for v in bounds)))
    scale = 1.2 / (max(v.z for v in bounds) - center.z)
    randomizer = random.Random(42)
    objects = []
    for row in range(5):
        for side in (-.4, .4):
            for index in range(45):
                x = 9 + index * .75 + randomizer.uniform(-.07, .07)
                y = 7 + row * 6 + side + randomizer.uniform(-.05, .05)
                matrix = (Matrix.Translation(Vector((x, y, .015)))
                          @ Matrix.Rotation(randomizer.uniform(0, 2 * math.pi), 4, "Z")
                          @ Matrix.Scale(scale * randomizer.uniform(.85, 1.1), 4)
                          @ Matrix.Translation(-center))
                for original in templates:
                    obj = original.copy()
                    bpy.context.scene.collection.objects.link(obj)
                    obj.name = f"maize_{row}_{side}_{index}"
                    obj.matrix_world = matrix @ original.matrix_world
                    objects.append(obj)
    for obj in templates:
        bpy.data.objects.remove(obj, do_unlink=True)
    grass = texture_material("Grass_CC0", repository / "Media/models/materials/textures/grass_color.jpg")
    soil = texture_material("Soil_PolyHaven_CC0", soil_texture)
    objects.append(field_surface("Ground", (25, 20, 0), (60, 50), grass))
    objects.append(field_surface("Cultivated_soil", (25, 20, .005), (43, 35), soil))
    metal = bpy.data.materials.new("Fence_galvanized")
    metal.diffuse_color = (.43, .47, .48, 1)
    corners = [(-1, -3), (51, -3), (51, 43), (-1, 43)]
    for a, b in zip(corners, corners[1:] + corners[:1]):
        length = math.dist(a, b)
        segments = math.ceil(length / 4)
        for index in range(segments):
            x = a[0] + (b[0] - a[0]) * index / segments
            y = a[1] + (b[1] - a[1]) * index / segments
            bpy.ops.mesh.primitive_cylinder_add(vertices=10, radius=.055, depth=1.4, location=(x, y, .7))
            obj = bpy.context.object
            obj.name = "Fence_post"
            obj.data.materials.append(metal)
            objects.append(obj)
        direction = Vector((b[0] - a[0], b[1] - a[1], 0))
        for height in (.5, 1.1):
            bpy.ops.mesh.primitive_cylinder_add(vertices=6, radius=.018, depth=length,
                location=((a[0]+b[0])/2, (a[1]+b[1])/2, height))
            obj = bpy.context.object
            obj.name = "Fence_rail"
            obj.rotation_euler = direction.to_track_quat("Z", "Y").to_euler()
            obj.data.materials.append(metal)
            objects.append(obj)
    return objects, repository / "models/maize_02/model.config", "CC-BY-SA-4.0"


def load_tomato(repository):
    folder = repository / "gz_tomato_farm_generator"
    script = folder / "blender/tomato_gen.py"
    expected = "18e2ded8f7a728d786f7b28de82adaa5ef08ca066a97aedf2a86184920b5d432"
    if hashlib.sha256(script.read_bytes()).hexdigest() != expected:
        raise ValueError("Review the changed upstream tomato generator before importing it")
    bpy.ops.wm.open_mainfile(filepath=str(folder / "blender/tomato_gen.blend"))
    spec = importlib.util.spec_from_file_location("reviewed_aoc_tomato", script)
    module = importlib.util.module_from_spec(spec)
    saved_args = sys.argv
    try:
        sys.argv = [str(script), "--", "--seed", "42", "--model_dir", tempfile.gettempdir()]
        spec.loader.exec_module(module)
        plant = module.gen_plant(write_results=False, keep_materials=True)
    finally:
        sys.argv = saved_args
    templates = list(plant.all_objects)
    for obj in list(bpy.context.scene.objects):
        if obj not in templates:
            bpy.data.objects.remove(obj, do_unlink=True)
    # Preserve UV/material boundaries while bounding repeated crop geometry.
    total = sum(len(obj.data.polygons) for obj in templates)
    for obj in templates:
        if len(obj.data.polygons) > 32 and total > 5000:
            bpy.context.view_layer.objects.active = obj
            modifier = obj.modifiers.new("CropLOD", "DECIMATE")
            modifier.ratio = 5000 / total
            bpy.ops.object.modifier_apply(modifier=modifier.name)
    bpy.context.view_layer.update()
    objects = []
    for row, y in enumerate((-3., 3.)):
        for index in range(19):
            for original in templates:
                obj = original.copy()
                bpy.context.scene.collection.objects.link(obj)
                obj.name = f"tomato_{row}_{index}_{original.name}"
                obj.matrix_world = Matrix.Translation(Vector((-9 + index, y, .55))) @ original.matrix_world
                objects.append(obj)
    for obj in templates:
        bpy.data.objects.remove(obj, do_unlink=True)
    # The blend retains obsolete TIFF alpha nodes. The supplied colour PNGs
    # carry the matching alpha channel and must be used for portable foliage.
    for material in bpy.data.materials:
        if not material.use_nodes:
            continue
        for node in material.node_tree.nodes:
            if node.type != "BSDF_PRINCIPLED" or not node.inputs["Base Color"].is_linked:
                continue
            colour = node.inputs["Base Color"].links[0].from_node
            if colour.type == "TEX_IMAGE" and colour.image and colour.image.channels == 4:
                material.node_tree.links.new(colour.outputs["Alpha"], node.inputs["Alpha"])
    paths = folder / "templates"
    support_height = 0.0
    for kind, filename, positions in (
        ("structure", "Structure.dae", [(0., 0., 0.)]),
        ("soilbed", "SoilBed.dae", [(float(x), y, 0.) for y in (-3., 3.) for x in range(-9, 10)]),
        ("flowerpot", "maceta.dae", [(float(x), y, 0.) for y in (-3., 3.) for x in range(-9, 10)]),
    ):
        mesh = next((paths / (kind + "_model_gazebo_classic_and_sim")).rglob(filename))
        sdf = ET.parse(mesh.parent.parent / "model.sdf")
        if kind == "structure":
            parts = []
            for visual in sdf.findall(".//visual"):
                filename = Path(visual.findtext("geometry/mesh/uri")).name
                section = import_mesh(mesh.parent / filename)
                scale = list(map(float, visual.findtext("geometry/mesh/scale", "1 1 1").split()))
                material = bpy.data.materials.new("Aoc_" + visual.attrib["name"])
                material.diffuse_color = tuple(map(float, visual.findtext("material/diffuse").split()))
                for obj in section:
                    obj.matrix_world = transform(visual.findtext("pose")) @ Matrix.Diagonal(Vector([*scale, 1.])) @ obj.matrix_world
                    obj.data.materials.clear()
                    obj.data.materials.append(material)
                parts.extend(section)
        else:
            parts = import_mesh(mesh)
        # CAD presentation planes are not part of the pot/bed/greenhouse.
        for obj in list(parts):
            if obj.name.startswith("Plane"):
                parts.remove(obj)
                bpy.data.objects.remove(obj, do_unlink=True)
        if not parts:
            raise ValueError(f"No agricultural geometry in {mesh}")
        scale = [1., 1., 1.] if kind == "structure" else [float(v) for v in sdf.findtext(".//visual/geometry/mesh/scale", "1 1 1").split()]
        asset_tf = Matrix.Diagonal(Vector([*scale, 1.]))
        if kind == "soilbed":
            asset_tf = Matrix.Rotation(math.pi / 2, 4, "Z") @ asset_tf
        for obj in parts:
            obj.matrix_world = asset_tf @ obj.matrix_world
        bpy.context.view_layer.update()
        corners = [obj.matrix_world @ Vector(p) for obj in parts for p in obj.bound_box]
        center = Vector(((min(p.x for p in corners) + max(p.x for p in corners))/2,
                         (min(p.y for p in corners) + max(p.y for p in corners))/2,
                         min(p.z for p in corners)))
        for i, position in enumerate(positions):
            for original in parts:
                obj = original.copy()
                bpy.context.scene.collection.objects.link(obj)
                obj.name = f"aoc_{kind}_{i}_{original.name}"
                lift = support_height if kind == "flowerpot" else 0.0
                obj.matrix_world = Matrix.Translation(Vector(position) - center + Vector((0, 0, lift))) @ original.matrix_world
                objects.append(obj)
        if kind != "structure":
            support_height += max(p.z for p in corners) - min(p.z for p in corners)
        for obj in parts:
            bpy.data.objects.remove(obj, do_unlink=True)
    for obj in objects:
        if obj.name.startswith("tomato_"):
            obj.location.z += support_height - .55
    floor = bpy.data.materials.new("Glasshouse_concrete")
    floor.diffuse_color = (.48, .49, .47, 1.)
    objects.append(field_surface("Ground", (0, 0, 0), (36, 24), floor))
    return objects, repository / "LICENSE", "Apache-2.0"


def compact_textures(output, size):
    directory = output / "textures"
    directory.mkdir(exist_ok=True)
    for image in bpy.data.images:
        if not image.filepath or image.size[0] == 0:
            continue
        digest = hashlib.sha256(image.filepath.encode()).hexdigest()[:10]
        destination = directory / (digest + "_" + Path(image.filepath).stem + ".png")
        width, height = image.size
        ratio = min(1.0, size / max(width, height))
        if ratio < 1:
            image.scale(max(1, round(width * ratio)), max(1, round(height * ratio)))
        image.filepath_raw = str(destination)
        image.file_format = "PNG"
        image.save()


def load_vegetables(repository, soil_texture):
    revision = subprocess.check_output(["git", "-C", str(repository), "rev-parse", "HEAD"], text=True).strip()
    if revision != "7d8ebd6e52ade4ce8e0c89b67e2f048ec9e6989e":
        raise ValueError("Review the changed row-crop assets before importing them")
    if soil_texture is None or not soil_texture.is_file():
        raise ValueError("Vegetable scene requires the Poly Haven brown_mud diffuse texture")
    objects = []
    randomizer = random.Random(42)
    plots = (("artichoke", -6, -4.5, 5, 2, 1.5, 1.8, 4000),
             ("zucchini", 6, -4.5, 6, 3, 1.25, 1.25, 4000),
             ("chard", -6, 4.5, 13, 5, .55, .6, 1800),
             ("lettuce", 6, 4.5, 13, 5, .55, .6, 1800))
    for family, cx, cy, columns, rows, dx, dy, face_budget in plots:
        variants = []
        for path in sorted((repository / "plants" / family).glob("*/*.obj"))[:2]:
            parts = import_mesh(path)
            total = sum(len(obj.data.polygons) for obj in parts)
            for obj in parts:
                bpy.context.view_layer.objects.active = obj
                modifier = obj.modifiers.new("PlantVisualLOD", "DECIMATE")
                modifier.ratio = min(1., face_budget / total)
                bpy.ops.object.modifier_apply(modifier=modifier.name)
            bpy.context.view_layer.update()
            bounds = [obj.matrix_world @ Vector(corner) for obj in parts for corner in obj.bound_box]
            center = Vector(((min(v.x for v in bounds) + max(v.x for v in bounds)) / 2,
                             (min(v.y for v in bounds) + max(v.y for v in bounds)) / 2,
                             min(v.z for v in bounds)))
            variants.append((parts, center))
        if len(variants) != 2:
            raise ValueError(f"Two reviewed variants are required for {family}")
        for row in range(rows):
            for column in range(columns):
                parts, center = variants[(row + column) % len(variants)]
                position = Vector((cx + (column - (columns - 1) / 2) * dx,
                                   cy + (row - (rows - 1) / 2) * dy, .01))
                matrix = (Matrix.Translation(position)
                          @ Matrix.Rotation(randomizer.uniform(-.25, .25), 4, "Z")
                          @ Matrix.Translation(-center))
                for part, original in enumerate(parts):
                    obj = original.copy()
                    bpy.context.scene.collection.objects.link(obj)
                    obj.name = f"vegetable_{family}_{row}_{column}_{part}"
                    obj.matrix_world = matrix @ original.matrix_world
                    objects.append(obj)
        for parts, _ in variants:
            for obj in parts:
                bpy.data.objects.remove(obj, do_unlink=True)
    soil = texture_material("Soil_PolyHaven_CC0", soil_texture)
    lane = bpy.data.materials.new("Compacted_field_track")
    lane.diffuse_color = (.38, .34, .27, 1)
    objects.append(field_surface("Ground", (0, 0, 0), (44, 30), lane))
    for family, cx, cy, *_ in plots:
        objects.append(field_surface("vegetable_bed_" + family, (cx, cy, .005), (8.4, 4.4), soil))
    fence = bpy.data.materials.new("Vegetable_field_fence")
    fence.diffuse_color = (.42, .43, .39, 1)
    # A normal perimeter fence provides finite field boundaries, not hidden
    # localization aids; crops remain at their original metric scale.
    edges = [((-20, -13), (20, -13)), ((20, -13), (20, 13)),
             ((20, 13), (-20, 13)), ((-20, 13), (-20, 2)),
             ((-20, -2), (-20, -13))]
    for edge, (a, b) in enumerate(edges):
        vector = Vector((b[0] - a[0], b[1] - a[1], 0))
        length = vector.length
        yaw = math.atan2(vector.y, vector.x)
        for index in range(math.ceil(length / .22)):
            fraction = (index + .5) / math.ceil(length / .22)
            bpy.ops.mesh.primitive_cube_add(size=1, location=Vector((a[0], a[1], .6)) + vector * fraction)
            obj = bpy.context.object
            obj.name = f"vegetable_fence_picket_{edge}_{index}"
            obj.scale = (.12, .05, 1.2)
            obj.rotation_euler.z = yaw
            obj.data.materials.append(fence)
            objects.append(obj)
        for height in (.25, .9):
            bpy.ops.mesh.primitive_cube_add(size=1,
                location=((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, height))
            obj = bpy.context.object
            obj.name = f"vegetable_fence_rail_{edge}_{height}"
            obj.scale = (length, .07, .08)
            obj.rotation_euler.z = yaw
            obj.data.materials.append(fence)
            objects.append(obj)
    return objects, repository / "README.md", "MIT"


def load_strawberry(repository):
    models = repository / "harvester_gazebo/models"
    model = read_harvester_geometry(models / "random_strawberry_plant/model.sdf")
    cache, plant_parts = {}, []
    for link in model.findall("link"):
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                raise ValueError("Unexpected strawberry visual geometry")
            path = models / mesh.findtext("uri").removeprefix("model://")
            if path not in cache:
                cache[path] = import_mesh(path)
            scale = Matrix.Diagonal(Vector([*map(float, mesh.findtext("scale", "1 1 1").split()), 1]))
            color = visual.findtext("material/diffuse")
            material = None
            if color:
                material = bpy.data.materials.get("strawberry_stem") or bpy.data.materials.new("strawberry_stem")
                material.diffuse_color = tuple(map(float, color.split()))
            for original in cache[path]:
                if not original.data.polygons:
                    continue
                obj = original.copy()
                obj.data = original.data.copy()
                bpy.context.scene.collection.objects.link(obj)
                obj.matrix_world = transform(link.findtext("pose")) @ transform(visual.findtext("pose")) @ scale @ original.matrix_world
                if material:
                    obj.data.materials.clear()
                    obj.data.materials.append(material)
                plant_parts.append(obj)
    for originals in cache.values():
        for obj in originals:
            bpy.data.objects.remove(obj, do_unlink=True)
    bpy.ops.object.select_all(action="DESELECT")
    for obj in plant_parts:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = plant_parts[0]
    bpy.ops.object.join()
    plant = bpy.context.object
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    modifier = plant.modifiers.new("PlantVisualLOD", "DECIMATE")
    modifier.ratio = min(1, 6000 / len(plant.data.polygons))
    bpy.ops.object.modifier_apply(modifier=modifier.name)
    objects = []
    randomizer = random.Random(42)
    for row, y in enumerate((-6, 0, 6)):
        for side in (-.4, .4):
            for index in range(12):
                obj = plant.copy()
                bpy.context.scene.collection.objects.link(obj)
                obj.name = f"strawberry_{row}_{side}_{index}"
                obj.matrix_world = (Matrix.Translation(Vector((-5.5 + index, y + side, .45)))
                    @ Matrix.Rotation(randomizer.uniform(0, 2 * math.pi), 4, "Z"))
                objects.append(obj)
    bpy.data.objects.remove(plant, do_unlink=True)
    beds = import_mesh(models / "bed/meshes/bed.dae")
    for row, y in enumerate((-6, 0, 6)):
        for segment, x in enumerate((-4.5, -1.5, 1.5, 4.5)):
            for original in beds:
                if not original.data.polygons:
                    continue
                obj = original.copy()
                bpy.context.scene.collection.objects.link(obj)
                obj.name = f"strawberry_bed_{row}_{segment}"
                obj.matrix_world = Matrix.Translation(Vector((x, y, 0))) @ original.matrix_world
                objects.append(obj)
    for obj in beds:
        bpy.data.objects.remove(obj, do_unlink=True)
    soil = texture_material("Harvester_dirt", models / "dirt_plane/materials/textures/dirt_texture.png")
    objects.append(field_surface("Ground", (0, 0, -.002), (32, 26), soil))
    wood = bpy.data.materials.new("Field_fence_wood")
    wood.diffuse_color = (.34, .27, .18, 1)
    # A normal enclosed growing plot, with a 3.6 m entrance on the west side.
    # Bare parallel beds otherwise provide almost no along-row lidar constraint.
    edges = [((-14, -10.5), (14, -10.5)), ((14, -10.5), (14, 10.5)),
             ((14, 10.5), (-14, 10.5)), ((-14, 10.5), (-14, 1.8)),
             ((-14, -1.8), (-14, -10.5))]
    for edge, (a, b) in enumerate(edges):
        vector = Vector((b[0] - a[0], b[1] - a[1], 0))
        length = vector.length
        yaw = math.atan2(vector.y, vector.x)
        for index in range(math.ceil(length / .18)):
            fraction = (index + .5) / math.ceil(length / .18)
            position = Vector((a[0], a[1], .6)) + vector * fraction
            bpy.ops.mesh.primitive_cube_add(size=1, location=position)
            obj = bpy.context.object
            obj.name = f"field_fence_picket_{edge}_{index}"
            obj.scale = (.13, .045, 1.2)
            obj.rotation_euler.z = yaw
            obj.data.materials.append(wood)
            objects.append(obj)
        for height in (.25, .9):
            bpy.ops.mesh.primitive_cube_add(size=1,
                location=((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, height))
            obj = bpy.context.object
            obj.name = f"field_fence_rail_{edge}_{height}"
            obj.scale = (length, .07, .08)
            obj.rotation_euler.z = yaw
            obj.data.materials.append(wood)
            objects.append(obj)
    # Cameras/lights bundled inside upstream COLLADA files are not scene assets.
    for obj in list(bpy.context.scene.objects):
        if obj.type != "MESH":
            bpy.data.objects.remove(obj, do_unlink=True)
    return objects, repository / "LICENSE", "BSD-2-Clause"


def load_vineyard(repository, soil_texture):
    expected = "7128cd2acade50cc4a5a1761210b55989ab62527"
    commit = subprocess.check_output(["git", "-C", str(repository), "rev-parse", "HEAD"], text=True).strip()
    if commit != expected:
        raise ValueError("Review the changed CropCraft assets before importing them")
    if soil_texture is None or not soil_texture.is_file():
        raise ValueError("Vineyard scene requires the reviewed Poly Haven brown_mud texture")
    plants = []
    for variant in range(1, 5):
        parts = import_mesh(repository / f"assets/plants/vine1/vine_{variant:02d}.obj")
        bpy.ops.object.select_all(action="DESELECT")
        for obj in parts:
            obj.select_set(True)
        bpy.context.view_layer.objects.active = parts[0]
        bpy.ops.object.join()
        plant = bpy.context.object
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        plants.append(plant)
    bpy.context.view_layer.update()
    objects = []
    randomizer = random.Random(42)
    for row, y in enumerate((-6, 0, 6)):
        for index in range(11):
            plant = plants[(row + index) % len(plants)]
            bounds = [Vector(p) for p in plant.bound_box]
            center = Vector(((min(v.x for v in bounds) + max(v.x for v in bounds)) / 2,
                             (min(v.y for v in bounds) + max(v.y for v in bounds)) / 2,
                             min(v.z for v in bounds)))
            obj = plant.copy()
            bpy.context.scene.collection.objects.link(obj)
            obj.name = f"vine_{row}_{index}"
            obj.matrix_world = (Matrix.Translation(Vector((-10 + 2 * index, y, 0)))
                @ Matrix.Rotation(randomizer.uniform(-.05, .05), 4, "Z")
                @ Matrix.Translation(-center))
            objects.append(obj)
    for plant in plants:
        bpy.data.objects.remove(plant, do_unlink=True)
    # Trellis posts and wires are local geometry, not part of CropCraft's plants.
    support = bpy.data.materials.new("Vineyard_galvanized_trellis")
    support.diffuse_color = (.47, .5, .48, 1)
    for row, y in enumerate((-6, 0, 6)):
        for index, x in enumerate((-11, -5, 1, 7, 11)):
            bpy.ops.mesh.primitive_cylinder_add(vertices=10, radius=.045, depth=1.8,
                                                location=(x, y, .9))
            obj = bpy.context.object
            obj.name = f"trellis_post_{row}_{index}"
            obj.data.materials.append(support)
            objects.append(obj)
        for level in (.75, 1.4):
            bpy.ops.mesh.primitive_cylinder_add(vertices=6, radius=.008, depth=22,
                                                location=(0, y, level))
            obj = bpy.context.object
            obj.rotation_euler.y = math.pi / 2
            obj.name = f"trellis_wire_{row}_{level}"
            obj.data.materials.append(support)
            objects.append(obj)
    soil = texture_material("Vineyard_soil_PolyHaven_CC0", soil_texture)
    objects.append(field_surface("Ground", (0, 0, -.002), (44, 30), soil))
    return objects, repository / "LICENSE", "Apache-2.0"


def export(objects, destination):
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    bpy.ops.export_scene.obj(filepath=str(destination), use_selection=True,
                             axis_forward="Y", axis_up="Z", use_triangles=True,
                             path_mode="RELATIVE", use_materials=True)


def clip_height(points, height, above):
    result = []
    for a, b in zip(points, points[1:] + points[:1]):
        inside_a = a.z >= height if above else a.z <= height
        inside_b = b.z >= height if above else b.z <= height
        if inside_a:
            result.append(a)
        if inside_a != inside_b:
            result.append(a + (b - a) * ((height - a.z) / (b.z - a.z)))
    return result


def occupancy(objects, output, source, resolution=.05):
    bpy.context.view_layer.update()
    bounds = [obj.matrix_world @ Vector(corner) for obj in objects for corner in obj.bound_box]
    minimum = [math.floor(min(v[i] for v in bounds)) - 2 for i in range(2)]
    maximum = [math.ceil(max(v[i] for v in bounds)) + 2 for i in range(2)]
    width, height = [round((b - a) / resolution) for a, b in zip(minimum, maximum)]
    if width * height > 100_000_000:
        raise ValueError("Unexpected scene extent; verify upstream mesh units and SDF scales")
    image = Image.new("L", (width, height), 254)
    draw = ImageDraw.Draw(image)
    for obj in objects:
        if obj.get("navigation_surface", False):
            continue
        obj.data.calc_loop_triangles()
        vertices = [obj.matrix_world @ v.co for v in obj.data.vertices]
        for triangle in obj.data.loop_triangles:
            points = [vertices[i] for i in triangle.vertices]
            if source != "cpr":
                if max(v.z for v in points) < .18 or min(v.z for v in points) > 2.2:
                    continue
                points = clip_height(clip_height(points, .18, True), 2.2, False)
            if len(points) >= 3:
                pixels = [((v.x - minimum[0]) / resolution, (maximum[1] - v.y) / resolution) for v in points]
                draw.polygon(pixels, fill=0)
                draw.line(pixels + pixels[:1], fill=0, width=1)
    image.save(output / "map.pgm")
    preview = image.copy()
    preview.thumbnail((1200, 1000))
    preview.save(output / "map_preview.png")
    (output / "map.yaml").write_text(f"image: map.pgm\nresolution: {resolution}\norigin: [{minimum[0]}, {minimum[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    return dict(origin=minimum, maximum=maximum, resolution=resolution,
                heightBand=None if source == "cpr" else [.18, 2.2],
                projection="non_terrain_geometry" if source == "cpr" else "height_band")


def world_file(output, source):
    root = ET.Element("sdf", version="1.6")
    world = ET.SubElement(root, "world", name=source)
    ET.SubElement(ET.SubElement(world, "include"), "uri").text = "model://sun"
    physics = ET.SubElement(world, "physics", name="ode", type="ode")
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_update_rate").text = "1000"
    scene = ET.SubElement(world, "scene")
    ET.SubElement(scene, "ambient").text = "0.55 0.55 0.55 1"
    ET.SubElement(scene, "background").text = "0.65 0.78 0.9 1"
    ET.SubElement(scene, "shadows").text = "true"
    gui = ET.SubElement(world, "gui")
    camera = ET.SubElement(gui, "camera", name="overview")
    ET.SubElement(camera, "pose").text = {
        "cpr": "20 25 40 0 0.9 -2.35",
        "harbot": "20 -25 23 0 0.65 2.35",
        "maize": "-12 -18 36 0 0.65 0.75",
        "tomato": "-17 -20 17 0 0.6 0.85",
        "strawberry": "-17 -20 17 0 0.6 0.85",
        "vineyard": "-22 -24 20 0 0.6 0.85",
        "vegetables": "-26 -26 25 0 0.65 0.8",
    }[source]
    model = ET.SubElement(world, "model", name="agricultural_environment")
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="environment")
    for tag, mesh_name in (("visual", "scene.obj"), ("collision", "collision.obj")):
        node = ET.SubElement(link, tag, name=tag)
        ET.SubElement(ET.SubElement(ET.SubElement(node, "geometry"), "mesh"), "uri").text = mesh_name
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(output / "world.sdf", encoding="utf-8", xml_declaration=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", choices=("cpr", "harbot", "maize", "tomato", "strawberry", "vineyard", "vegetables"), required=True)
    parser.add_argument("--repository", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--texture-size", type=int, default=2048)
    parser.add_argument("--soil-texture", type=Path)
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    output = args.output.resolve()
    if output.exists():
        parser.error("Output must be new; do not overwrite a previously prepared scene")
    output.mkdir(parents=True)
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    if args.source == "cpr":
        package = args.repository / "cpr_agriculture_gazebo"
        objects = import_mesh(package / "meshes/agriculture_world.dae")
        for obj in objects:
            obj.matrix_world = Matrix.Rotation(math.pi, 4, "Z") @ obj.matrix_world
            obj["navigation_surface"] = obj.name == "Ground"
        license_file, license_id = package / "docs/LICENSE.txt", "BSD-3-Clause"
    elif args.source == "harbot":
        objects, license_file, license_id = load_harbot(args.repository)
    elif args.source == "tomato":
        objects, license_file, license_id = load_tomato(args.repository)
    elif args.source == "strawberry":
        objects, license_file, license_id = load_strawberry(args.repository)
    elif args.source == "vineyard":
        objects, license_file, license_id = load_vineyard(args.repository, args.soil_texture)
    elif args.source == "vegetables":
        objects, license_file, license_id = load_vegetables(args.repository, args.soil_texture)
    else:
        objects, license_file, license_id = load_maize(args.repository, args.soil_texture)
    compact_textures(output, args.texture_size)
    export(objects, output / "scene.obj")
    visual_faces = sum(len(obj.data.polygons) for obj in objects)
    collision_objects = []
    for visual in objects:
        obj = visual.copy()
        obj.data = visual.data.copy()
        obj.name = visual.name + "_collision"
        bpy.context.scene.collection.objects.link(obj)
        bpy.context.view_layer.objects.active = obj
        count = len(obj.data.polygons)
        if count > 10000:
            modifier = obj.modifiers.new("CollisionLOD", "DECIMATE")
            modifier.ratio = 10000 / count
            bpy.ops.object.modifier_apply(modifier=modifier.name)
        collision_objects.append(obj)
    export(collision_objects, output / "collision.obj")
    # Project the same collision mesh that Gazebo uses, excluding the floor and roof.
    map_info = occupancy(collision_objects, output, args.source)
    world_file(output, args.source)
    shutil.copy2(license_file, output / "LICENSE.upstream.txt")
    source = dict(repository=subprocess.check_output(["git", "-C", str(args.repository), "remote", "get-url", "origin"], text=True).strip(),
                  commit=subprocess.check_output(["git", "-C", str(args.repository), "rev-parse", "HEAD"], text=True).strip(),
                  license=license_id, visualFaces=visual_faces,
                  collisionFaces=sum(len(obj.data.polygons) for obj in collision_objects), map=map_info,
                  modifications=["ROS-independent static world", "bounded texture resolution", "visual/collision LOD", "occupancy projected from collision geometry"])
    if args.source == "harbot":
        source["modifications"].append("external decorative trees omitted; greenhouse, crops and fruit retained")
        source["modifications"].append("existing greenhouse shell rendered as translucent milky plastic film; geometry and occupancy unchanged")
        source["plasticFilm"] = dict(materials=["glass", "glass.001", "glass.002"],
                                     opacity=.28, colour=[.88, .92, .86], roughness=.65,
                                     geometry="existing upstream shell", collisionChanged=False)
    if args.source == "maize":
        source["modifications"].append("photogrammetric maize arranged in five paired rows; normalized plant height 1.2m")
        source["author"] = "Kamaro Engineering e.V. (2021), maize_02"
        source["additionalAssets"] = [
            {"name": "Brown Mud", "url": "https://polyhaven.com/a/brown_mud", "author": "Rob Tuytel", "license": "CC0-1.0"},
            {"name": "Ground003", "url": "https://ambientcg.com/view?id=Ground003", "author": "CC0Textures.com (2018)", "license": "CC0-1.0"}]
    if args.source == "tomato":
        source["modifications"].append("seed-42 AOC plants, two 19-plant rows, centered glasshouse and concrete floor")
        source["modifications"].append("SDF structure sections and metric scales retained; soil beds rotated along rows; PNG alpha reused for foliage")
        source["generatorSha256"] = "18e2ded8f7a728d786f7b28de82adaa5ef08ca066a97aedf2a86184920b5d432"
    if args.source == "strawberry":
        source["modifications"].append("pinned SDF unquoted attributes parsed without executing ROS1 plugins")
        source["modifications"].append("three metric raised-bed rows, 72 textured plants; plant visual LOD about 6000 faces")
        source["modifications"].append("local procedural 1.2m picket perimeter fence with 3.6m west entrance; not an upstream asset")
        source["geometrySdfSha256"] = HARVESTER_SDF_SHA256
    if args.source == "vineyard":
        source["modifications"].append("33 vines using four upstream variants, metric scale preserved, three 6m-spaced rows")
        source["modifications"].append("local procedural 1.8m trellis posts and wires; not upstream assets")
        source["additionalAssets"] = [
            {"name": "Brown Mud", "url": "https://polyhaven.com/a/brown_mud", "author": "Rob Tuytel", "license": "CC0-1.0"}]
        source["plantAssets"] = [dict(path=str(path.relative_to(args.repository)),
            sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            for path in sorted((args.repository / "assets/plants/vine1").iterdir()) if path.is_file()]
    if args.source == "vegetables":
        source["modifications"].extend([
            "Four crop plots: 10 artichokes, 18 zucchini, 65 chard and 65 lettuce; two upstream variants per family",
            "Original metric scale retained; plant roots placed on the field surface; visual LOD 1800/4000 faces per plant",
            "Local procedural field tracks and 1.2m perimeter fence with 4m west entrance; not upstream assets"])
        source["author"] = "Marco Ambrosio, Brenno Tuberga (2022)"
        source["licenseEvidence"] = "Upstream README states all repository code and models are MIT and contains the full license; retained verbatim"
        source["additionalAssets"] = [
            {"name": "Brown Mud", "url": "https://polyhaven.com/a/brown_mud", "author": "Rob Tuytel", "license": "CC0-1.0"}]
        source["plantAssets"] = [dict(path=str(path.relative_to(args.repository)),
            sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            for family in ("artichoke", "zucchini", "chard", "lettuce")
            for folder in sorted((args.repository / "plants" / family).iterdir())[:2]
            for path in sorted(folder.rglob("*")) if path.is_file() and path.suffix != ".blend"]
    source["objects"] = []
    for obj in objects:
        bounds = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
        source["objects"].append(dict(name=obj.name,
            minimum=[min(v[i] for v in bounds) for i in range(3)],
            maximum=[max(v[i] for v in bounds) for i in range(3)],
            navigationSurface=bool(obj.get("navigation_surface", False))))
    source["surfaceHeights"] = []
    for obj in objects:
        if not obj.get("navigation_surface", False):
            continue
        inverse = obj.matrix_world.inverted()
        for x, y in ((0, 0), (-10, 0), (-10, -10), (10, 0)):
            hit, point, _, _ = obj.ray_cast(inverse @ Vector((x, y, 50)),
                                          inverse.to_3x3() @ Vector((0, 0, -1)))
            if hit:
                source["surfaceHeights"].append([x, y, (obj.matrix_world @ point).z])
    (output / "source.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(source), flush=True)


if __name__ == "__main__":
    main()
