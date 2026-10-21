#!/usr/bin/env python3
"""Separate inspected Car9 CAD rotors and apply a combine paint material."""

import argparse
import json
import math
from pathlib import Path
import shutil
import sys

import bpy
import bmesh
from mathutils import Vector, Matrix
from mathutils.geometry import convex_hull_2d


GROUPS = {
    "rear_right": [1341, 1344], "rear_left": [1342, 1343],
    "right_sprocket": [400], "left_sprocket": [424],
    "right_idler_back": list(range(401, 407)),
    "right_idler_front": list(range(410, 416)),
    "left_idler_front": list(range(425, 431)),
    "left_idler_back": list(range(434, 440)),
    **{"right_roller_" + str(i): [416 + 2*i, 417 + 2*i] for i in range(4)},
    **{"left_roller_" + str(i): [440 + 2*i, 441 + 2*i] for i in range(4)},
}


def export(objects, path):
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    bpy.ops.export_scene.obj(filepath=str(path), use_selection=True, axis_forward="Y", axis_up="Z",
                             use_triangles=True, use_materials=True, path_mode="STRIP")


def extract_faces(obj, selected, name, remove=False):
    indices = {face.index for face in selected}
    mesh = obj.data.copy()
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bmesh.ops.delete(bm, geom=[f for f in bm.faces if f.index not in indices], context="FACES")
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context="VERTS")
    bm.to_mesh(mesh)
    bm.free()
    result = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(result)
    if remove:
        bm = bmesh.new()
        bm.from_mesh(obj.data)
        bmesh.ops.delete(bm, geom=[f for f in bm.faces if f.index in indices], context="FACES")
        bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context="VERTS")
        bm.to_mesh(obj.data)
        bm.free()
        obj.data.update()
    return result


def track_path(obj):
    from agribot_vehicle_config_tools.visual_belts import BeltPath
    points = [Vector((v.co.x, v.co.z)) for v in obj.data.vertices]
    hull = [points[i] for i in convex_hull_2d(points)]
    if sum(a.x*b.y-b.x*a.y for a,b in zip(hull, hull[1:]+hull[:1])) > 0:
        hull.reverse()
    center = sum(hull, Vector((0, 0))) / len(hull)
    # A reference curve inside the CAD rubber belt, not a new contact shape.
    points = [list(p - .06*(p-center).normalized()) for p in hull]
    return BeltPath(points)


def tire_surface_faces(obj, center):
    parent = list(range(len(obj.data.vertices)))
    def root(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index
    for face in obj.data.polygons:
        for index in face.vertices[1:]:
            parent[root(index)] = root(face.vertices[0])
    components = {}
    for face in obj.data.polygons:
        components.setdefault(root(face.vertices[0]), []).append(face)
    result = []
    for faces in components.values():
        points = [obj.data.vertices[i].co for face in faces for i in face.vertices]
        low = Vector([min(p[i] for p in points) for i in range(3)])
        high = Vector([max(p[i] for p in points) for i in range(3)])
        middle, size = (low+high)/2, high-low
        if (abs(middle.x-center.x) < .03 and abs(middle.y-center.y) < .04
                and 1.4 < size.x < 2 and abs(size.x-size.z) < .03 and .4 < size.y < .7):
            result.extend(faces)
    return result


def prepare_belt(objects, key, path, config, output, materials):
    from agribot_vehicle_config_tools.visual_belts import BeltPath
    all_points = [v.co for obj in objects for v in obj.data.vertices]
    y = (min(p.y for p in all_points)+max(p.y for p in all_points))/2
    for obj in objects:
        bm = bmesh.new()
        bm.from_mesh(obj.data)
        for _ in range(6):
            edges = [e for e in bm.edges if e.calc_length() > .16]
            if not edges:
                break
            bmesh.ops.subdivide_edges(bm, edges=edges, cuts=1, use_grid_fill=True)
        bmesh.ops.triangulate(bm, faces=list(bm.faces))
        bm.to_mesh(obj.data)
        bm.free()
        obj.data.update()
    count = 64
    buckets = [[] for _ in range(count)]
    for obj in objects:
        for face in obj.data.polygons:
            distance = path.project(face.center.x, face.center.z)[1] % path.length
            buckets[min(int(distance/path.length*count), count-1)].append((obj, face))
    segments = []
    for index, faces in enumerate(buckets):
        if not faces:
            continue
        distance = (index+.5)*path.length/count
        x, z, angle = path.sample(distance)
        inverse = Matrix.Rotation(-angle, 3, "Y")
        vertices, triangles = [], []
        for obj, face in faces:
            start = len(vertices)
            vertices.extend(inverse @ (obj.data.vertices[i].co-Vector((x,y,z))) for i in face.vertices)
            triangles.append(tuple(range(start, len(vertices))))
        mesh = bpy.data.meshes.new("belt_segment")
        mesh.from_pydata(vertices, [], triangles)
        mesh.materials.append(materials["rubber"])
        pad = bpy.data.objects.new("belt_"+key+"_"+str(index), mesh)
        bpy.context.collection.objects.link(pad)
        filename = pad.name + ".obj"
        export([pad], output / filename)
        segments.append(dict(index=index, distance=distance, meshFile=filename))
        bpy.data.objects.remove(pad, do_unlink=True)
    return dict(id=key, sourceWheel="front_"+key, sourceRadius=next(
        w["radiusM"] for w in config["geometry"]["wheelCenters"] if w["id"] == "front_"+key),
        centerY=y, points=path.points, segments=segments)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--texture", type=Path, required=True)
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "agribot_vehicle_config_tools"))
    if args.output.exists():
        parser.error("Use a new appearance directory; previous assets are retained")
    shutil.copytree(args.source, args.output)
    config = json.loads(args.config.read_text())
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    bpy.ops.import_scene.obj(filepath=str(args.source / "vehicle_visual.obj"), axis_forward="Y", axis_up="Z", use_split_objects=True)
    objects = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    solids = {int(o.name.split("_")[1]): o for o in objects if o.name.startswith("solid_")}
    groups = {key: [solids[index] for index in indices] for key, indices in GROUPS.items()}
    open_shell = solids[1345]
    for key in ("rear_left", "rear_right"):
        points = [v.co for obj in groups[key] for v in obj.data.vertices]
        center = Vector([(min(p[i] for p in points)+max(p[i] for p in points))/2 for i in range(3)])
        faces = tire_surface_faces(open_shell, center)
        if not faces:
            raise ValueError("Rear tire open tread surfaces were not found: " + key)
        shell = extract_faces(open_shell, faces, key+"_tire_tread", remove=True)
        tire_z = (min(v.co.z for v in shell.data.vertices)+max(v.co.z for v in shell.data.vertices))/2
        # The earlier CAD preparation lowered only the tire core, not the open
        # tread surfaces. Restore their common visual axle; physics is untouched.
        for obj in groups[key]:
            for vertex in obj.data.vertices:
                vertex.co.z += tire_z-center.z
        groups[key].append(shell)
        objects.append(shell)
    belt_objects, paths = {}, {}
    for key, index in (("left",433), ("right",409)):
        path = track_path(solids[index])
        paths[key] = path
        y = sum(v.co.y for v in solids[index].data.vertices)/len(solids[index].data.vertices)
        faces = [f for f in open_shell.data.polygons if abs(f.center.y-y) < .45
                 and path.project(f.center.x, f.center.z)[0] < .16
                 and all(abs(open_shell.data.vertices[i].co.y-y) < .5 for i in f.vertices)]
        belt_objects[key] = [solids[index]]
        if faces:
            shell = extract_faces(open_shell, faces, key+"_belt_tread", remove=True)
            belt_objects[key].append(shell)
            objects.append(shell)
    materials = {}
    for name, rgba in (("combine_yellow", (.9, .9, .9, 1)), ("rubber", (.045, .05, .055, 1)),
                       ("steel", (.32, .36, .39, 1)), ("rim_yellow", (.9, .64, .035, 1)),
                       ("cab_glass", (.035, .085, .12, 1)), ("chassis", (.09, .105, .115, 1))):
        material = bpy.data.materials.new(name)
        material.diffuse_color = rgba
        material.use_nodes = True
        shader = material.node_tree.nodes.get("Principled BSDF")
        shader.inputs["Base Color"].default_value = rgba
        shader.inputs["Roughness"].default_value = .72 if name == "rubber" else .35
        if name == "combine_yellow":
            texture = material.node_tree.nodes.new("ShaderNodeTexImage")
            texture.image = bpy.data.images.load(str(args.texture.resolve()))
            material.node_tree.links.new(texture.outputs["Color"], shader.inputs["Base Color"])
        materials[name] = material
    for obj in objects:
        old = obj.data.materials[0].name if obj.data.materials else ""
        kind = "rubber" if old == "rubber" else "steel" if "steel" in old else "combine_yellow"
        obj.data.materials.clear()
        for material in materials.values():
            obj.data.materials.append(material)
        for face in obj.data.polygons:
            center = face.center
            selected = kind
            if obj.name.startswith("solid_1345"):
                if 2.45 < center.x < 3.98 and 1.95 < center.z < 3.25 and abs(center.y) < .96:
                    selected = "cab_glass"
                elif center.z < 1.15:
                    selected = "chassis"
            face.material_index = list(materials).index(selected)
        if not obj.data.uv_layers:
            uv = obj.data.uv_layers.new(name="PaintUV")
            for face in obj.data.polygons:
                axis = max(range(3), key=lambda i: abs(face.normal[i]))
                axes = [i for i in range(3) if i != axis]
                for loop in face.loop_indices:
                    point = obj.data.vertices[obj.data.loops[loop].vertex_index].co
                    uv.data[loop].uv = (point[axes[0]], point[axes[1]])
    rotors, moving = [], set()
    for key, selected in groups.items():
        points = [vertex.co for obj in selected for vertex in obj.data.vertices]
        low = Vector([min(p[i] for p in points) for i in range(3)])
        high = Vector([max(p[i] for p in points) for i in range(3)])
        center = (low + high) / 2
        radius = max(high.x-low.x, high.z-low.z) / 2
        source = key if key.startswith("rear_") else "front_" + ("left" if key.startswith("left") else "right")
        wheel = next(w for w in config["geometry"]["wheelCenters"] if w["id"] == source)
        for obj in selected:
            for face in obj.data.polygons:
                radial = ((face.center.x-center.x)**2 + (face.center.z-center.z)**2)**.5
                face.material_index = list(materials).index("rubber" if radial > .74*radius else "rim_yellow")
            for vertex in obj.data.vertices:
                vertex.co -= center
        filename = "rotor_" + key + ".obj"
        export(selected, args.output / filename)
        rotors.append(dict(id=key, sourceWheel=source, center=list(center),
                           ratio=wheel.get("radiusM", config["geometry"]["wheelRadiusM"])/radius,
                           meshFile=filename))
        moving.update(selected)
    visual_belts = []
    for key, selected in belt_objects.items():
        visual_belts.append(prepare_belt(selected, key, paths[key], config, args.output, materials))
        moving.update(selected)
    export([obj for obj in objects if obj not in moving], args.output / "vehicle_visual.obj")
    # All exported objects use the same named materials and local texture.
    material_groups = {}
    all_parts = rotors + [part for belt in visual_belts for part in belt["segments"]]
    for path in [args.output / "vehicle_visual.mtl", *[args.output / item["meshFile"].replace(".obj", ".mtl") for item in all_parts]]:
        for group in path.read_text().split("newmtl ")[1:]:
            material_groups[group.splitlines()[0]] = "newmtl " + group.strip() + "\n\n"
    for item in all_parts:
        path = args.output / item["meshFile"]
        lines = path.read_text().splitlines()
        path.write_text("\n".join("mtllib vehicle_visual.mtl" if line.startswith("mtllib ") else line for line in lines) + "\n")
        path.with_suffix(".mtl").unlink()
    texture_name = args.texture.name
    shutil.copy2(args.texture, args.output / texture_name)
    material_path = args.output / "vehicle_visual.mtl"
    material_path.write_text("".join(material_groups.values()).replace(str(args.texture.resolve()), texture_name))
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from agribot_farm_sim.materials import normalize_ambient
    normalize_ambient(material_path)
    config["metadata"]["simulationVisualRotors"] = rotors
    config["metadata"]["simulationVisualBelts"] = visual_belts
    config["metadata"]["simulationAppearanceRevision"] = "combine_yellow_complete_tire_belts_v2"
    (args.output / "vehicle_config.json").write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(dict(output=str(args.output), rotors=len(rotors),
                          fixedObjects=len(objects)-len(moving), physicalParametersChanged=False)))


if __name__ == "__main__":
    main()
