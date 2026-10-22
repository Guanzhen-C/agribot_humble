#!/usr/bin/env python3
"""Create local-only Unity previews from the exact generated URDF geometry.

blender -b --python prepare_unity_farm.py -- --unity-project PATH --workspace-src PATH
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import struct
import sys
import xml.etree.ElementTree as ET

import bpy
from mathutils import Euler, Matrix, Vector


def transform(element):
    if element is None:
        return Matrix.Identity(4)
    xyz = [float(x) for x in element.get("xyz", "0 0 0").split()]
    rpy = [float(x) for x in element.get("rpy", "0 0 0").split()]
    return Matrix.Translation(Vector(xyz)) @ Euler(rpy, "XYZ").to_matrix().to_4x4()


def preview_color(material, fallback):
    if material is None:
        return fallback
    from agribot_farm_sim.materials import canonical_diffuse
    color = canonical_diffuse(material.name, material.diffuse_color[:3]) + (material.diffuse_color[3],)
    if material.use_nodes:
        shader = next((node for node in material.node_tree.nodes if node.type == "BSDF_PRINCIPLED"), None)
        if shader is not None:
            links = shader.inputs["Base Color"].links
            if links and links[0].from_node.type == "TEX_IMAGE":
                image = links[0].from_node.image
                if image is not None and image.channels >= 3:
                    pixels = image.pixels[:]
                    stride = max(1, len(pixels) // image.channels // 1024) * image.channels
                    samples = [pixels[offset:offset+3] for offset in range(0, len(pixels), stride)]
                    color = tuple(color[axis] * sum(value[axis] for value in samples) / len(samples)
                                  for axis in range(3)) + (color[3],)
    return color


def preview(bundle, workspace, output):
    config = json.loads((bundle / "vehicle_config.json").read_text())
    robot = ET.parse(bundle / "urdf" / (config["vehicleId"] + ".urdf.xacro")).getroot()
    # Moving belts are rendered by Gazebo/RViz, not URDF links. Include their
    # zero-displacement poses in the same metric Unity preview.
    sys.path.insert(0, str(workspace / "agribot_vehicle_config_tools"))
    from agribot_vehicle_config_tools.visual_belts import BeltPath, belts
    base = robot.find("link[@name='base_link']")
    for belt in belts(config):
        path = BeltPath(belt["points"])
        for part in belt["segments"]:
            x, z, angle = path.sample(part["distance"])
            visual = ET.SubElement(base, "visual")
            ET.SubElement(visual, "origin", xyz=f"{x} {belt['centerY']} {z}", rpy=f"0 {angle} 0")
            ET.SubElement(ET.SubElement(visual, "geometry"), "mesh", filename=
                          config["visuals"][0]["meshUri"].rsplit("/", 1)[0] + "/" + part["meshFile"])
    transforms = {"base_link": Matrix.Identity(4)}
    remaining = list(robot.findall("joint"))
    while remaining:
        changed = False
        for joint in list(remaining):
            parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
            if parent not in transforms:
                continue
            transforms[child] = transforms[parent] @ transform(joint.find("origin"))
            remaining.remove(joint)
            changed = True
        if not changed:
            raise ValueError("URDF has an unresolved link hierarchy")
    groups = {}
    low, high = [float("inf")] * 3, [-float("inf")] * 3
    for link in robot.findall("link"):
        for visual in link.findall("visual"):
            bpy.ops.object.select_all(action="SELECT")
            bpy.ops.object.delete(use_global=False)
            for mesh in list(bpy.data.meshes):
                if mesh.users == 0:
                    bpy.data.meshes.remove(mesh)
            for material in list(bpy.data.materials):
                if material.users == 0:
                    bpy.data.materials.remove(material)
            geometry = visual.find("geometry")
            node = geometry[0]
            color = visual.find("material/color")
            color = tuple(float(v) for v in color.get("rgba").split()) if color is not None else (.2, .24, .27, 1.)
            if node.tag == "mesh":
                uri = node.get("filename")
                if not uri.startswith("package://"):
                    raise ValueError("only package-local preview meshes are allowed")
                path = workspace / uri[len("package://"):]
                bpy.ops.import_scene.obj(filepath=str(path.resolve()), axis_forward="Y", axis_up="Z", use_split_objects=True)
                scale = Matrix.Diagonal(Vector([float(x) for x in node.get("scale", "1 1 1").split()] + [1.]))
            elif node.tag == "box":
                bpy.ops.mesh.primitive_cube_add(size=1)
                bpy.context.object.scale = [float(v) for v in node.get("size").split()]
                scale = Matrix.Identity(4)
            elif node.tag == "cylinder":
                bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=float(node.get("radius")), depth=float(node.get("length")))
                scale = Matrix.Identity(4)
            else:
                raise ValueError("unsupported URDF visual: " + node.tag)
            bpy.context.view_layer.update()
            for obj in list(bpy.context.scene.objects):
                if obj.type != "MESH":
                    continue
                matrix = transforms[link.get("name")] @ transform(visual.find("origin")) @ scale @ obj.matrix_world
                mesh = obj.data
                mesh.calc_loop_triangles()
                # AFM1 is a flat-colour preview. Sample paint textures instead of
                # showing their untextured white multiplier as the vehicle colour.
                colors = [preview_color(slot.material, color) for slot in obj.material_slots]
                for triangle in mesh.loop_triangles:
                    rgba = color
                    if colors:
                        rgba = colors[triangle.material_index]
                    key = (link.get("name"), rgba)
                    points = groups.setdefault(key, [])
                    for index in triangle.vertices:
                        point = matrix @ mesh.vertices[index].co
                        points.extend(point)
                        for i in range(3):
                            low[i], high[i] = min(low[i], point[i]), max(high[i], point[i])
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("wb") as stream:
        stream.write(b"AFM1")
        stream.write(struct.pack("<I", len(groups)))
        for (name, color), points in groups.items():
            encoded = name.encode("utf-8")
            stream.write(struct.pack("<I", len(encoded)))
            stream.write(encoded)
            stream.write(struct.pack("<4fI", *color, len(points) // 9))
            stream.write(struct.pack("<" + "f" * len(points), *points))
    return {"meshGroups": len(groups), "triangles": sum(len(p) // 9 for p in groups.values()),
            "minimumRos": low, "maximumRos": high, "previewSha256": hashlib.sha256(output.read_bytes()).hexdigest()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--unity-project", type=Path, required=True)
    parser.add_argument("--workspace-src", type=Path, required=True)
    parser.add_argument("--asset-root", type=Path, default=Path.home() / ".local/share/agribot/farm_assets")
    parser.add_argument("--source", action="append", default=[], metavar="ID=PATH",
                        help="Prepare only these explicitly selected canonical exports")
    parser.add_argument("--appearance-only", action="store_true",
                        help="Prepare a scene vehicle's visual overlay, not a complete farm preset")
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from agribot_farm_sim.materials import normalize_ambient
    sources = {"compact_tractor": args.asset_root / "compact_tractor/ros_v6",
               "tractor_chisel": args.asset_root / "mounted/tractor_chisel_v4",
               "tractor_harrow": args.asset_root / "mounted/tractor_harrow_v4"}
    if args.source:
        sources = {}
        for item in args.source:
            key, separator, path = item.partition("=")
            if not separator or not key or Path(key).name != key or key in (".", ".."):
                parser.error("--source requires ID=PATH with a plain vehicle ID")
            if key in sources:
                parser.error("duplicate vehicle source: " + key)
            sources[key] = Path(path).expanduser().resolve()
    for key, source in sources.items():
        bundle = args.workspace_src / "agribot_vehicle_description/generated" / key
        asset_group = "LocalVehicleAppearances" if args.appearance_only else "LocalFarmExports"
        preview_group = "LocalVehicleAppearances" if args.appearance_only else "LocalFarmAssets"
        raw = args.unity_project / "Assets/StreamingAssets" / asset_group / key
        display = args.unity_project / "Assets/AgribotConfig/Resources" / preview_group / key
        raw.mkdir(parents=True, exist_ok=True)
        config = json.loads((source / "vehicle_config.json").read_text())
        if config["vehicleId"] != key:
            raise ValueError("source vehicle ID mismatch: " + key)
        if config != json.loads((bundle / "vehicle_config.json").read_text()):
            raise ValueError("source config and runtime bundle disagree: " + key)
        files = []
        for path in source.iterdir():
            if path.suffix.lower() in (".obj", ".mtl", ".png", ".jpg", ".jpeg") or path.name == "vehicle_config.json":
                target = raw / path.name
                shutil.copy2(path, target)
                if path.suffix == ".mtl":
                    normalize_ambient(target)
                files.append({"name": path.name, "sha256": hashlib.sha256(target.read_bytes()).hexdigest()})
        info = preview(bundle, args.workspace_src, display / "preview.bytes")
        info.update(vehicleId=key, schemaVersion=1, files=sorted(files, key=lambda item: item["name"]),
                    redistributionLicense="unverified_user_supplied", sourceConfigSha256=hashlib.sha256((source / "vehicle_config.json").read_bytes()).hexdigest())
        (raw / "asset_manifest.json").write_text(json.dumps(info, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps({"vehicleId": key, **info}, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
