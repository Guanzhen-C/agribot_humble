"""Preserve the full original K2 Gaussians and the original collision meshes."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import shlex
import struct
import subprocess
import xml.etree.ElementTree as ET
import zipfile

import cv2
import numpy as np
from PIL import Image
import vtk
from vtk.util.numpy_support import vtk_to_numpy
import yaml


def read_mesh(path):
    reader = vtk.vtkPLYReader()
    reader.SetFileName(str(path))
    reader.Update()
    triangles = vtk.vtkTriangleFilter()
    triangles.SetInputConnection(reader.GetOutputPort())
    triangles.Update()
    mesh = triangles.GetOutput()
    vertices = vtk_to_numpy(mesh.GetPoints().GetData()).copy()
    faces = vtk_to_numpy(mesh.GetPolys().GetData()).reshape(-1, 4)
    if not np.all(faces[:, 0] == 3) or not np.isfinite(vertices).all():
        raise ValueError("Expected finite triangulated K2 geometry")
    return vertices, faces[:, 1:].copy(), mesh


def append_gaussian_visual(link, directory):
    visual = ET.SubElement(link, "visual", name="original_full_gaussians")
    box = ET.SubElement(ET.SubElement(visual, "geometry"), "box")
    ET.SubElement(box, "size").text = ".000001 .000001 .000001"
    ET.SubElement(visual, "cast_shadows").text = "false"
    plugin = ET.SubElement(visual, "plugin", name="k2_original_gaussians",
                           filename=str(directory / "libK2GaussianVisual.so"))
    ET.SubElement(plugin, "directory").text = str(directory)
    return visual


def append_vehicle_light(world):
    light = ET.SubElement(world, "light", name="vehicle_daylight", type="directional")
    ET.SubElement(light, "cast_shadows").text = "false"
    ET.SubElement(light, "pose").text = "0 0 10 0 0 0"
    ET.SubElement(light, "diffuse").text = ".9 .9 .9 1"
    ET.SubElement(light, "specular").text = ".1 .1 .1 1"
    ET.SubElement(light, "direction").text = "-.4 .2 -1"
    return light


def compile_renderer(reference, destination):
    source = Path(__file__).resolve().parents[1] / "plugins/K2GaussianVisual.cpp"
    if not source.is_file():
        from ament_index_python.packages import get_package_share_directory
        source = Path(get_package_share_directory("agribot_k2_sim")) / "plugins/K2GaussianVisual.cpp"
    flags = shlex.split(subprocess.check_output(["pkg-config", "--cflags", "gazebo"], text=True))
    renderer = (reference / "renderer") if reference is not None else source.parent.parent / "vendor/renderer"
    command = ["/usr/bin/g++", *flags, "-O2", "-std=c++17", "-Wno-register", "-shared", "-fPIC",
               "-I/usr/include/OGRE", "-I/usr/include/OGRE/Paging", "-I/usr/include/OGRE/Terrain",
               "-I/usr/include/OGRE/RTShaderSystem", "-I/usr/include/eigen3",
               "-I" + str(renderer / "include"), str(source),
               str(renderer / "src/splat_cloud.cpp"), str(renderer / "src/cpu_sorter.cpp"),
               str(renderer / "src/perf_local.cpp"), "-o", str(destination / "libK2GaussianVisual.so"),
               "-lgazebo_rendering", "-lgazebo_common", "-lOgreMain", "-lGLEW", "-lGL",
               "-lsdformat9", "-lignition-math6", "-lboost_system", "-lboost_filesystem", "-pthread"]
    subprocess.run(command, check=True)


def write_world(destination, collisions, z_shift, camera_pose, final_directory=None):
    """Keep rendered Gaussians and physics under the same rigid transform."""
    final = final_directory or destination
    sdf = ET.Element("sdf", version="1.6")
    world = ET.SubElement(sdf, "world", name="k2_automation_world")
    ET.SubElement(world, "gravity").text = "0 0 -9.81"
    physics = ET.SubElement(world, "physics", name="ode", type="ode")
    ET.SubElement(physics, "max_step_size").text = "0.002"
    ET.SubElement(physics, "real_time_update_rate").text = "500"
    scene = ET.SubElement(world, "scene")
    for key, value in (("ambient", "1 1 1 1"), ("background", ".15 .18 .2 1"),
                       ("shadows", "false"), ("grid", "false")):
        ET.SubElement(scene, key).text = value
    append_vehicle_light(world)
    model = ET.SubElement(world, "model", name="k2_automation_sensor_scene")
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"0 0 {z_shift} 0 0 0"
    link = ET.SubElement(model, "link", name="scanned_environment")
    append_gaussian_visual(link, final / "gaussian")
    for i, collision in enumerate(collisions):
        element = ET.SubElement(link, "collision", name=f"collision_{i:02d}")
        mesh = ET.SubElement(ET.SubElement(element, "geometry"), "mesh")
        ET.SubElement(mesh, "uri").text = (final / "meshes" / collision.name).as_uri()
        ode = ET.SubElement(ET.SubElement(ET.SubElement(element, "surface"), "friction"), "ode")
        ET.SubElement(ode, "mu").text = "1.0"
        ET.SubElement(ode, "mu2").text = "1.0"
    camera = ET.SubElement(ET.SubElement(world, "gui", fullscreen="0"), "camera", name="user_camera")
    ET.SubElement(camera, "pose").text = " ".join(map(str, camera_pose))
    ET.indent(sdf)
    ET.ElementTree(sdf).write(destination / "world.sdf", encoding="utf-8", xml_declaration=True)


def project_map(meshes, ground_z, destination, resolution=.1):
    low = np.min([vertices.min(axis=0) for vertices, _ in meshes], axis=0)
    high = np.max([vertices.max(axis=0) for vertices, _ in meshes], axis=0)
    origin = np.floor((low[:2] - 1) / resolution) * resolution
    width, height = np.ceil((high[:2] + 1 - origin) / resolution).astype(int)
    if width * height > 25_000_000:
        raise ValueError("Unexpected K2 map extent")
    floor = np.zeros((height, width), dtype=np.uint8)
    obstacle = floor.copy()
    for vertices, faces in meshes:
        xyz = vertices[faces]
        normals = np.cross(xyz[:, 1] - xyz[:, 0], xyz[:, 2] - xyz[:, 0])
        norm = np.linalg.norm(normals, axis=1)
        flat = np.abs(normals[:, 2]) > .85 * np.maximum(norm, 1e-12)
        zmin, zmax = xyz[:, :, 2].min(axis=1), xyz[:, :, 2].max(axis=1)
        ground = flat & (zmin >= ground_z - .4) & (zmax < ground_z + .18)
        blocked = (zmax >= ground_z + .20) & (zmin < ground_z + 2.2)
        pixels = np.rint((xyz[:, :, :2] - origin) / resolution).astype(np.int32)
        for polygon in pixels[ground]:
            cv2.fillConvexPoly(floor, polygon, 255)
        for polygon in pixels[blocked]:
            cv2.fillConvexPoly(obstacle, polygon, 255)
    floor = cv2.morphologyEx(floor, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    occupancy = np.full_like(floor, 205)
    occupancy[floor > 0] = 254
    occupancy[obstacle > 0] = 0
    Image.fromarray(np.flipud(occupancy)).save(destination / "map.pgm")
    Image.fromarray(np.flipud(occupancy)).save(destination / "map_preview.png")
    (destination / "map.yaml").write_text(yaml.safe_dump({
        "image": "map.pgm", "resolution": resolution, "origin": [*origin.tolist(), 0.0],
        "negate": 0, "occupied_thresh": .65, "free_thresh": .196,
    }, sort_keys=False), encoding="utf-8")
    return {"resolution": resolution, "origin": origin.tolist(), "width": int(width),
            "height": int(height), "ground_reference_z": ground_z,
            "obstacle_band_m": [.20, 2.2], "unknown_outside_observed_ground": True}


def prepare(reference, archive, destination):
    reference, archive, destination = map(lambda p: Path(p).resolve(),
                                          (reference, archive, destination))
    if destination.exists():
        raise FileExistsError("Use a new destination; existing scenes are never overwritten")
    source = reference / "source"
    config = json.loads((reference / "scene_config.json").read_text())
    manifest = json.loads((source / config["source_manifest"]).read_text())
    # Verify that reused decoded inputs really belong to the requested archive.
    with zipfile.ZipFile(archive) as zf:
        names = [config["source_manifest"], *manifest["root"]["meshFiles"]]
        for name in names:
            if zf.read(name) != (source / name).read_bytes():
                raise ValueError(f"Archive/reference mismatch: {name}")
    cache = reference / "gaussians.bin"
    with cache.open("rb") as stream:
        magic, count = struct.unpack("<8sQ", stream.read(16))
    if magic != b"LCCGS001" or count != config["render_gaussians"]:
        raise ValueError("Invalid decoded Gaussian cache")
    destination.mkdir(parents=True)
    gaussian_dir = destination / "gaussian"
    gaussian_dir.mkdir()
    shutil.copy2(cache, gaussian_dir / "gaussians.bin")
    shutil.copy2(reference / "scene_config.json", gaussian_dir / "scene_config.json")
    shutil.copy2(reference / "renderer/LICENSE", gaussian_dir / "RENDERER_LICENSE")
    shutil.copytree(reference / "shaders", gaussian_dir / "shaders")
    compile_renderer(reference, gaussian_dir)
    print(f"Copied all {count} original Gaussians; no color baking or resampling", flush=True)
    geometry_dir = destination / "meshes"
    geometry_dir.mkdir()
    meshes, collisions, reports = [], [], []
    for i, relative in enumerate(manifest["root"]["meshFiles"]):
        vertices, faces, geometry = read_mesh(source / relative)
        meshes.append((vertices, faces))
        collision = geometry_dir / f"collision_{i:02d}.stl"
        writer = vtk.vtkSTLWriter()
        writer.SetFileName(str(collision))
        writer.SetFileTypeToBinary()
        writer.SetInputData(geometry)
        if writer.Write() != 1:
            raise RuntimeError("Collision export failed")
        collisions.append(collision)
        reports.append({"source": relative, "triangles": len(faces), "vertices": len(vertices)})
        print(f"Mesh {i + 1}/16: {len(faces)} original triangles", flush=True)
    spawn = json.loads((reference / "vehicle_demo/spawn_config.json").read_text())
    ground_z = float(spawn["ground_z"])
    z_shift = -ground_z
    map_report = project_map(meshes, ground_z, destination)
    write_world(destination, collisions, z_shift, [3.6, -21.5, -.4 + z_shift, 0, .23, 2.15])
    waypoints = [{"x": .8, "y": -15.5, "yaw": math.pi / 2},
                 {"x": .8, "y": -12.5, "yaw": math.pi / 2}]
    (destination / "waypoints.yaml").write_text(yaml.safe_dump({"waypoints": waypoints}), encoding="utf-8")
    external = {"schema_version": 1, "id": "k2_automation",
                "world": "world.sdf", "map": "map.yaml", "waypoints": "waypoints.yaml",
                "spawn": [.8, -17.5, math.pi / 2], "spawn_ground_z": 0.0}
    (destination / "scene.json").write_text(json.dumps(external, indent=2) + "\n")
    report = {"source_archive": str(archive), "source_archive_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
              "reference": str(reference), "source_gaussians": count,
              "source_collision_triangles": sum(item["triangles"] for item in reports),
              "mesh_reports": reports, "map": map_report, "scene_translation_z_m": z_shift,
              "appearance": "All original highest-resolution Gaussians and environment; EWA rendering, no mesh appearance proxy",
              "server_camera_validation": "pending; inspect gaussian/render_status_server.json and actual ROS images",
              "collision_geometry": "All original source triangles; no substituted ground plane",
              "runtime_validated": False, "rtk": "Simulated GNSS from Gazebo; not recorded K2 RTK measurements"}
    (destination / "preparation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"Prepared scene manifest: {destination / 'scene.json'}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    prepare(args.reference, args.archive, args.output)


if __name__ == "__main__":
    main()
