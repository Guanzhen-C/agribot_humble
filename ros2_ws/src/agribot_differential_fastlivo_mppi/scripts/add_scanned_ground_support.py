#!/usr/bin/env python3
"""Fill missing scan ground with a continuous terrain support collision mesh.

The lower local terrain envelope preserves large-scale grade. All original
collision meshes remain active, including walls, trees and road obstacles.
The Gaussian appearance is not changed. This models firm traversable ground,
not deformable soil, water, crop mechanics, or a measured terrain survey.
"""
import argparse
import json
import math
import os
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy import ndimage
import yaml

from prepare_scanned_route_validation import stl_triangles, surface_heights, nav2_free_pixels


def lower_terrain(vertices, lower, upper, cell=.5):
    shape = np.ceil((np.asarray(upper)-lower)/cell).astype(int)+1
    width, height = shape
    if width*height > 2_000_000:
        raise ValueError("Terrain grid exceeds the bounded preparation size")
    indices = np.floor((vertices[:, :2]-lower)/cell).astype(int)
    valid = ((indices >= 0) & (indices < shape)).all(1)
    indices, samples = indices[valid], vertices[valid, 2]
    if len(samples) < 10:
        raise ValueError("Too few observed ground samples")
    keys = indices[:, 1]*width+indices[:, 0]
    order = np.lexsort((samples, keys))
    unique, start, counts = np.unique(keys[order], return_index=True, return_counts=True)
    grid = np.full(width*height, np.nan)
    grid[unique] = samples[order[start+np.floor((counts-1)*.1).astype(int)]]
    grid = grid.reshape(height, width)
    missing = ~np.isfinite(grid)
    nearest = ndimage.distance_transform_edt(missing, return_distances=False, return_indices=True)
    filled = grid[tuple(nearest)]
    # Raised access roads are terrain, not removable objects. A wide morphology
    # opening would erase those roads and incorrectly fill scan holes too low.
    ground = ndimage.median_filter(filled, size=3)
    # Keep support below observed lower terrain; it must not bury real objects.
    ground = np.minimum(ndimage.gaussian_filter(ground, sigma=.5), filled)-.04
    x, y = np.meshgrid(lower[0]+np.arange(width)*cell, lower[1]+np.arange(height)*cell)
    points = np.stack((x, y, ground), axis=-1)
    a, b = points[:-1, :-1], points[:-1, 1:]
    c, d = points[1:, :-1], points[1:, 1:]
    triangles = np.concatenate((np.stack((a, b, c), axis=2).reshape(-1, 3, 3),
                                np.stack((b, d, c), axis=2).reshape(-1, 3, 3)))
    return triangles, {"cell_m": cell, "width": int(width), "height": int(height),
                       "observed_cells": int((~missing).sum()), "filled_cells": int(missing.sum()),
                       "support_below_observation_m": .04, "height_min_m": float(ground.min()),
                       "height_max_m": float(ground.max())}


def write_stl(path, triangles):
    normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)[:, None]
    normals = np.divide(normals, lengths, out=np.zeros_like(normals), where=lengths > 1e-12)
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    data = np.zeros(len(triangles), dtype=dtype)
    data["normal"], data["vertices"] = normals, triangles
    with path.open("xb") as stream:
        stream.write(b"Continuous support below original scan terrain".ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(data)))
        data.tofile(stream)


def heightfield_geometry(bundle, triangles, stats):
    """Use ODE's terrain collider, avoiding internal trimesh edge contacts."""
    from PIL import Image
    width, height, cell = stats["width"], stats["height"], stats["cell_m"]
    points = triangles.reshape(-1, 3)
    origin = points[:, :2].min(0)
    indices = np.rint((points[:, :2]-origin)/cell).astype(int)
    grid = np.full((height, width), np.nan)
    grid[indices[:, 1], indices[:, 0]] = points[:, 2]
    if not np.isfinite(grid).all():
        raise ValueError("Heightfield requires a complete finite terrain grid")
    side = 2**math.ceil(math.log2(max(width, height)-1))+1
    y, x = np.meshgrid(np.linspace(0, height-1, side),
                        np.linspace(0, width-1, side), indexing="ij")
    sampled = ndimage.map_coordinates(grid, [y, x], order=1, mode="nearest")
    minimum = float(sampled.min())
    span = max(float(sampled.max()-minimum), .001)
    # Gazebo 11's image height loader consumes 8-bit samples. Encoding 16-bit
    # PNG would silently read bytes rather than 16-bit elevations.
    encoded = np.rint((sampled-minimum)/span*255).astype(np.uint8)
    output = bundle/"ground_support_height.png"
    if output.exists():
        raise ValueError("Heightfield output already exists")
    Image.fromarray(encoded[::-1]).save(output)
    size = np.array([(width-1)*cell, (height-1)*cell])
    center = origin+size/2
    geometry = ET.Element("geometry")
    field = ET.SubElement(geometry, "heightmap")
    ET.SubElement(field, "uri").text = output.as_uri()
    ET.SubElement(field, "size").text = f"{size[0]:.12g} {size[1]:.12g} {span:.12g}"
    ET.SubElement(field, "pos").text = f"0 0 {minimum:.12g}"
    ET.SubElement(field, "sampling").text = "1"
    stats.update(collision_representation="ODE heightfield",
                 heightfield_pixels=side,
                 heightfield_quantization_bound_m=span/510,
                 heightfield_center_xy=center.tolist())
    return geometry, f"{center[0]:.12g} {center[1]:.12g} 0 0 0 0"


def prepare(bundle, cell):
    if (bundle/"ground_support.sdf").exists():
        raise ValueError("Ground support was already prepared")
    report = json.loads((bundle/"preparation.json").read_text())
    source = Path(report["source_scene"])
    parts = [stl_triangles(p) for p in sorted((source/"meshes").glob("collision_*.stl"))]
    original = np.concatenate(parts)
    yaw = report["scene_yaw_rad"]
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    world = ET.parse(bundle/"world.sdf")
    physics = world.find("world/physics")
    physics.find("max_step_size").text = "0.001"
    physics.find("real_time_update_rate").text = "1000"
    ode = physics.find("ode")
    if ode is None:
        ode = ET.SubElement(physics, "ode")
    for section, settings in {"solver": {"type": "quick", "iters": "50", "sor": "1.3"},
                               "constraints": {"contact_max_correcting_vel": "100", "contact_surface_layer": "0.001"}}.items():
        element = ode.find(section)
        if element is None:
            element = ET.SubElement(ode, section)
        for key, value in settings.items():
            child = element.find(key)
            if child is None:
                child = ET.SubElement(element, key)
            child.text = value
    scene = world.find("world/model[@name='k2_automation_sensor_scene']")
    pose = np.array(list(map(float, scene.findtext("pose").split())))
    original[:, :, :2] = original[:, :, :2] @ rotation.T+pose[:2]
    original[:, :, 2] += pose[2]
    normals = np.cross(original[:, 1]-original[:, 0], original[:, 2]-original[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    candidates = original[abs(normals[:, 2]) >= .8*np.maximum(lengths, 1e-9)].reshape(-1, 3)
    geometry = yaml.safe_load((bundle/"map.yaml").read_text())
    from PIL import Image
    width, height = Image.open(geometry["image"]).size
    pointcloud_source = None
    if "map_base" in report:
        import importlib.util
        from ament_index_python.packages import get_package_prefix
        reader_file = Path(get_package_prefix("agribot_hardware_bringup"))/"lib/agribot_hardware_bringup/pcd_to_nav2_map.py"
        spec = importlib.util.spec_from_file_location("scanned_ground_pcd_reader", reader_file)
        reader = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(reader)
        pointcloud_source = Path(report["map_base"]+".pcd")
        candidates = reader.read_pcd_xyz(pointcloud_source).astype(float)
        candidates[:, 2] += pose[2]
        # The manually verified map identifies ground sampling areas; building
        # roofs must not be used to fill the adjacent navigable road surface.
        image = np.asarray(Image.open(geometry["image"]))[::-1]
        index = np.floor((candidates[:, :2]-geometry["origin"][:2])/geometry["resolution"]).astype(int)
        inside = ((index >= 0) & (index < [width, height])).all(1)
        candidates, index = candidates[inside], index[inside]
        free = nav2_free_pixels(image, geometry)
        candidates = candidates[free[index[:, 1], index[:, 0]]]
    if abs(geometry["origin"][2]) > 1e-8:
        raise ValueError("Support preparation expects the real axis-aligned map")
    lower = np.array(geometry["origin"][:2]) - 2
    upper = np.array(geometry["origin"][:2])+np.array([width, height])*geometry["resolution"]+2
    triangles, stats = lower_terrain(candidates, lower, upper, cell)
    path = bundle/"ground_support.stl"
    write_stl(path, triangles)
    support = ET.SubElement(world.find("world"), "model", name="reconstructed_ground_support")
    ET.SubElement(support, "static").text = "true"
    link = ET.SubElement(support, "link", name="ground")
    collision = ET.SubElement(link, "collision", name="continuous_ground")
    geometry, collision_pose = heightfield_geometry(bundle, triangles, stats)
    ET.SubElement(collision, "pose").text = collision_pose
    collision.append(geometry)
    surface = ET.SubElement(collision, "surface")
    friction = ET.SubElement(ET.SubElement(surface, "friction"), "ode")
    for key in ("mu", "mu2"):
        ET.SubElement(friction, key).text = "100"
    contact = ET.SubElement(ET.SubElement(surface, "contact"), "ode")
    for key, value in {"kp": "100000", "kd": "1000", "max_vel": ".1", "min_depth": ".001"}.items():
        ET.SubElement(contact, key).text = value
    sdf = ET.Element("sdf", version="1.6")
    sdf.append(support)
    # ElementTree allows the same element reference in this output and world.
    ET.indent(sdf, space="  ")
    ET.ElementTree(sdf).write(bundle/"ground_support.sdf", encoding="utf-8", xml_declaration=True)
    ET.indent(world, space="  ")
    world.write(bundle/"world.sdf", encoding="utf-8", xml_declaration=True)
    route = json.loads((bundle/"saved_route.json").read_text())
    all_supported = all(len(surface_heights(triangles, [route["path"][i]["x"], route["path"][i]["y"]]))
                        for i in route["anchor_path_indices"])
    if not all_supported:
        raise ValueError("Continuous support does not cover every original route anchor")
    stats.update({"all_104_anchor_supports": all_supported, "original_collision_parts_retained": len(parts),
                  "ground_pointcloud_source": str(pointcloud_source) if pointcloud_source else None,
                  "contact_stiffness_n_m": 100000, "contact_damping_ns_m": 1000,
                  "physics_step_s": .001, "ode_solver_iterations": 50, "ode_sor": 1.3,
                  "max_contact_correction_m_s": 100.,
                  "terrain_triangle_count": len(triangles), "model": "Firm traversable terrain lower envelope; original obstacle collisions unchanged"})
    report["ground_support"] = stats
    report["scope"] += " Continuous lower-envelope ground support fills scan holes; original obstacle meshes retained."
    (bundle/"preparation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(stats, indent=2), flush=True)


def spawn(bundle):
    if os.environ.get("ROS_DOMAIN_ID") != "93" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise ValueError("Runtime terrain insertion is restricted to isolated simulation domain 93")
    import rclpy
    from gazebo_msgs.srv import SpawnEntity
    rclpy.init()
    node = rclpy.create_node("insert_scanned_ground_support")
    try:
        client = node.create_client(SpawnEntity, "/spawn_entity")
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError("Gazebo factory is not available")
        request = SpawnEntity.Request()
        request.name = "reconstructed_ground_support"
        request.xml = (bundle/"ground_support.sdf").read_text()
        request.initial_pose.orientation.w = 1.
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30)
        if not future.done() or not future.result().success:
            raise RuntimeError("Failed to insert support into the running physics world")
        print(future.result().status_message, flush=True)
        (bundle/"ground_support_spawned.txt").write_text(future.result().status_message)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    parser.add_argument("--cell", type=float, default=.5)
    parser.add_argument("--spawn", action="store_true")
    parser.add_argument("--spawn-only", action="store_true")
    args = parser.parse_args()
    if not math.isfinite(args.cell) or not .2 <= args.cell <= 1.:
        raise ValueError("Ground cell must be between 0.2 and 1 m")
    if not args.spawn_only:
        prepare(args.bundle.resolve(), args.cell)
    if args.spawn or args.spawn_only:
        spawn(args.bundle.resolve())


if __name__ == "__main__":
    main()
