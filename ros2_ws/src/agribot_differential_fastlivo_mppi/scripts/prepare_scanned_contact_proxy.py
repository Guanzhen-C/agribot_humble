#!/usr/bin/env python3
"""Separate scanned appearance/rays from the firm-ground contact proxy.

Only low scan triangles in map-traversable cells are replaced by reconstructed
terrain. Map thresholds and the planner's unknown-space policy are respected;
mapped obstacles and elevated geometry remain. The source scene is unchanged.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from scipy import ndimage
import yaml

from prepare_scanned_route_validation import stl_triangles, sha256, nav2_free_pixels
from add_scanned_ground_support import write_stl


def terrain_grid(triangles, stats):
    cell, width, height = stats["cell_m"], stats["width"], stats["height"]
    points = triangles.reshape(-1, 3)
    origin = points[:, :2].min(0)
    index = np.rint((points[:, :2]-origin)/cell).astype(int)
    grid = np.full((height, width), np.nan)
    grid[index[:, 1], index[:, 0]] = points[:, 2]
    if not np.isfinite(grid).all():
        raise ValueError("Support mesh must be a complete regular terrain grid")
    return origin, grid


class ContactMask:
    def __init__(self, image, geometry, ground, ground_origin, ground_cell, height_limit=.45,
                 allow_unknown=False):
        self.resolution = float(geometry["resolution"])
        self.map_origin = np.asarray(geometry["origin"][:2])
        self.h, self.w = image.shape
        blocked = ndimage.binary_dilation(~nav2_free_pixels(image, geometry, include_unknown=allow_unknown),
                                          iterations=max(1, math.ceil(.1/self.resolution)))
        self.integral = np.pad(blocked.astype(np.int32), ((1, 0), (1, 0))).cumsum(0).cumsum(1)
        self.ground, self.ground_origin = ground, ground_origin
        self.ground_cell, self.height_limit = ground_cell, height_limit

    def classify(self, triangles):
        low = np.floor((triangles[:, :, :2].min(1)-self.map_origin)/self.resolution).astype(int)
        high = np.floor((triangles[:, :, :2].max(1)-self.map_origin)/self.resolution).astype(int)
        inside = (low >= 0).all(1) & (high < [self.w, self.h]).all(1)
        x0, y0 = np.clip(low, 0, [self.w-1, self.h-1]).T
        x1, y1 = (np.clip(high, 0, [self.w-1, self.h-1])+1).T
        grid = self.integral
        occupied = grid[y1, x1]-grid[y0, x1]-grid[y1, x0]+grid[y0, x0]
        free = inside & (occupied == 0)
        points = np.concatenate((triangles, triangles.mean(1)[:, None]), axis=1)
        coords = (points[:, :, :2]-self.ground_origin)/self.ground_cell
        sampled = ndimage.map_coordinates(self.ground, [coords[:, :, 1].ravel(), coords[:, :, 0].ravel()],
                                         order=1, mode="constant", cval=np.nan).reshape(-1, 4)
        delta = points[:, :, 2]-sampled
        low_surface = np.isfinite(delta).all(1) & (delta <= self.height_limit).all(1)
        keep = ~(free & low_surface)
        has_free = occupied < (x1-x0)*(y1-y0)
        mixed = keep & has_free & (delta <= self.height_limit).any(1)
        return keep, mixed


def retained_contacts(triangles, image, geometry, ground, ground_origin,
                      ground_cell, height_limit=.45):
    return ContactMask(image, geometry, ground, ground_origin, ground_cell, height_limit).classify(triangles)[0]


def refined_contacts(triangles, mask, max_edge=.1):
    """Split mixed-region faces instead of keeping their entire large bbox."""
    pending, retained, removed = triangles, [], 0
    for depth in range(9):
        if not len(pending):
            break
        if len(pending) > 2_000_000:
            raise ValueError("Ambiguous collision refinement exceeds preparation bound")
        keep, mixed = mask.classify(pending)
        edges = pending-np.roll(pending, 1, axis=1)
        split = mixed & (np.linalg.norm(edges, axis=2).max(1) > max_edge) & (depth < 8)
        retained.append(pending[keep & ~split])
        removed += int((~keep).sum())
        a, b, c = pending[split].transpose(1, 0, 2)
        ab, ac, bc = (a+b)/2, (a+c)/2, (b+c)/2
        pending = np.concatenate([np.stack(t, axis=1) for t in ((a, ab, ac), (ab, b, bc),
                                                                              (ac, bc, c), (ab, bc, ac))])
    result = np.concatenate(retained) if retained else np.empty((0, 3, 3))
    return result, removed


def prepare(bundle, height_limit):
    report = json.loads((bundle/"preparation.json").read_text())
    if "contact_proxy" in report:
        raise ValueError("Contact proxy was already prepared; use a new validation bundle")
    protected = report["original_source_hashes"]
    if any(sha256(path) != value for path, value in protected.items()):
        raise ValueError("A protected scene/map/route source changed")
    world = ET.parse(bundle/"world.sdf")
    scene = world.find("world/model[@name='k2_automation_sensor_scene']")
    pose = np.array(list(map(float, scene.findtext("pose").split())))
    rotation = np.array([[math.cos(pose[5]), -math.sin(pose[5])],
                         [math.sin(pose[5]), math.cos(pose[5])]])
    terrain_origin, ground = terrain_grid(stl_triangles(bundle/"ground_support.stl"), report["ground_support"])
    geometry = yaml.safe_load((bundle/"map.yaml").read_text())
    image = np.asarray(Image.open(geometry["image"]))[::-1]
    mask = ContactMask(image, geometry, ground, terrain_origin,
                       report["ground_support"]["cell_m"], height_limit,
                       allow_unknown=report.get("allow_unknown", False))
    parts = []
    for link in scene.findall("link"):
        for original in list(link.findall("collision")):
            uri = original.find("geometry/mesh/uri")
            if uri is None:
                continue
            source = Path(uri.text.removeprefix("file://"))
            if not source.is_absolute():
                source = Path(report["source_scene"])/source
            local = stl_triangles(source)
            vertices = local.copy()
            vertices[:, :, :2] = vertices[:, :, :2] @ rotation.T+pose[:2]
            vertices[:, :, 2] += pose[2]
            refined, removed = refined_contacts(vertices, mask)
            proxy = copy.deepcopy(original)
            proxy.set("name", original.get("name")+"_hard_contact")
            if len(refined):
                output = bundle/(original.get("name")+"_hard_contact.stl")
                refined[:, :, :2] = (refined[:, :, :2]-pose[:2]) @ rotation
                refined[:, :, 2] -= pose[2]
                write_stl(output, refined)
                proxy.find("geometry/mesh/uri").text = output.as_uri()
                link.append(proxy)
            # Keep the exact original mesh in the ray scene without applying
            # response forces. Separate hard geometry above supplies contact.
            surface = original.find("surface")
            if surface is None:
                surface = ET.SubElement(original, "surface")
            contact = surface.find("contact")
            if contact is None:
                contact = ET.SubElement(surface, "contact")
            ET.SubElement(contact, "collide_without_contact").text = "true"
            # ODE rays have their own intersection callback. No physical
            # contact pairs are needed for this retained sensor-only mesh.
            ET.SubElement(contact, "collide_without_contact_bitmask").text = "0"
            parts.append({"source": str(source), "original_triangles": len(local),
                          "hard_contact_triangles": len(refined),
                          "terrain_replaced_leaf_triangles": removed})
    if len(parts) != 16:
        raise ValueError("Expected the full 16-part imported collision scene")
    ET.indent(world, space="  ")
    world.write(bundle/"world.sdf", encoding="utf-8", xml_declaration=True)
    report["contact_proxy"] = {"height_above_terrain_limit_m": height_limit,
                               "requires_map_traversable_bbox": True,
                               "mapped_obstacle_buffer_m": .1,
                               "original_ray_meshes_retained": True,
                               "ray_only_contact_bitmask": 0,
                               "mixed_region_refinement_edge_m": .1,
                               "free_space_rule": "Nav2 trinary free_thresh and negate, not a fixed white cutoff",
                               "unknown_low_ground_traversable": report.get("allow_unknown", False),
                               "parts": parts}
    report["scope"] += (" Contact separates low scan roughness in verified free cells from hard obstacles; "
                        "full original mesh remains for sensor rays. Firm-ground approximation, not soil mechanics.")
    (bundle/"preparation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report["contact_proxy"], indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    parser.add_argument("--height-limit", type=float, default=.45)
    args = parser.parse_args()
    if not math.isfinite(args.height_limit) or not .1 <= args.height_limit <= 1.:
        raise ValueError("Low scan terrain limit must be between 10 cm and 1 m")
    prepare(args.bundle.resolve(), args.height_limit)


if __name__ == "__main__":
    main()
