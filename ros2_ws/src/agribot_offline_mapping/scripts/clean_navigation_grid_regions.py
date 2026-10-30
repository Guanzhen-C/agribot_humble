#!/usr/bin/env python3
"""Clean reviewed road/soil regions in a Nav2 grid without changing the PCD."""

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage
import yaml


def load_sibling(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def disk(radius):
    y, x = np.ogrid[-radius:radius + 1, -radius:radius + 1]
    return x*x + y*y <= radius*radius


def region_mask(shape, origin, resolution, region):
    coordinates = (np.asarray(region["coordinates"], dtype=float) - origin) / resolution - 0.5
    if coordinates.ndim != 2 or coordinates.shape[1] != 2 or not np.isfinite(coordinates).all():
        raise ValueError("Region coordinates must be finite XY pairs")
    image = Image.new("1", (shape[1], shape[0]))
    draw = ImageDraw.Draw(image)
    xy = [tuple(p) for p in coordinates]
    if region["kind"] == "polygon":
        if len(xy) < 3:
            raise ValueError("A polygon needs three vertices")
        draw.polygon(xy, fill=1)
    elif region["kind"] == "corridor":
        width = float(region["width_m"])
        if width <= 0 or len(xy) < 2:
            raise ValueError("A corridor needs positive width and at least two points")
        pixels = max(1, round(width/resolution))
        draw.line(xy, fill=1, width=pixels, joint="curve")
        radius = pixels/2
        for x, y in xy:
            draw.ellipse((x-radius, y-radius, x+radius, y+radius), fill=1)
    else:
        raise ValueError("Unknown region kind")
    return np.asarray(image, dtype=bool)


def fit_floor(points):
    if len(points) < 100:
        raise ValueError("Not enough surface observations")
    minimum = points[:, :2].min(0)
    indices = np.floor((points[:, :2]-minimum)/0.4).astype(int)
    width = indices[:, 0].max()+1
    keys = indices[:, 1]*width+indices[:, 0]
    order = np.lexsort((points[:, 2], keys))
    _, starts, counts = np.unique(keys[order], return_index=True, return_counts=True)
    selected = order[starts+np.floor((counts-1)*0.3).astype(int)]
    samples = points[selected]
    center = samples[:, :2].mean(0)
    design = np.column_stack([samples[:, :2]-center, np.ones(len(samples))])
    inliers = np.ones(len(samples), dtype=bool)
    for _ in range(6):
        if inliers.sum() < 20 or np.linalg.matrix_rank(design[inliers]) < 3:
            raise ValueError("Surface observations do not span a plane")
        coefficients = np.linalg.lstsq(design[inliers], samples[inliers, 2], rcond=None)[0]
        residual = samples[:, 2]-design@coefficients
        median = np.median(residual[inliers])
        mad = np.median(np.abs(residual[inliers]-median))
        inliers = np.abs(residual-median) <= max(0.15, 3.5*1.4826*mad)
    if np.linalg.norm(coefficients[:2]) > 0.15:
        raise ValueError("Region surface is too steep to auto-clean")
    if inliers.mean() < 0.5:
        raise ValueError("Region lacks a dominant ground surface")
    residual_p95 = float(np.quantile(np.abs(residual[inliers]), 0.95))
    if residual_p95 > 0.22:
        raise ValueError("Region contains inconsistent surface levels; split or review it")
    height = np.column_stack([points[:, :2]-center, np.ones(len(points))])@coefficients
    return points[:, 2]-height, {
        "plane_origin_xy": center.tolist(), "plane_coefficients": coefficients.tolist(),
        "plane_inlier_fraction": float(inliers.mean()),
        "plane_residual_p95_m": residual_p95,
    }


def structural_mask(seed, counts, resolution):
    connected = ndimage.binary_dilation(seed, structure=disk(1))
    labels, number = ndimage.label(connected, structure=np.ones((3, 3)))
    area = np.bincount(labels.ravel())*resolution**2
    observations = np.bincount(labels.ravel(), weights=counts.ravel(), minlength=number+1)
    keep = (area >= 0.08) | (observations >= 10)
    keep[0] = False
    result = keep[labels]
    return ndimage.binary_dilation(result, structure=disk(max(1, int(np.ceil(0.1/resolution)))))


def review_region(cells, points, origin, resolution, region):
    mask = region_mask(cells.shape, origin, resolution, region)
    rows, columns = np.nonzero(mask)
    clear, protected = np.zeros(cells.shape, bool), np.zeros(cells.shape, bool)
    if not len(rows):
        return clear, protected, {"name": region["name"], "skipped": "outside map"}
    y0, y1, x0, x1 = rows.min(), rows.max()+1, columns.min(), columns.max()+1
    crop = mask[y0:y1, x0:x1]
    indices = np.floor((points[:, :2]-origin)/resolution).astype(int)
    valid = ((indices[:, 0] >= x0) & (indices[:, 0] < x1)
             & (indices[:, 1] >= y0) & (indices[:, 1] < y1))
    selected = np.flatnonzero(valid)
    selected = selected[crop[indices[selected, 1]-y0, indices[selected, 0]-x0]]
    subset = points[selected]
    try:
        height, stats = fit_floor(subset)
    except ValueError as error:
        return clear, protected, {"name": region["name"], "skipped": str(error)}
    xy = indices[selected]-[x0, y0]
    ground = np.zeros(crop.shape, bool)
    threshold = float(region.get("ground_tolerance_m", 0.16))
    good = np.abs(height) <= threshold
    ground[xy[good, 1], xy[good, 0]] = True
    if good.sum() < 100:
        return clear, protected, {"name": region["name"], "skipped": "insufficient ground support"}
    distance = ndimage.distance_transform_edt(~ground, sampling=resolution)
    observed = distance <= float(region.get("ground_support_radius_m", 0.3))
    counts = np.zeros(crop.shape, np.uint32)
    high = (height >= 0.40) & (height <= 2.5)
    np.add.at(counts, (xy[high, 1], xy[high, 0]), 1)
    obstacles = structural_mask(counts > 0, counts, resolution)
    depths = np.zeros(crop.shape, np.uint32)
    low = height <= -0.35
    np.add.at(depths, (xy[low, 1], xy[low, 0]), 1)
    # A lower layer is a drop-off only where the expected road/soil surface is absent.
    dropoff = structural_mask((depths > 0) & (distance > 0.15), depths, resolution)
    interior = ndimage.binary_erosion(crop, structure=disk(max(1, round(0.1/resolution))))
    clear[y0:y1, x0:x1] = interior & observed
    protected[y0:y1, x0:x1] = crop & (obstacles | dropoff)
    return clear, protected, {"name": region["name"], **stats,
                              "surface_points": len(subset), "ground_support_points": int(good.sum()),
                              "reviewed_area_m2": float(mask.sum()*resolution**2),
                              "protected_area_m2": float(protected.sum()*resolution**2)}


def clean_grid(cells, points, origin, resolution, regions):
    clear, protected = np.zeros(cells.shape, bool), np.zeros(cells.shape, bool)
    reports = []
    for region in regions:
        allowed, keep, report = review_region(cells, points, origin, resolution, region)
        clear |= allowed
        protected |= keep
        reports.append(report)
        print(json.dumps(report), flush=True)
    clear &= ~protected
    result = cells.copy()
    result[clear] = 254
    return result, clear, protected, reports


def hash_file(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_comparison(path, before, after, origin, resolution):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(15, 10), constrained_layout=True)
    extent = [origin[0], origin[0]+before.shape[1]*resolution,
              origin[1], origin[1]+before.shape[0]*resolution]
    for ax, grid, title in zip(axes, [before, after], ["Before road / bare-soil cleanup", "After: boundaries and supported structures retained"]):
        ax.imshow(grid, origin="lower", extent=extent, cmap="gray", vmin=0, vmax=255, interpolation="nearest")
        ax.set_title(title); ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    fig.savefig(path, dpi=160)
    plt.close(fig)


def process(map_base, annotations, from_backup=False):
    map_base = map_base.resolve()
    config = json.loads(annotations.read_text())
    if config["map_id"] != map_base.name:
        raise ValueError("Annotations belong to another map")
    files = {suffix: Path(str(map_base)+suffix) for suffix in [".pcd", ".pgm", ".yaml", "_georeference.yaml"]}
    identity_before = {suffix: hash_file(files[suffix]) for suffix in [".pcd", ".yaml", "_georeference.yaml"]}
    backup = Path(str(map_base)+"_before_road_cleanup.pgm")
    source = backup if from_backup else files[".pgm"]
    if from_backup:
        previous = json.loads(Path(str(map_base)+"_road_cleanup.json").read_text())
        if hash_file(files[".pgm"]) != previous["output_pgm_sha256"]:
            raise ValueError("The grid was edited since cleanup; refusing to replace it")
    source_hash = hash_file(source)
    if source_hash != config["source_pgm_sha256"]:
        raise ValueError("The source grid differs from the reviewed map; refusing cleanup")
    settings = yaml.safe_load(files[".yaml"].read_text())
    if settings.get("origin", [0, 0, 0])[2] != 0 or settings.get("negate") != 0:
        raise ValueError("Only the reviewed, axis-aligned non-negated grid is supported")
    resolution = float(settings["resolution"])
    origin = np.array(settings["origin"][:2], dtype=float)
    before = np.array(Image.open(source))[::-1].copy()
    reader = load_sibling("finalize_lio_sam_map")
    _, cloud = reader.read_binary_pcd(files[".pcd"])
    points = reader.xyz_array(cloud)
    after, cleared, protected, reports = clean_grid(before, points, origin, resolution, config["regions"])
    if backup.exists() and not from_backup:
        raise ValueError("An earlier cleanup backup exists; refusing to overwrite it")
    if not from_backup:
        shutil.copy2(files[".pgm"], backup)
    preview_path = Path(str(map_base)+"_preview.png")
    if preview_path.exists() and not from_backup:
        shutil.copy2(preview_path, Path(str(map_base)+"_before_road_cleanup_preview.png"))
    temporary = Path(str(files[".pgm"])+".tmp")
    Image.fromarray(after[::-1]).save(temporary, format="PPM")
    temporary.replace(files[".pgm"])
    for suffix, fingerprint in identity_before.items():
        if hash_file(files[suffix]) != fingerprint:
            raise RuntimeError(f"Unexpected modification to {suffix}")
    write_comparison(Path(str(map_base)+"_road_cleanup_comparison.png"), before, after, origin, resolution)
    prepared = load_sibling("prepare_lixel_navigation_map")
    _, trajectory = reader.read_binary_pcd(Path(str(map_base)+"_trajectory.pcd"))
    prepared.preview(map_base, points, cloud["rgb"], after, origin, resolution, reader.xyz_array(trajectory))
    report = {"created_at_utc": datetime.now(timezone.utc).isoformat(),
              "source_pgm_sha256": source_hash, "output_pgm_sha256": hash_file(files[".pgm"]),
              "preserved_file_sha256": identity_before, "annotations": str(annotations.resolve()),
              "backup": str(backup), "regions": reports,
              "removed_occupied_cells": int(((before == 0) & (after == 254)).sum()),
              "observed_unknown_cells_changed_to_free": int(((before == 205) & (after == 254)).sum()),
              "changed_cells": int((before != after).sum()), "protected_cells": int(protected.sum()),
              "outside_reviewed_support_unchanged": bool(np.array_equal(before[~cleared], after[~cleared])),
              "real_vehicle_validation_completed": False}
    Path(str(map_base)+"_road_cleanup.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_base", type=Path)
    parser.add_argument("annotations", type=Path)
    parser.add_argument("--from-backup", action="store_true",
                        help="Reprocess the immutable original only if the previous output is unchanged")
    arguments = parser.parse_args()
    process(arguments.map_base, arguments.annotations, arguments.from_backup)


if __name__ == "__main__":
    main()
