#!/usr/bin/env python3
"""Export a complete Lixel scanner trace into an existing map's georeferenced frame."""

import argparse
from collections import Counter
import csv
import hashlib
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation
import yaml

from prepare_lixel_navigation_map import (
    EnuFrame, fit_trajectory, member, open_capture, read_capture, sha256_file, write_pcd,
)


def in_map_frame(gnss, poses, georeference, source_crs):
    reference = georeference["reference"]
    frame = EnuFrame(source_crs, [reference[k] for k in
                     ("longitude_deg", "latitude_deg", "altitude_m")])
    positions, stats = fit_trajectory(poses, gnss, frame.geodetic(gnss[:, 1:4]))
    if stats["horizontal_rmse_m"] > .20:
        raise ValueError("Scanner/GNSS consistency exceeds 0.20 m; do not overlay an unverified frame")
    transform = georeference["map_from_enu"]
    rotation = Rotation.from_euler("xyz", transform["rpy"]).as_matrix()
    translation = np.asarray(transform["xyz"], dtype=float)
    if not np.isfinite(rotation).all() or translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError("Invalid map_from_enu transform")
    return positions @ rotation.T + translation, stats


def nearest_quality(timestamps, rows):
    times = np.asarray([float(row["#timestamp"]) for row in rows])
    if not np.isfinite(times).all() or (np.diff(times) <= 0).any():
        raise ValueError("Invalid GNSS measurement times")
    right = np.clip(np.searchsorted(times, timestamps), 0, len(times)-1)
    left = np.maximum(0, right-1)
    nearest = np.where(abs(times[left]-timestamps) <= abs(times[right]-timestamps), left, right)
    deltas = times[nearest]-timestamps
    quality = np.asarray([int(row["status"]) for row in rows])[nearest]
    return np.where(abs(deltas) <= .15, quality, -1), deltas


def plot_overlay(base, positions, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    geometry = yaml.safe_load(base.with_suffix(".yaml").read_text())
    image = Path(geometry["image"])
    image = image if image.is_absolute() else base.parent/image
    cells = np.asarray(Image.open(image))
    origin = geometry["origin"]
    yaw = origin[2]
    rotate = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    xy = (positions[:, :2]-origin[:2]) @ rotate
    xy /= float(geometry["resolution"])
    fig, ax = plt.subplots(figsize=(12, 12), constrained_layout=True)
    ax.imshow(cells[::-1], origin="lower", cmap="gray", vmin=0, vmax=255)
    previous = Path(str(base)+"_saved_paths.json")
    old_count = None
    if previous.exists():
        document = json.loads(previous.read_text())
        if (document["map_id"] != base.name
                or document["fingerprints"]["pgm"] != sha256_file(image)
                or document["fingerprints"]["georeference"] != sha256_file(Path(str(base)+"_georeference.yaml"))):
            raise ValueError("Existing route does not match the displayed map")
        route = document["routes"][0]
        old_xy = (np.array([[p["x"], p["y"]] for p in route["path"]])-origin[:2]) @ rotate
        old_xy /= geometry["resolution"]
        ax.plot(*old_xy.T, color="#2275bd", linewidth=.7, alpha=.65, label="Existing saved route (104 anchors)")
        old_count = len(route["anchor_path_indices"])
    ax.plot(*xy.T, color="#e23937", linewidth=.9, label="New complete scanner trace (not a planned route)")
    for point, text in ((xy[0], "Start"), (xy[-1], "End")):
        ax.scatter(*point, s=35, color="#e23937", edgecolor="white", zorder=5)
        ax.annotate(text, point, xytext=(6, 7), textcoords="offset points", fontsize=10)
    ax.set_xlim(0, cells.shape[1]); ax.set_ylim(0, cells.shape[0])
    ax.set_aspect("equal"); ax.set_xlabel("Map image column (pixels)"); ax.set_ylabel("Map image row from bottom (pixels)")
    ax.set_title("New scanner trace in the current navigation map frame")
    ax.legend(fontsize=8, loc="upper right")
    fig.savefig(output, dpi=180)
    plt.close(fig)
    inside = ((xy >= 0).all(1) & (xy[:, 0] < cells.shape[1]) & (xy[:, 1] < cells.shape[0]))
    return {"samples_inside_map": int(inside.sum()), "samples_outside_map": int((~inside).sum()),
            "previous_route_anchors": old_count, "width": cells.shape[1], "height": cells.shape[0]}


def export(args):
    base, output = args.map_base.resolve(), args.output_base.resolve()
    suffixes = ("_trajectory.csv", "_trajectory.pcd", "_overlay.png", "_analysis.json")
    if any(Path(str(output)+suffix).exists() for suffix in suffixes):
        raise ValueError("Refusing to overwrite an existing trajectory export")
    geofile = Path(str(base)+"_georeference.yaml")
    geo = yaml.safe_load(geofile.read_text())
    if geo["map"]["id"] != base.name:
        raise ValueError("Map georeference ID mismatch")
    protected = [geofile, base.with_suffix(".yaml"), base.with_suffix(".pgm")]
    saved = Path(str(base)+"_saved_paths.json")
    if saved.exists(): protected.append(saved)
    before = {str(p): sha256_file(p) for p in protected}
    with open_capture(args.capture) as capture:
        gnss, poses, qualities = read_capture(capture)
        rows = list(csv.DictReader(io.StringIO(capture.read(member(capture, "/project_data/gnss.csv")).decode("utf-8-sig"))))
        capture_hashes = {suffix: hashlib.sha256(capture.read(member(capture, "/"+suffix))).hexdigest()
                          for suffix in ("project_data/poses.csv", "project_data/gnss.csv")}
    positions, alignment = in_map_frame(gnss, poses, geo, args.source_crs)
    quality, deltas = nearest_quality(poses[:, 0], rows)
    output.parent.mkdir(parents=True, exist_ok=True)
    with Path(str(output)+"_trajectory.csv").open("x", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["capture_timestamp_s", "elapsed_s", "map_x_m", "map_y_m", "map_z_m", "gnss_quality", "nearest_gnss_delta_s"])
        for index, point in enumerate(positions):
            writer.writerow([f"{poses[index,0]:.6f}", f"{poses[index,0]-poses[0,0]:.6f}",
                             *[f"{v:.9f}" for v in point], int(quality[index]), f"{deltas[index]:.6f}"])
    write_pcd(Path(str(output)+"_trajectory.pcd"), positions)
    overlay = plot_overlay(base, positions, Path(str(output)+"_overlay.png"))
    report = {"source_directory": str(args.capture.resolve()), "map_base": str(base),
              "source_crs": args.source_crs, "capture_csv_sha256": capture_hashes,
              "sample_count": len(poses), "duration_sec": float(poses[-1,0]-poses[0,0]),
              "raw_xy_length_m": float(np.linalg.norm(np.diff(positions[:,:2], axis=0), axis=1).sum()),
              "max_pose_gap_sec": float(np.diff(poses[:,0]).max()), "gnss_quality_counts": qualities,
              "matched_pose_quality_counts": dict(Counter(map(int, quality))),
              "pose_gnss_consistency": alignment, "overlay": overlay,
              "reference": geo["reference"], "map_from_enu": geo["map_from_enu"],
              "existing_artifacts_sha256": before, "existing_artifacts_unchanged": True,
              "note": "Complete raw scanner path, not a drivable planned route or independently surveyed ground truth. No new waypoint choices were made."}
    assert before == {str(p): sha256_file(p) for p in protected}
    Path(str(output)+"_analysis.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("map_base", type=Path)
    parser.add_argument("output_base", type=Path)
    parser.add_argument("--source-crs", required=True)
    export(parser.parse_args())


if __name__ == "__main__":
    main()
