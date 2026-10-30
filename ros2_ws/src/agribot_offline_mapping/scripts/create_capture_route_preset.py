#!/usr/bin/env python3
"""Transform a Lixel capture path into map-bound editable mobile route presets."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from prepare_lixel_navigation_map import EnuFrame, fit_trajectory, open_capture, read_capture


def simplify_indices(points, tolerance):
    """Ordered RDP using finite segments; preserves hairpins and repeated passes."""
    if not math.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("Simplification tolerance must be positive")
    if len(points) < 2 or not np.isfinite(points).all():
        raise ValueError("At least two finite positions are required")
    keep = {0, len(points) - 1}
    pending = [(0, len(points) - 1)]
    while pending:
        first, last = pending.pop()
        if last <= first + 1:
            continue
        segment = points[last] - points[first]
        delta = points[first:last + 1] - points[first]
        length2 = segment @ segment
        fractions = np.clip(delta @ segment / length2, 0, 1) if length2 > 0 else np.zeros(len(delta))
        distance = np.linalg.norm(delta - fractions[:, None] * segment, axis=1)
        split = int(distance.argmax())
        if distance[split] > tolerance:
            middle = first + split
            keep.add(middle)
            pending.extend([(first, middle), (middle, last)])
    return sorted(keep)


def to_map(points, source_frame, target_reference, transform):
    target = EnuFrame("EPSG:32650", [target_reference["longitude_deg"], target_reference["latitude_deg"], target_reference["altitude_m"]])
    ecef = points @ source_frame.rotation + source_frame.origin_ecef
    target_enu = (ecef - target.origin_ecef) @ target.rotation.T
    rotation = Rotation.from_euler("xyz", transform["rpy"]).as_matrix()
    return target_enu @ rotation.T + np.asarray(transform["xyz"], dtype=float)


def create(args):
    base = args.map_base.expanduser().resolve()
    output = Path(f"{base}_routes.json")
    preview_path = Path(f"{base}_routes_preview.png")
    csv_path = Path(f"{base}_capture_route.csv")
    if any(path.exists() for path in (output, preview_path, csv_path)):
        raise ValueError("Refusing to overwrite an existing preset or analysis artifact")
    georeference_path = Path(f"{base}_georeference.yaml")
    georeference_bytes = georeference_path.read_bytes()
    georeference = yaml.safe_load(georeference_bytes)
    geometry = yaml.safe_load(base.with_suffix(".yaml").read_text(encoding="utf-8"))
    if georeference["map"]["id"] != base.name:
        raise ValueError("Target map/georeference identity mismatch")
    with open_capture(args.capture.expanduser().resolve()) as capture:
        gnss, poses, quality = read_capture(capture)
    source_frame = EnuFrame("EPSG:32650", gnss[0, 1:4])
    source_path, alignment = fit_trajectory(poses, gnss, source_frame.geodetic(gnss[:, 1:4]))
    if alignment["horizontal_rmse_m"] > 0.20:
        raise ValueError("Source pose / fixed-GNSS consistency exceeds 0.20 m")
    path = to_map(source_path, source_frame, georeference["reference"], georeference["map_from_enu"])
    selected = simplify_indices(path[:, :2], args.tolerance)
    if len(selected) > 100:
        raise ValueError(f"{len(selected)} keypoints exceed the 100-pose gateway limit; split the task or explicitly adjust tolerance")
    keypoints = path[selected, :2]
    directions = np.diff(keypoints, axis=0)
    if (np.linalg.norm(directions, axis=1) < 0.01).any():
        raise ValueError("Route contains duplicate keypoints")
    yaw = np.arctan2(directions[:, 1], directions[:, 0])
    route = [{"x": float(point[0]), "y": float(point[1]), "yaw": float(yaw[min(i, len(yaw) - 1)])} for i, point in enumerate(keypoints)]
    document = {
        "schema_version": 1, "map_id": base.name, "frame_id": "map",
        "map_binding": {"georeference_sha256": hashlib.sha256(georeference_bytes).hexdigest(), "resolution": geometry["resolution"], "origin": geometry["origin"]},
        "routes": [{
            "id": args.route_id, "label": args.label, "vehicle_type": "differential",
            "source_capture": str(args.capture.expanduser().resolve()),
            "source_pose_samples": len(poses), "source_duration_s": float(poses[-1, 0] - poses[0, 0]),
            "source_raw_xy_step_sum_m": float(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum()),
            "simplified_length_m": float(np.linalg.norm(directions, axis=1).sum()),
            "simplification_tolerance_m": args.tolerance, "keypoint_source_indices": selected,
            "gnss_quality_counts": quality, "source_pose_gnss_consistency": alignment,
            "warning": "扫描人员轨迹未做整车通行验收；含RTK非固定解区段。请核对地图、障碍和路线后执行。",
            "poses": route,
        }],
    }
    # A preview is not a cleared corridor: never edit the navigation PGM here.
    from PIL import Image
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    image = Path(geometry["image"])
    if not image.is_absolute():
        image = base.parent / image
    with Image.open(image) as source_image:
        pixels = np.asarray(source_image.convert("L"))
    height, width = pixels.shape
    origin_x, origin_y, origin_yaw = geometry["origin"]
    resolution = geometry["resolution"]
    from matplotlib.transforms import Affine2D
    fig, ax = plt.subplots(figsize=(9, 12), layout="constrained")
    ax.imshow(pixels, origin="upper", extent=(0, width * resolution, 0, height * resolution), cmap="gray", vmin=0, vmax=255,
              transform=Affine2D().rotate(origin_yaw).translate(origin_x, origin_y) + ax.transData)
    ax.plot(path[:, 0], path[:, 1], color="#296cb0", linewidth=0.7, label="Capture path in target map")
    ax.plot(keypoints[:, 0], keypoints[:, 1], "o-", color="#e57221", linewidth=0.8, markersize=3, label=f"{len(route)} editable waypoints")
    for i, point in enumerate(keypoints):
        ax.annotate(str(i + 1), point, fontsize=6, xytext=(3, 3), textcoords="offset points")
    ax.set(xlabel="Map X (m)", ylabel="Map Y (m)", title="Map-aligned scanner route preset; vehicle clearance not verified")
    ax.set_aspect("equal")
    corners = np.array([[0, 0], [width * resolution, 0], [0, height * resolution], [width * resolution, height * resolution]])
    cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)
    corners = corners @ np.array([[cosine, sine], [-sine, cosine]]) + [origin_x, origin_y]
    ax.set_xlim(corners[:, 0].min(), corners[:, 0].max())
    ax.set_ylim(corners[:, 1].min(), corners[:, 1].max())
    ax.legend(fontsize=8)
    fig.savefig(preview_path, dpi=150)
    plt.close(fig)
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["capture_timestamp_s", "elapsed_s", "map_x_m", "map_y_m", "map_z_m", "waypoint_number"])
        indices = {source: i + 1 for i, source in enumerate(selected)}
        for i, position in enumerate(path):
            writer.writerow([f"{poses[i, 0]:.6f}", f"{poses[i, 0] - poses[0, 0]:.6f}", *[f"{v:.6f}" for v in position], indices.get(i, "")])
    output.write_text(json.dumps(document, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    print(json.dumps({"output": str(output), "waypoints": len(route), "source_poses": len(poses), "start": route[0], "end": route[-1]}, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("map_base", type=Path)
    parser.add_argument("--route-id", default="scanner_20261029_143812")
    parser.add_argument("--label", default="10月29日下午扫描完整路线")
    parser.add_argument("--tolerance", type=float, default=0.25)
    create(parser.parse_args())


if __name__ == "__main__":
    main()
