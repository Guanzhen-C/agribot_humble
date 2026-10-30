#!/usr/bin/env python3
"""Make local-ENU navigation maps from a Lixel LAS capture ZIP or directory."""

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import zipfile

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree
import yaml


class DirectoryCapture:
    """Read extracted captures through the same read-only interface as a ZIP."""

    def __init__(self, root):
        self.root = Path(root).resolve()
        self._files = {}
        for path in self.root.rglob("*"):
            if path.is_file():
                relative = path.relative_to(self.root)
                if "__MACOSX" in relative.parts or path.name.startswith("._"):
                    continue
                resolved = path.resolve()
                if not resolved.is_relative_to(self.root):
                    raise ValueError(f"Capture file escapes source directory: {path}")
                self._files[f"capture/{relative.as_posix()}"] = resolved
        self._snapshot = {name: self._signature(path) for name, path in self._files.items()}

    @staticmethod
    def _signature(path):
        stat = path.stat()
        return stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns

    def namelist(self):
        return list(self._files)

    def open(self, name):
        return self._files[name].open("rb")

    def read(self, name):
        return self._files[name].read_bytes()

    def getinfo(self, name):
        return SimpleNamespace(file_size=self._files[name].stat().st_size)

    def verify_unchanged(self):
        if any(self._signature(path) != self._snapshot[name]
               for name, path in self._files.items()):
            raise ValueError("Source capture changed during processing")

    def __enter__(self):
        return self

    def __exit__(self, *unused):
        self.verify_unchanged()


def open_capture(path):
    return DirectoryCapture(path) if path.is_dir() else zipfile.ZipFile(path)


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def member(archive, suffix):
    matches = [n for n in archive.namelist()
               if not n.startswith("__MACOSX/") and n.endswith(suffix)]
    if len(matches) != 1:
        raise ValueError(f"Expected one {suffix}, found {len(matches)}")
    return matches[0]


def read_capture(archive):
    rows = list(csv.DictReader(io.StringIO(archive.read(
        member(archive, "/project_data/gnss.csv")).decode("utf-8-sig"))))
    fixed = [row for row in rows if row["status"] == "4"]
    if len(fixed) < 10:
        raise ValueError("Too few fixed GNSS positions to verify the map frame")
    gnss = np.array([[float(r[k]) for k in
                      ("#timestamp", "longitude", "latitude", "altitude")]
                     for r in fixed])
    poses = np.loadtxt(io.BytesIO(archive.read(
        member(archive, "/project_data/poses.csv"))), delimiter=",")
    if poses.ndim != 2 or poses.shape[1] != 8 or not np.isfinite(poses).all():
        raise ValueError("Invalid capture poses")
    if (np.diff(poses[:, 0]) <= 0).any() or (np.diff(gnss[:, 0]) <= 0).any():
        raise ValueError("Capture timestamps are not strictly increasing")
    return gnss, poses, dict(Counter(row["status"] for row in rows))


class EnuFrame:
    def __init__(self, source_crs, reference):
        from pyproj import CRS, Transformer

        self.reference = np.asarray(reference, dtype=float)  # longitude, latitude, height
        self.source_crs = CRS.from_user_input(source_crs)
        if not self.source_crs.is_projected:
            raise ValueError("The LAS source CRS must be explicitly projected")
        self.to_geodetic = Transformer.from_crs(self.source_crs, 4979, always_xy=True)
        self.to_projected = Transformer.from_crs(4979, self.source_crs, always_xy=True)
        self.to_ecef = Transformer.from_crs(4979, 4978, always_xy=True)
        self.origin_ecef = np.array(self.to_ecef.transform(*self.reference))
        lon, lat = np.deg2rad(self.reference[:2])
        self.rotation = np.array([
            [-np.sin(lon), np.cos(lon), 0],
            [-np.sin(lat)*np.cos(lon), -np.sin(lat)*np.sin(lon), np.cos(lat)],
            [np.cos(lat)*np.cos(lon), np.cos(lat)*np.sin(lon), np.sin(lat)],
        ])

    def geodetic(self, llh):
        xyz = np.column_stack(self.to_ecef.transform(*np.asarray(llh).T))
        return (xyz - self.origin_ecef) @ self.rotation.T

    def projected(self, points):
        llh = np.column_stack(self.to_geodetic.transform(*np.asarray(points).T))
        return self.geodetic(llh)


def fit_trajectory(poses, gnss, enu):
    times = poses[:, 0]
    right = np.searchsorted(gnss[:, 0], times)
    valid = (right > 0) & (right < len(gnss))
    right = np.clip(right, 1, len(gnss) - 1)
    valid &= gnss[right, 0] - gnss[right - 1, 0] <= 0.5
    source = poses[valid, 1:4]
    if len(source) < 10:
        raise ValueError("Not enough time-matched fixed GNSS / pose samples")
    target = np.column_stack([np.interp(times[valid], gnss[:, 0], enu[:, i])
                              for i in range(3)])
    source_center, target_center = source[:, :2].mean(0), target[:, :2].mean(0)
    u, _, vt = np.linalg.svd((source[:, :2] - source_center).T @
                             (target[:, :2] - target_center))
    if np.linalg.det(u @ vt) < 0:
        u[:, -1] *= -1
    rotation = u @ vt
    translation = target_center - source_center @ rotation
    all_positions = poses[:, 1:4].copy()
    all_positions[:, :2] = all_positions[:, :2] @ rotation + translation
    all_positions[:, 2] += np.median(target[:, 2] - source[:, 2])
    residual = all_positions[valid, :2] - target[:, :2]
    errors = np.linalg.norm(residual, axis=1)
    # This is a course consistency check, not validation of a dual-antenna heading sensor.
    a, b = all_positions[valid, :2][::10], target[:, :2][::10]
    da, db = np.diff(a, axis=0), np.diff(b, axis=0)
    moving = (np.linalg.norm(da, axis=1) >= 1.0) & (np.linalg.norm(db, axis=1) >= 1.0)
    angles = np.arctan2(da[moving, 1], da[moving, 0]) - np.arctan2(db[moving, 1], db[moving, 0])
    angles = np.arctan2(np.sin(angles), np.cos(angles))
    return all_positions, {
        "sample_count": len(source),
        "horizontal_rmse_m": float(np.sqrt(np.mean(errors**2))),
        "horizontal_p95_m": float(np.quantile(errors, 0.95)),
        "horizontal_max_m": float(errors.max()),
        "course_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(angles**2)))) if len(angles) else 180.0,
        "pose_to_enu_yaw_deg": float(np.rad2deg(np.arctan2(rotation[0, 1], rotation[0, 0]))),
        "note": "Same-timestamp pose/GNSS consistency; not independent survey accuracy. Scanner antenna lever arm is not recalibrated.",
    }


def voxel_keys(points, voxel, minimum, dimensions):
    indices = np.floor(points / voxel).astype(np.int64) - minimum
    return np.ravel_multi_index(indices.T, dimensions)


def merge_voxels(parts):
    keys = np.concatenate([item[0] for item in parts])
    points = np.concatenate([item[1] for item in parts])
    unique, first = np.unique(keys, return_index=True)
    return unique, points[first]


def read_voxel_las(archive, voxel):
    import laspy

    name = member(archive, "/map.las")
    entry = archive.getinfo(name)
    parts = []
    with archive.open(name) as stream, laspy.open(stream, closefd=False) as reader:
        header = reader.header
        if header.point_format.id != 3 or header.are_points_compressed:
            raise ValueError("This input path requires uncompressed LAS point format 3")
        if entry.file_size != header.offset_to_point_data + header.point_count * header.point_format.size:
            raise ValueError("LAS file size does not match its declared point count")
        offsets = header.offsets.copy()
        minimum = np.floor((header.mins - offsets) / voxel).astype(np.int64) - 1
        dimensions = np.ceil((header.maxs - offsets) / voxel).astype(np.int64) - minimum + 2
        metadata = {"point_count": int(header.point_count), "offsets": offsets.tolist(),
                    "bounds_min": header.mins.tolist(), "bounds_max": header.maxs.tolist(),
                    "las_size_validated": True}
        if isinstance(archive, zipfile.ZipFile):
            metadata["zip_member_crc32"] = f"{entry.CRC:08x}"
        read = 0
        for chunk in reader.chunk_iterator(1_000_000):
            relative = np.column_stack([chunk.X * header.scales[0],
                                        chunk.Y * header.scales[1],
                                        chunk.Z * header.scales[2]])
            keys = voxel_keys(relative, voxel, minimum, dimensions)
            keys, first = np.unique(keys, return_index=True)
            # Store actual surface samples, not voxel centers; PCD XYZ are converted after reduction.
            colors = np.column_stack([chunk.red[first], chunk.green[first], chunk.blue[first]]).astype(np.uint32)
            if colors.max(initial=0) > 255:
                colors //= 256
            packed = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]
            data = np.empty((len(first), 5), dtype=np.float64)
            data[:, :3] = relative[first]
            data[:, 3] = chunk.intensity[first]
            data[:, 4] = packed
            parts.append((keys, data))
            read += len(chunk)
            if len(parts) >= 8:
                parts = [merge_voxels(parts)]
                print(f"LAS {read}/{header.point_count}; retained voxels {len(parts[0][0])}", flush=True)
        if read != header.point_count or stream.read(1):
            raise ValueError("LAS payload length mismatch")
        # Only ZipExtFile has an archive CRC; plain files get a SHA-256 instead.
        metadata["las_crc_verified"] = isinstance(archive, zipfile.ZipFile)
    if isinstance(archive, DirectoryCapture):
        metadata["sha256"] = sha256_file(archive._files[name])
    _, points = merge_voxels(parts)
    points[:, :3] += offsets
    return points, metadata


def disk(radius):
    y, x = np.ogrid[-radius:radius+1, -radius:radius+1]
    return x*x + y*y <= radius*radius


def ground_model(points, cell=0.5):
    origin = np.floor(points[:, :2].min(0) / cell) * cell
    indices = np.floor((points[:, :2] - origin) / cell).astype(int)
    width, height = indices.max(0) + 1
    keys = indices[:, 1] * width + indices[:, 0]
    order = np.lexsort((points[:, 2], keys))
    unique, starts, counts = np.unique(keys[order], return_index=True, return_counts=True)
    heights = np.full(int(width * height), np.nan)
    heights[unique] = points[order[starts + np.floor((counts - 1)*0.1).astype(int)], 2]
    heights = heights.reshape(height, width)
    missing = ~np.isfinite(heights)
    nearest = ndimage.distance_transform_edt(missing, return_distances=False, return_indices=True)
    filled = heights[tuple(nearest)]
    # Opening suppresses objects above the lower local surface without flattening the whole site.
    terrain = ndimage.grey_opening(filled, size=(17, 17))
    terrain = ndimage.median_filter(terrain, size=3)
    return terrain[indices[:, 1], indices[:, 0]]


def occupancy(points, path, resolution=0.05):
    if not np.isfinite(points).all() or len(points) < 10:
        raise ValueError("Insufficient finite geometry")
    ground = ground_model(points)
    relative_height = points[:, 2] - ground
    origin = np.floor((points[:, :2].min(0) - 1.0) / resolution) * resolution
    width, height = (np.ceil((points[:, :2].max(0) + 1.0 - origin) / resolution).astype(int) + 1)
    if width * height > 70_000_000:
        raise ValueError("Map exceeds the 70-million-cell memory limit")
    indices = np.floor((points[:, :2] - origin) / resolution).astype(int)
    near_path, _ = cKDTree(path[::3, :2]).query(points[:, :2], workers=2)
    ground_points = (np.abs(relative_height) <= 0.12) & (near_path <= 20.0)
    obstacles = (relative_height >= 0.15) & (relative_height <= 2.0)
    free = np.zeros((height, width), dtype=bool)
    free[indices[ground_points, 1], indices[ground_points, 0]] = True
    free = ndimage.binary_dilation(free, structure=disk(max(1, int(math.ceil(0.10/resolution)))))
    free = ndimage.binary_closing(free, structure=disk(2))
    occupied = np.zeros_like(free)
    occupied[indices[obstacles, 1], indices[obstacles, 0]] = True
    occupied = ndimage.binary_dilation(occupied, structure=disk(1))
    cells = np.full(free.shape, 205, dtype=np.uint8)
    cells[free] = 254
    cells[occupied] = 0
    return cells, origin, {"free_cells": int((cells == 254).sum()),
                           "occupied_cells": int((cells == 0).sum()),
                           "unknown_cells": int((cells == 205).sum()),
                           "ground_cell_m": 0.5, "ground_threshold_m": 0.12,
                           "obstacle_height_band_m": [0.15, 2.0],
                           "free_ground_max_distance_from_capture_m": 20.0}


def write_pcd(path, xyz, intensity=None, rgb=None):
    fields = ["x", "y", "z", "intensity"]
    formats = ["<f4"] * 4
    if rgb is not None:
        fields.append("rgb")
        formats.append("<u4")
    records = np.empty(len(xyz), dtype=list(zip(fields, formats)))
    for index, axis in enumerate("xyz"):
        records[axis] = xyz[:, index]
    records["intensity"] = 0 if intensity is None else intensity
    if rgb is not None:
        records["rgb"] = rgb
    types = ["F"]*4 + (["U"] if rgb is not None else [])
    header = ("# .PCD v0.7\nVERSION 0.7\nFIELDS " + " ".join(fields)
              + "\nSIZE " + " ".join(["4"]*len(fields)) + "\nTYPE " + " ".join(types)
              + "\nCOUNT " + " ".join(["1"]*len(fields))
              + f"\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n")
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        records.tofile(stream)


def fnv1a(data):
    result = 14695981039346656037
    for value in data:
        result = ((result ^ value)*1099511628211) & 0xffffffffffffffff
    return f"{result:016x}"


def preview(base, points, rgb, cells, origin, resolution, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(16, 8), constrained_layout=True)
    stride = max(1, len(points)//400000)
    color = rgb[::stride].astype(np.uint32)
    colors = np.column_stack([(color >> 16) & 255, (color >> 8) & 255, color & 255])/255.0
    axes[0].scatter(points[::stride, 0], points[::stride, 1], c=colors, s=0.12, rasterized=True)
    axes[0].set_title("Local ENU localization point cloud (top view)")
    extent = [origin[0], origin[0]+cells.shape[1]*resolution,
              origin[1], origin[1]+cells.shape[0]*resolution]
    axes[1].imshow(cells, origin="lower", extent=extent, cmap="gray", vmin=0, vmax=255,
                   interpolation="nearest")
    axes[1].set_title("Nav2 map: black obstacle / white observed ground / gray unknown")
    for axis in axes:
        axis.plot(path[:, 0], path[:, 1], color="#cb2e35", linewidth=0.7, label="Capture path (not a driving route)")
        axis.set_aspect("equal")
        axis.set_xlabel("East (m)")
        axis.set_ylabel("North (m)")
        axis.legend(fontsize=8)
    fig.savefig(str(base) + "_preview.png", dpi=180)
    plt.close(fig)


def prepare(args):
    from PIL import Image

    args.map_base = args.map_base.expanduser().resolve()
    if any(args.map_base.parent.glob(args.map_base.name + ".*")) or any(args.map_base.parent.glob(args.map_base.name + "_*")):
        raise ValueError("Refusing to overwrite existing map artifacts")
    args.map_base.parent.mkdir(parents=True, exist_ok=True)
    source = args.archive.expanduser().resolve()
    source_key = "source_directory" if source.is_dir() else "source_archive"
    with open_capture(source) as archive:
        gnss, poses, quality = read_capture(archive)
        frame = EnuFrame(args.source_crs, gnss[0, 1:4])
        with archive.open(member(archive, "/map.las")) as stream:
            import laspy
            header = laspy.LasHeader.read_from(stream)
        projected_reference = np.array(frame.to_projected.transform(*frame.reference))
        origin_error = float(np.linalg.norm(header.offsets - projected_reference))
        if origin_error > 0.5:
            raise ValueError(f"LAS origin / first fixed GNSS mismatch {origin_error:.3f} m; verify source CRS")
        fixed_enu = frame.geodetic(gnss[:, 1:4])
        path, alignment = fit_trajectory(poses, gnss, fixed_enu)
        if alignment["horizontal_rmse_m"] > 0.20:
            raise ValueError(f"Pose/GNSS frame consistency exceeds 0.20 m: {alignment}")
        print(json.dumps({"source": args.archive.name, "crs": args.source_crs,
                          "origin_check_m": origin_error, "alignment": alignment}), flush=True)
        points, las_metadata = read_voxel_las(archive, args.voxel)
        capture_hashes = {
            suffix.lstrip("/"): hashlib.sha256(archive.read(member(archive, suffix))).hexdigest()
            for suffix in ("/project_data/gnss.csv", "/project_data/poses.csv")
        }
    points[:, :3] = frame.projected(points[:, :3])
    near, _ = cKDTree(path[::3, :2]).query(points[:, :2], workers=2)
    points = points[np.isfinite(points).all(axis=1) & (near <= 40.0)]
    neighbor_distances, _ = cKDTree(points[:, :3]).query(points[:, :3], k=3, distance_upper_bound=0.35, workers=2)
    points = points[np.isfinite(neighbor_distances[:, -1])]
    print(f"Localization map: {len(points)} points", flush=True)
    cells, grid_origin, grid_metadata = occupancy(points[:, :3], path, args.resolution)
    with tempfile.TemporaryDirectory(prefix=f".{args.map_base.name}-", dir=args.map_base.parent) as staging:
        base = Path(staging)/args.map_base.name
        write_pcd(base.with_suffix(".pcd"), points[:, :3], points[:, 3], points[:, 4])
        write_pcd(Path(str(base)+"_trajectory.pcd"), path)
        Image.fromarray(cells[::-1]).save(base.with_suffix(".pgm"))
        base.with_suffix(".yaml").write_text(yaml.safe_dump({
            "image": args.map_base.with_suffix(".pgm").name, "mode": "trinary",
            "resolution": args.resolution, "origin": [float(grid_origin[0]), float(grid_origin[1]), 0.0],
            "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196,
        }, sort_keys=False), encoding="utf-8")
        fingerprint = fnv1a(base.with_suffix(".pcd").read_bytes())
        calibration = {
            "horizontal_rmse_m": alignment["horizontal_rmse_m"],
            "yaw_rmse_deg": alignment["course_rmse_deg"], "yaw_validation_passed": False,
            "sample_count": alignment["sample_count"], "source_bag": "",
            source_key: str(source), "version": "lixel_las_utm_to_enu_v1",
            "metric_scope": alignment["note"],
            "hash": fnv1a(json.dumps(alignment, sort_keys=True).encode()),
        }
        georeference = {
            "schema_version": 1, "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "map": {"id": args.map_base.name, "pcd": args.map_base.with_suffix(".pcd").name,
                    "fingerprint_fnv1a64": fingerprint},
            "reference": {"latitude_deg": float(frame.reference[1]), "longitude_deg": float(frame.reference[0]),
                          "altitude_m": float(frame.reference[2])},
            "map_from_enu": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
            "calibration": calibration,
            "source_crs": frame.source_crs.to_string(),
            "source_origin_check_m": origin_error,
            "validation": "Prepared offline; real C16 relocalization and chassis motion are not yet validated.",
        }
        Path(str(base)+"_georeference.yaml").write_text(yaml.safe_dump(georeference, sort_keys=False), encoding="utf-8")
        report = {source_key: str(source), f"{source_key}_unchanged": True,
                  "source_las": las_metadata, "gnss_quality_counts": quality,
                  "capture_csv_sha256": capture_hashes,
                  "source_crs": frame.source_crs.to_string(), "origin_check_m": origin_error,
                  "pose_gnss_consistency": alignment, "map_voxel_m": args.voxel,
                  "pcd_points": len(points), "resolution_m": args.resolution,
                  "grid_size": [int(cells.shape[1]), int(cells.shape[0])], "grid": grid_metadata,
                  "frame": "local ENU at first fixed GNSS position; source terrain preserved",
                  "visual_index_created": False, "real_vehicle_validation_completed": False,
                  "raw_xbin_crc_checked": False}
        preview(base, points[:, :3], points[:, 4], cells, grid_origin, args.resolution, path)
        report["artifact_sha256"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(staging).iterdir()}
        Path(str(base)+"_preparation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        for artifact in Path(staging).iterdir():
            artifact.rename(args.map_base.parent/artifact.name)
    print(json.dumps(report, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path, help="Original capture ZIP or extracted capture directory")
    parser.add_argument("map_base", type=Path)
    parser.add_argument("--source-crs", required=True, help="Verified LAS projected CRS, e.g. EPSG:32650")
    parser.add_argument("--voxel", type=float, default=0.10)
    parser.add_argument("--resolution", type=float, default=0.05)
    args = parser.parse_args()
    if not 0.02 <= args.voxel <= 0.5 or not 0.02 <= args.resolution <= 0.2:
        parser.error("Voxel must be 0.02..0.5 m and resolution 0.02..0.2 m")
    prepare(args)


if __name__ == "__main__":
    main()
