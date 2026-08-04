#!/usr/bin/env python3

import argparse
import math
from pathlib import Path
import shutil
import subprocess

import numpy as np


PCD_TYPES = {
    ("F", 4): "<f4",
    ("F", 8): "<f8",
    ("I", 1): "<i1",
    ("I", 2): "<i2",
    ("I", 4): "<i4",
    ("I", 8): "<i8",
    ("U", 1): "<u1",
    ("U", 2): "<u2",
    ("U", 4): "<u4",
    ("U", 8): "<u8",
}


def read_binary_pcd(path: Path):
    metadata = {}
    header_lines = []
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"PCD header has no DATA entry: {path}")
            header_lines.append(line)
            decoded = line.decode("ascii").strip()
            if decoded and not decoded.startswith("#"):
                key, *values = decoded.split()
                metadata[key.upper()] = values
                if key.upper() == "DATA":
                    break

        if metadata.get("DATA", [""])[0].lower() != "binary":
            raise ValueError("horizontal leveling requires a binary PCD")
        fields = metadata.get("FIELDS", [])
        sizes = [int(value) for value in metadata.get("SIZE", [])]
        types = metadata.get("TYPE", [])
        counts = [
            int(value)
            for value in metadata.get("COUNT", ["1"] * len(fields))
        ]
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError("PCD field metadata lengths do not match")
        if not {"x", "y", "z"}.issubset(fields):
            raise ValueError("PCD must contain x, y and z fields")
        for axis in ("x", "y", "z"):
            if counts[fields.index(axis)] != 1:
                raise ValueError(f"PCD field {axis} must be scalar")

        formats = []
        for field_type, size, count in zip(types, sizes, counts):
            scalar_type = PCD_TYPES.get((field_type.upper(), size))
            if scalar_type is None:
                raise ValueError(
                    f"unsupported PCD scalar type {field_type}{size}"
                )
            formats.append(
                scalar_type if count == 1 else (scalar_type, (count,))
            )
        point_count = int(
            metadata.get("POINTS", metadata.get("WIDTH", ["0"]))[0]
        )
        if point_count < 1:
            raise ValueError("PCD contains no points")
        points = np.fromfile(
            stream, dtype=np.dtype({"names": fields, "formats": formats}),
            count=point_count
        ).copy()
        if points.size != point_count:
            raise ValueError("PCD binary payload is truncated")
    return b"".join(header_lines), points


def write_binary_pcd(path: Path, header: bytes, points) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as stream:
        stream.write(header)
        points.tofile(stream)


def xyz_array(points):
    return np.column_stack((points["x"], points["y"], points["z"])).astype(
        np.float64, copy=False
    )


def fit_trajectory_plane(points):
    finite = np.isfinite(points).all(axis=1)
    if np.count_nonzero(finite) < 10:
        raise ValueError("trajectory needs at least 10 finite points")

    inliers = finite.copy()
    coefficients = None
    for _ in range(4):
        selected = points[inliers]
        design = np.column_stack(
            (selected[:, 0], selected[:, 1], np.ones(selected.shape[0]))
        )
        if np.linalg.matrix_rank(design) < 3:
            raise ValueError("trajectory does not span a two-dimensional area")
        coefficients = np.linalg.lstsq(
            design, selected[:, 2], rcond=None
        )[0]
        residuals = points[:, 2] - (
            coefficients[0] * points[:, 0]
            + coefficients[1] * points[:, 1]
            + coefficients[2]
        )
        center = np.median(residuals[inliers])
        mad = np.median(np.abs(residuals[inliers] - center))
        threshold = max(0.05, 4.0 * 1.4826 * mad)
        refined = finite & (np.abs(residuals - center) <= threshold)
        if np.count_nonzero(refined) < 10 or np.array_equal(refined, inliers):
            break
        inliers = refined

    selected = points[inliers]
    design = np.column_stack(
        (selected[:, 0], selected[:, 1], np.ones(selected.shape[0]))
    )
    coefficients = np.linalg.lstsq(design, selected[:, 2], rcond=None)[0]
    normal = np.array(
        [-coefficients[0], -coefficients[1], 1.0], dtype=np.float64
    )
    normal /= np.linalg.norm(normal)
    tilt_degrees = math.degrees(math.acos(np.clip(normal[2], -1.0, 1.0)))
    return coefficients, normal, tilt_degrees, inliers


def leveling_rotation(normal):
    target = np.array([0.0, 0.0, 1.0])
    cross = np.cross(normal, target)
    cosine = float(np.dot(normal, target))
    sine = float(np.linalg.norm(cross))
    if sine < 1.0e-12:
        return np.eye(3)
    skew = np.array(
        [
            [0.0, -cross[2], cross[1]],
            [cross[2], 0.0, -cross[0]],
            [-cross[1], cross[0], 0.0],
        ]
    )
    return np.eye(3) + skew + skew @ skew * ((1.0 - cosine) / sine**2)


def transform_pcd_points(points, rotation, pivot):
    transformed = (xyz_array(points) - pivot) @ rotation.T + pivot
    result = points.copy()
    for column, axis in enumerate(("x", "y", "z")):
        result[axis] = transformed[:, column]
    return result


def write_leveling_metadata(
    path: Path, coefficients, rotation, pivot, tilt_degrees, inlier_count
):
    temporary = path.with_suffix(".yaml.tmp")
    rows = [value for row in rotation for value in row]
    temporary.write_text(
        "mode: horizontal_trajectory_plane\n"
        f"source_plane_z_from_xy: [{coefficients[0]:.12g}, "
        f"{coefficients[1]:.12g}, {coefficients[2]:.12g}]\n"
        f"source_tilt_degrees: {tilt_degrees:.12g}\n"
        f"trajectory_inliers: {inlier_count}\n"
        f"pivot_xyz: [{pivot[0]:.12g}, {pivot[1]:.12g}, "
        f"{pivot[2]:.12g}]\n"
        "source_map_to_leveled_map_rotation_row_major: ["
        + ", ".join(f"{value:.12g}" for value in rows)
        + "]\n",
        encoding="ascii",
    )
    temporary.replace(path)


def level_global_map(
    source_directory: Path, map_base: Path, maximum_tilt_degrees: float
):
    source = source_directory / "GlobalMap.pcd"
    trajectory_path = source_directory / "trajectory.pcd"
    if not source.is_file():
        raise FileNotFoundError(f"LIO-SAM GlobalMap.pcd not found: {source}")
    if not trajectory_path.is_file():
        raise FileNotFoundError(
            f"LIO-SAM trajectory.pcd not found: {trajectory_path}"
        )

    trajectory_header, trajectory = read_binary_pcd(trajectory_path)
    trajectory_xyz = xyz_array(trajectory)
    coefficients, normal, tilt_degrees, inliers = fit_trajectory_plane(
        trajectory_xyz
    )
    if tilt_degrees > maximum_tilt_degrees:
        raise ValueError(
            f"trajectory plane tilt {tilt_degrees:.3f} deg exceeds the "
            f"{maximum_tilt_degrees:.3f} deg safety limit"
        )
    finite_indices = np.flatnonzero(np.isfinite(trajectory_xyz).all(axis=1))
    pivot = trajectory_xyz[finite_indices[0]]
    rotation = leveling_rotation(normal)

    map_header, map_points = read_binary_pcd(source)
    destination = map_base.with_suffix(".pcd")
    temporary = destination.with_suffix(".pcd.tmp")
    transformed_map = transform_pcd_points(map_points, rotation, pivot)
    write_binary_pcd(
        temporary, map_header, transformed_map
    )
    temporary.replace(destination)

    reference_path = map_base.parent / f".{map_base.name}_level_reference.pcd"
    write_binary_pcd(
        reference_path,
        trajectory_header,
        transform_pcd_points(trajectory, rotation, pivot),
    )
    metadata_path = map_base.parent / f"{map_base.name}_leveling.yaml"
    write_leveling_metadata(
        metadata_path,
        coefficients,
        rotation,
        pivot,
        tilt_degrees,
        int(np.count_nonzero(inliers)),
    )
    return destination, reference_path, metadata_path, tilt_degrees


def copy_global_map(source_directory: Path, map_base: Path) -> Path:
    source = source_directory / "GlobalMap.pcd"
    if not source.is_file():
        raise FileNotFoundError(f"LIO-SAM GlobalMap.pcd not found: {source}")
    map_base.parent.mkdir(parents=True, exist_ok=True)
    destination = map_base.with_suffix(".pcd")
    temporary = destination.with_suffix(".pcd.tmp")
    shutil.copyfile(source, temporary)
    temporary.replace(destination)
    return destination


def project_nav2_map(
    pcd_path: Path,
    height_reference_pcd: Path,
    map_base: Path,
    resolution: float,
    minimum_z: float,
    maximum_z: float,
    padding: float,
    dilation: float,
) -> None:
    subprocess.run(
        [
            "ros2",
            "run",
            "agribot_hardware_bringup",
            "pcd_to_nav2_map.py",
            str(pcd_path),
            str(map_base),
            "--resolution",
            str(resolution),
            "--min-z",
            str(minimum_z),
            "--max-z",
            str(maximum_z),
            "--padding",
            str(padding),
            "--dilation",
            str(dilation),
            "--height-reference-pcd",
            str(height_reference_pcd),
        ],
        check=True,
    )


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Copy the optimized LIO-SAM PCD and generate its Nav2 map"
    )
    parser.add_argument("lio_sam_map_directory", type=Path)
    parser.add_argument("map_base", type=Path)
    parser.add_argument("--resolution", type=float, default=0.05)
    # The projection measures this band from the nearest optimized C16 pose.
    # It spans the optical-center plane to 1 m above the local ground.
    parser.add_argument("--min-z", type=float, default=0.0)
    parser.add_argument("--max-z", type=float, default=0.6395)
    parser.add_argument("--padding", type=float, default=1.0)
    parser.add_argument("--dilation", type=float, default=0.05)
    parser.add_argument(
        "--level-horizontal-trajectory",
        action="store_true",
        help=(
            "rigidly level the map from its trajectory plane; only use when "
            "the driven site is known to be horizontal"
        ),
    )
    parser.add_argument(
        "--maximum-leveling-angle-deg", type=float, default=5.0
    )
    parser.add_argument("--skip-projection", action="store_true")
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    if arguments.resolution <= 0.0 or arguments.max_z <= arguments.min_z:
        raise SystemExit(
            "resolution must be positive and max-z must exceed min-z"
        )
    if arguments.padding < 0.0 or arguments.dilation < 0.0:
        raise SystemExit("padding and dilation must be non-negative")
    if arguments.maximum_leveling_angle_deg <= 0.0:
        raise SystemExit("maximum-leveling-angle-deg must be positive")

    temporary_height_reference = None
    if arguments.level_horizontal_trajectory:
        (
            pcd_path,
            temporary_height_reference,
            metadata_path,
            tilt_degrees,
        ) = level_global_map(
            arguments.lio_sam_map_directory,
            arguments.map_base,
            arguments.maximum_leveling_angle_deg,
        )
        print(
            f"Rigidly leveled a {tilt_degrees:.3f} deg trajectory plane; "
            f"metadata: {metadata_path}"
        )
    else:
        pcd_path = copy_global_map(
            arguments.lio_sam_map_directory, arguments.map_base
        )

    try:
        if arguments.skip_projection:
            print(f"Finalized optimized map: {pcd_path}")
            return
        height_reference_pcd = temporary_height_reference or (
            arguments.lio_sam_map_directory / "trajectory.pcd"
        )
        if not height_reference_pcd.is_file():
            raise FileNotFoundError(
                "LIO-SAM trajectory.pcd not found: "
                f"{height_reference_pcd}"
            )
        project_nav2_map(
            pcd_path,
            height_reference_pcd,
            arguments.map_base,
            arguments.resolution,
            arguments.min_z,
            arguments.max_z,
            arguments.padding,
            arguments.dilation,
        )
        print(f"Finalized optimized map: {pcd_path}")
    finally:
        if temporary_height_reference is not None:
            temporary_height_reference.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
