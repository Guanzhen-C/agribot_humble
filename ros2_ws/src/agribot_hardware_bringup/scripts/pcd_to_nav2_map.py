#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

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


def read_pcd_xyz(path):
    header = {}
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD header has no DATA entry")
            decoded = line.decode("ascii").strip()
            if not decoded or decoded.startswith("#"):
                continue
            key, *values = decoded.split()
            header[key.upper()] = values
            if key.upper() == "DATA":
                break

        fields = header.get("FIELDS", [])
        sizes = [int(value) for value in header.get("SIZE", [])]
        types = header.get("TYPE", [])
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError("PCD field metadata lengths do not match")
        if not {"x", "y", "z"}.issubset(fields):
            raise ValueError("PCD must contain x, y and z fields")

        point_count = int(header.get("POINTS", header.get("WIDTH", ["0"]))[0])
        if point_count < 1:
            raise ValueError("PCD contains no points")

        formats = []
        for field_type, size, count in zip(types, sizes, counts):
            scalar_type = PCD_TYPES.get((field_type.upper(), size))
            if scalar_type is None:
                raise ValueError(f"unsupported PCD scalar type {field_type}{size}")
            formats.append(scalar_type if count == 1 else (scalar_type, (count,)))
        dtype = np.dtype({"names": fields, "formats": formats})

        data_mode = header["DATA"][0].lower()
        if data_mode == "binary":
            points = np.fromfile(stream, dtype=dtype, count=point_count)
            if points.size != point_count:
                raise ValueError("PCD binary payload is truncated")
            return np.column_stack((points["x"], points["y"], points["z"]))
        if data_mode == "ascii":
            values = np.loadtxt(stream, max_rows=point_count, ndmin=2)
            offsets = np.cumsum([0] + counts[:-1]).tolist()
            indices = [offsets[fields.index(axis)] for axis in ("x", "y", "z")]
            return values[:, indices]
        raise ValueError(f"unsupported PCD DATA mode '{data_mode}'")


def project_occupancy(points, resolution, min_z, max_z, padding, dilation):
    finite = np.isfinite(points).all(axis=1)
    selected = points[
        finite & (points[:, 2] >= min_z) & (points[:, 2] <= max_z)
    ]
    if selected.size == 0:
        raise ValueError("no PCD points remain inside the requested height band")

    minimum = selected[:, :2].min(axis=0)
    maximum = selected[:, :2].max(axis=0)
    origin = np.floor((minimum - padding) / resolution) * resolution
    dimensions = np.ceil((maximum + padding - origin) / resolution).astype(int) + 1
    width, height = int(dimensions[0]), int(dimensions[1])
    if width < 1 or height < 1 or width * height > 100_000_000:
        raise ValueError("generated occupancy map has invalid dimensions")

    cells = np.full((height, width), 254, dtype=np.uint8)
    indices = np.floor((selected[:, :2] - origin) / resolution).astype(int)
    dilation_cells = int(math.ceil(dilation / resolution))
    for offset_y in range(-dilation_cells, dilation_cells + 1):
        for offset_x in range(-dilation_cells, dilation_cells + 1):
            if offset_x * offset_x + offset_y * offset_y > dilation_cells**2:
                continue
            x = indices[:, 0] + offset_x
            y = indices[:, 1] + offset_y
            valid = (x >= 0) & (x < width) & (y >= 0) & (y < height)
            cells[y[valid], x[valid]] = 0
    return cells, origin, selected.shape[0]


def write_nav2_map(cells, origin, output_base, resolution):
    output_base.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = output_base.with_suffix(".pgm")
    yaml_path = output_base.with_suffix(".yaml")
    pgm_temp = pgm_path.with_suffix(".pgm.tmp")
    yaml_temp = yaml_path.with_suffix(".yaml.tmp")

    height, width = cells.shape
    with pgm_temp.open("wb") as stream:
        stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        stream.write(cells[::-1].tobytes())
    yaml_temp.write_text(
        f"image: {pgm_path.name}\n"
        "mode: trinary\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin[0]}, {origin[1]}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n",
        encoding="ascii",
    )
    pgm_temp.replace(pgm_path)
    yaml_temp.replace(yaml_path)
    return pgm_path, yaml_path


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Project an Agribot 3D PCD map into a Nav2 occupancy map"
    )
    parser.add_argument("pcd", type=Path)
    parser.add_argument("output_base", type=Path)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--min-z", type=float, default=0.233)
    parser.add_argument("--max-z", type=float, default=0.8725)
    parser.add_argument("--padding", type=float, default=1.0)
    parser.add_argument("--dilation", type=float, default=0.05)
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    if arguments.resolution <= 0.0 or arguments.max_z <= arguments.min_z:
        raise SystemExit("resolution must be positive and max-z must exceed min-z")
    if arguments.padding < 0.0 or arguments.dilation < 0.0:
        raise SystemExit("padding and dilation must be non-negative")

    points = read_pcd_xyz(arguments.pcd)
    cells, origin, selected_count = project_occupancy(
        points,
        arguments.resolution,
        arguments.min_z,
        arguments.max_z,
        arguments.padding,
        arguments.dilation,
    )
    pgm_path, yaml_path = write_nav2_map(
        cells, origin, arguments.output_base, arguments.resolution
    )
    occupied_count = int(np.count_nonzero(cells == 0))
    print(
        f"Projected {selected_count} points into {cells.shape[1]}x{cells.shape[0]} "
        f"map with {occupied_count} occupied cells: {pgm_path}, {yaml_path}"
    )


if __name__ == "__main__":
    main()
