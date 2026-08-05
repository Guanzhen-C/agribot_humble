import importlib.util
import struct
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).parents[1]
SCRIPT_PATH = PACKAGE_ROOT / "scripts" / "pcd_to_nav2_map.py"
SPEC = importlib.util.spec_from_file_location("pcd_to_nav2_map", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def write_binary_pcd(path, points):
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    )
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        for point in points:
            stream.write(struct.pack("<ffff", *point))


def test_binary_pcd_height_projection_and_nav2_output(tmp_path):
    pcd_path = tmp_path / "map.pcd"
    output_base = tmp_path / "projected"
    write_binary_pcd(
        pcd_path,
        [
            (0.0, 0.0, 0.05, 1.0),
            (0.0, 0.0, 0.20, 1.0),
            (1.0, 1.0, 1.20, 1.0),
            (2.0, 2.0, 1.21, 1.0),
        ],
    )

    points = MODULE.read_pcd_xyz(pcd_path)
    cells, origin, selected_count = MODULE.project_occupancy(
        points, resolution=0.5, min_z=0.10, max_z=1.20, padding=0.0,
        dilation=0.0
    )
    pgm_path, yaml_path = MODULE.write_nav2_map(
        cells, origin, output_base, resolution=0.5
    )

    assert selected_count == 2
    assert cells.shape == (3, 3)
    assert np.count_nonzero(cells == 0) == 2
    assert np.allclose(origin, [0.0, 0.0])
    assert pgm_path.read_bytes().startswith(b"P5\n3 3\n255\n")
    assert "image: projected.pgm" in yaml_path.read_text()
    assert "resolution: 0.5" in yaml_path.read_text()


def test_relative_height_projection_follows_optimized_trajectory():
    points = np.array(
        [
            (0.0, 0.0, 0.30),
            (10.0, 0.0, 2.30),
            (10.0, 1.0, 2.00),
            (0.0, 1.0, 1.30),
        ]
    )
    trajectory = np.array(
        [
            (0.0, 0.0, 0.0),
            (10.0, 0.0, 2.0),
        ]
    )

    cells, _, selected_count = MODULE.project_occupancy(
        points,
        resolution=0.5,
        min_z=0.10,
        max_z=0.50,
        padding=0.0,
        dilation=0.0,
        height_reference_points=trajectory,
    )

    assert selected_count == 2
    assert np.count_nonzero(cells == 0) == 2


def test_height_reference_requires_finite_trajectory_points():
    with np.testing.assert_raises_regex(
        ValueError, "height-reference PCD contains no finite points"
    ):
        MODULE.nearest_reference_heights(
            np.array([[0.0, 0.0]]), np.array([[np.nan, 0.0, 0.0]])
        )


def test_trajectory_corridor_clears_only_driven_free_space():
    cells = np.zeros((21, 21), dtype=np.uint8)
    trajectory = np.array([[0.2, 0.5, 0.0], [0.8, 0.5, 0.0]])

    cleared = MODULE.clear_trajectory_corridor(
        cells,
        origin=np.array([0.0, 0.0]),
        resolution=0.05,
        trajectory_points=trajectory,
        half_width=0.10,
    )

    assert cleared > 0
    assert cells[10, 10] == 254
    assert cells[2, 10] == 0
    assert cells[18, 10] == 0


def test_trajectory_corridor_rejects_nonfinite_path():
    with np.testing.assert_raises_regex(
        ValueError, "clear-trajectory PCD contains no finite points"
    ):
        MODULE.clear_trajectory_corridor(
            np.zeros((3, 3), dtype=np.uint8),
            origin=np.array([0.0, 0.0]),
            resolution=0.05,
            trajectory_points=np.array([[np.nan, 0.0, 0.0]]),
            half_width=0.10,
        )
