import importlib.util
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "finalize_lio_sam_map.py"
SPEC = importlib.util.spec_from_file_location("finalize_lio_sam_map", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_copy_global_map_is_atomic(tmp_path):
    source_directory = tmp_path / "lio_sam"
    source_directory.mkdir()
    (source_directory / "GlobalMap.pcd").write_bytes(b"pcd-data")
    map_base = tmp_path / "maps" / "corridor"

    result = MODULE.copy_global_map(source_directory, map_base)

    assert result == map_base.with_suffix(".pcd")
    assert result.read_bytes() == b"pcd-data"
    assert not result.with_suffix(".pcd.tmp").exists()


def test_copy_global_map_requires_official_output(tmp_path):
    try:
        MODULE.copy_global_map(tmp_path, tmp_path / "map")
    except FileNotFoundError as error:
        assert "GlobalMap.pcd" in str(error)
    else:
        raise AssertionError("missing GlobalMap.pcd was accepted")


def test_nav2_projection_uses_optimized_trajectory_height(
    monkeypatch, tmp_path
):
    commands = []

    def capture(command, check):
        commands.append((command, check))

    monkeypatch.setattr(MODULE.subprocess, "run", capture)
    map_pcd = tmp_path / "map.pcd"
    trajectory_pcd = tmp_path / "trajectory.pcd"
    map_base = tmp_path / "map"

    MODULE.project_nav2_map(
        map_pcd,
        trajectory_pcd,
        map_base,
        resolution=0.05,
        minimum_z=0.0,
        maximum_z=0.6395,
        padding=1.0,
        dilation=0.05,
        trajectory_clearance_half_width=0.28,
    )

    assert len(commands) == 1
    command, check = commands[0]
    assert check is True
    reference_index = command.index("--height-reference-pcd")
    assert command[reference_index + 1] == str(trajectory_pcd)
    clearance_index = command.index("--clear-trajectory-pcd")
    assert command[clearance_index + 1] == str(trajectory_pcd)
    width_index = command.index("--clear-trajectory-half-width")
    assert command[width_index + 1] == "0.28"


def write_test_pcd(path, xyz):
    points = np.zeros(
        len(xyz),
        dtype=[
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("intensity", "<f4"),
        ],
    )
    points["x"], points["y"], points["z"] = np.asarray(xyz).T
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    ).encode("ascii")
    MODULE.write_binary_pcd(path, header, points)


def test_horizontal_leveling_is_rigid_and_levels_the_reference(tmp_path):
    source_directory = tmp_path / "lio_sam"
    source_directory.mkdir()
    xy = np.array(
        [
            (x, y)
            for x in np.linspace(-3.0, 3.0, 7)
            for y in np.linspace(-2.0, 2.0, 5)
        ]
    )
    trajectory = np.column_stack(
        (xy, 0.04 * xy[:, 0] - 0.03 * xy[:, 1] + 0.2)
    )
    map_points = np.vstack(
        (trajectory, trajectory + np.array([0.0, 0.0, 1.0]))
    )
    write_test_pcd(source_directory / "trajectory.pcd", trajectory)
    write_test_pcd(source_directory / "GlobalMap.pcd", map_points)

    destination, reference, metadata, source_tilt = MODULE.level_global_map(
        source_directory, tmp_path / "maps" / "level", 5.0
    )
    _, leveled_reference = MODULE.read_binary_pcd(reference)
    _, leveled_map = MODULE.read_binary_pcd(destination)
    _, _, leveled_tilt, _ = MODULE.fit_trajectory_plane(
        MODULE.xyz_array(leveled_reference)
    )

    assert source_tilt == pytest.approx(
        np.degrees(np.arctan(0.05)), abs=1.0e-4
    )
    assert leveled_tilt == pytest.approx(0.0, abs=1.0e-5)
    leveled_distance = np.linalg.norm(
        MODULE.xyz_array(leveled_map)[0] - MODULE.xyz_array(leveled_map)[-1]
    )
    original_distance = np.linalg.norm(map_points[0] - map_points[-1])
    assert leveled_distance == pytest.approx(original_distance, abs=1.0e-5)
    assert metadata.is_file()
