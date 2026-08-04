import importlib.util
from pathlib import Path


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


def test_nav2_projection_uses_optimized_trajectory_height(monkeypatch, tmp_path):
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
    )

    assert len(commands) == 1
    command, check = commands[0]
    assert check is True
    reference_index = command.index("--height-reference-pcd")
    assert command[reference_index + 1] == str(trajectory_pcd)
