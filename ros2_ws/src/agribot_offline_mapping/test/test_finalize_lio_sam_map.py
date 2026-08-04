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
