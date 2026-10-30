import importlib.util
from pathlib import Path
from types import SimpleNamespace
import zipfile

import numpy as np
import pytest


PATH = Path(__file__).parents[1] / "scripts/prepare_lixel_navigation_map.py"
SPEC = importlib.util.spec_from_file_location("lixel_map", PATH)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def test_only_unambiguous_real_members_are_accepted(tmp_path):
    archive = tmp_path / "input.zip"
    with zipfile.ZipFile(archive, "w") as output:
        output.writestr("scan/map.las", b"test")
        output.writestr("__MACOSX/scan/map.las", b"sidecar")
    with zipfile.ZipFile(archive) as source:
        assert module.member(source, "/map.las") == "scan/map.las"
        with pytest.raises(ValueError, match="Expected one"):
            module.member(source, "/missing.csv")


def test_voxel_reduction_preserves_a_real_surface_sample():
    xyz = np.array([[0.01, 0, 0], [0.02, 0, 0], [0.21, 0, 0]])
    keys = module.voxel_keys(xyz, 0.1, np.array([-1, -1, -1]), (5, 5, 5))
    unique, data = module.merge_voxels([(keys[:2], xyz[:2]), (keys[2:], xyz[2:])])
    assert len(unique) == 2
    np.testing.assert_allclose(data[:, 0], [0.01, 0.21])


def test_enu_uses_double_precision_before_local_float_conversion():
    pytest.importorskip("pyproj")
    frame = module.EnuFrame("EPSG:32650", [115.9073, 38.9678, -3.9])
    projected = np.array(frame.to_projected.transform(*frame.reference))
    local = frame.projected(np.array([projected, projected + [0.02, 0.0, 0.0]]))
    assert np.linalg.norm(local[0]) < 1e-7
    assert 0.019 < np.linalg.norm(local[1] - local[0]) < 0.021


def test_projected_crs_is_required():
    pytest.importorskip("pyproj")
    with pytest.raises(ValueError, match="projected"):
        module.EnuFrame("EPSG:4326", [115, 38, 0])


def test_trajectory_fit_does_not_bridge_fixed_gnss_outages():
    t = np.arange(40) * 0.2
    source = np.column_stack([np.sin(t)*5, np.cos(t)*5, t*0.01])
    poses = np.column_stack([t, source, np.tile([0, 0, 0, 1], (40, 1))])
    gnss = np.column_stack([t, source])
    good = (t < 2) | (t > 4)
    output, stats = module.fit_trajectory(poses, gnss[good], source[good])
    assert stats["sample_count"] < len(poses) - 8
    assert stats["horizontal_rmse_m"] < 1e-10
    np.testing.assert_allclose(output, source, atol=1e-10)


def test_unknown_ground_is_not_implicitly_free_and_wall_wins():
    x, y = np.meshgrid(np.arange(0, 6, 0.1), np.arange(0, 6, 0.1))
    floor = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
    wall = np.column_stack([np.full(50, 3.0), np.linspace(0, 5, 50), np.ones(50)])
    points = np.vstack([floor, wall])
    cells, origin, counts = module.occupancy(points, np.array([[1., 1., 0], [5., 5., 0]]), 0.1)
    assert counts["free_cells"] > 100
    assert counts["occupied_cells"] > 10
    assert counts["unknown_cells"] > 10
    assert cells[0, 0] == 205
    ix, iy = np.floor((np.array([3.0, 2.0]) - origin)/0.1).astype(int)
    assert cells[iy, ix] == 0


def test_pcd_is_compatible_with_existing_xyz_intensity_reader(tmp_path):
    projection = Path(__file__).parents[2] / "agribot_hardware_bringup/scripts/pcd_to_nav2_map.py"
    spec = importlib.util.spec_from_file_location("projection", projection)
    reader = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(reader)
    xyz = np.array([[0.1, 0.2, 0.3], [4, 5, 6]])
    path = tmp_path / "test.pcd"
    module.write_pcd(path, xyz, np.array([5, 6]), np.array([0xff0000, 0x00ff00]))
    np.testing.assert_allclose(reader.read_pcd_xyz(path), xyz, atol=1e-7)


def test_fnv_matches_existing_cpp_algorithm():
    assert module.fnv1a(b"") == "cbf29ce484222325"
    assert module.fnv1a(b"hello") == "a430d84680aabd0b"


def test_existing_maps_are_not_overwritten(tmp_path):
    base = tmp_path / "map"
    base.with_suffix(".pcd").write_bytes(b"original")
    with pytest.raises(ValueError, match="overwrite"):
        module.prepare(SimpleNamespace(map_base=base, archive=tmp_path/"missing.zip"))
    assert base.with_suffix(".pcd").read_bytes() == b"original"


def test_zip_las_reader_checks_full_payload_and_keeps_rgb(tmp_path):
    laspy = pytest.importorskip("laspy")
    import io

    header = laspy.LasHeader(point_format=3, version="1.2")
    header.scales = np.array([0.001]*3)
    header.offsets = np.array([405000, 4313000, -3])
    las = laspy.LasData(header)
    las.x = np.array([405000.01, 405000.02, 405000.21])
    las.y = np.array([4313000.0]*3)
    las.z = np.array([-3.0]*3)
    las.intensity = [10, 11, 12]
    las.red = [255, 255, 255]
    las.green = [0, 0, 0]
    las.blue = [0, 0, 0]
    stream = io.BytesIO()
    las.write(stream)
    path = tmp_path/"test.zip"
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as output:
        output.writestr("scan/map.las", stream.getvalue())
    with zipfile.ZipFile(path) as archive:
        points, metadata = module.read_voxel_las(archive, 0.1)
    assert len(points) == 2
    assert metadata["las_crc_verified"]
    assert (points[:, 4] == 0xff0000).all()


def test_directory_capture_reads_without_modifying_source(tmp_path):
    scan = tmp_path / "scan"
    (scan / "project_data").mkdir(parents=True)
    (scan / "map.las").write_bytes(b"las payload")
    (scan / "project_data" / "poses.csv").write_bytes(b"pose payload")
    (scan / "._map.las").write_bytes(b"sidecar")
    with module.open_capture(scan) as capture:
        name = module.member(capture, "/map.las")
        assert capture.getinfo(name).file_size == 11
        with capture.open(name) as stream:
            assert stream.read() == b"las payload"
        assert capture.read(module.member(capture, "/project_data/poses.csv")) == b"pose payload"
        assert len(capture.namelist()) == 2
    assert (scan / "map.las").read_bytes() == b"las payload"


def test_directory_capture_detects_changed_source(tmp_path):
    path = tmp_path / "map.las"
    path.write_bytes(b"old")
    with pytest.raises(ValueError, match="changed during processing"):
        with module.open_capture(tmp_path):
            path.write_bytes(b"changed")


def test_directory_capture_rejects_external_symlinks(tmp_path):
    scan = tmp_path / "scan"
    scan.mkdir()
    outside = tmp_path / "outside.las"
    outside.write_bytes(b"outside")
    (scan / "map.las").symlink_to(outside)
    with pytest.raises(ValueError, match="escapes source"):
        module.open_capture(scan)


def test_directory_las_uses_same_voxels_as_zip_and_reports_real_checks(tmp_path):
    laspy = pytest.importorskip("laspy")
    header = laspy.LasHeader(point_format=3, version="1.2")
    header.scales = np.array([0.001] * 3)
    header.offsets = np.array([405000, 4313000, -3])
    las = laspy.LasData(header)
    las.x = [405000.01, 405000.02, 405000.21]
    las.y = [4313000.0] * 3
    las.z = [-3.0] * 3
    las.intensity = [10, 11, 12]
    las.red = [65535] * 3
    las.green = [0] * 3
    las.blue = [0] * 3
    scan = tmp_path / "scan"
    scan.mkdir()
    path = scan / "map.las"
    las.write(path)
    archive_path = tmp_path / "scan.zip"
    with zipfile.ZipFile(archive_path, "w") as output:
        output.write(path, "scan/map.las")
    with module.open_capture(scan) as capture:
        points, metadata = module.read_voxel_las(capture, 0.1)
    with module.open_capture(archive_path) as capture:
        zipped, zip_metadata = module.read_voxel_las(capture, 0.1)
    np.testing.assert_array_equal(points, zipped)
    assert metadata["las_size_validated"]
    assert not metadata["las_crc_verified"]
    assert "zip_member_crc32" not in metadata
    assert metadata["sha256"] == module.sha256_file(path)
    assert zip_metadata["las_crc_verified"]
