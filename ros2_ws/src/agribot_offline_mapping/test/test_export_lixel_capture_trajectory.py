import csv
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from PIL import Image
import pytest
import yaml


@pytest.fixture
def exporter(monkeypatch):
    directory = Path(__file__).parents[1]/"scripts"
    monkeypatch.syspath_prepend(str(directory))
    spec = importlib.util.spec_from_file_location("export_trace", directory/"export_lixel_capture_trajectory.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_overlay_uses_target_reference_not_capture_start(exporter):
    pytest.importorskip("pyproj")
    llh = np.column_stack([115.908 + .00005*np.sin(np.arange(50)/8),
                           38.968 + .00005*np.cos(np.arange(50)/8), np.full(50, -4.)])
    source_frame = exporter.EnuFrame("EPSG:32650", llh[0])
    source = source_frame.geodetic(llh)
    times = np.arange(50)*.2
    poses = np.column_stack([times, source, np.tile([0, 0, 0, 1], (50, 1))])
    gnss = np.column_stack([times, llh])
    target_llh = [115.9078, 38.9678, -3.]
    geo = {"reference": dict(zip(("longitude_deg", "latitude_deg", "altitude_m"), target_llh)),
           "map_from_enu": {"xyz": [2., 3., 1.], "rpy": [0., 0., .3]}}
    positions, _ = exporter.in_map_frame(gnss, poses, geo, "EPSG:32650")
    target = exporter.EnuFrame("EPSG:32650", target_llh).geodetic(llh)
    rotation = exporter.Rotation.from_euler("xyz", [0., 0., .3]).as_matrix()
    np.testing.assert_allclose(positions, target @ rotation.T + [2., 3., 1.], atol=1e-4)
    assert np.linalg.norm(positions[0, :2]) > 10


def test_quality_does_not_fill_missing_samples(exporter):
    rows = [{"#timestamp": "1", "status": "4"}, {"#timestamp": "2", "status": "0"}]
    quality, dt = exporter.nearest_quality(np.array([1., 1.1, 1.6, 2.]), rows)
    assert quality.tolist() == [4, 4, -1, 0]
    assert dt[2] == pytest.approx(.4)


def test_overlay_preserves_map_image_and_counts_off_map_points(exporter, tmp_path):
    base = tmp_path/"map"
    Image.fromarray(np.full((40, 50), 254, dtype=np.uint8)).save(base.with_suffix(".pgm"))
    base.with_suffix(".yaml").write_text(yaml.safe_dump({"image": "map.pgm", "resolution": .1, "origin": [-2., -1., 0.]}))
    before = base.with_suffix(".pgm").read_bytes()
    report = exporter.plot_overlay(base, np.array([[0., 0., 0.], [10., 0., 0.]]), tmp_path/"overlay.png")
    assert report["samples_inside_map"] == report["samples_outside_map"] == 1
    assert before == base.with_suffix(".pgm").read_bytes()


def test_existing_trace_export_is_never_overwritten(exporter, tmp_path):
    output = tmp_path/"trace"
    Path(str(output)+"_trajectory.csv").write_text("original")
    with pytest.raises(ValueError, match="overwrite"):
        exporter.export(SimpleNamespace(map_base=tmp_path/"map", output_base=output))
    assert Path(str(output)+"_trajectory.csv").read_text() == "original"
