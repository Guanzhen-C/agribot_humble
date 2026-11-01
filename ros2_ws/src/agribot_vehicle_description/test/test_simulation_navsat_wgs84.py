import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from pyproj import Geod, Transformer


def module():
    path = Path(__file__).resolve().parents[2]/"agribot_ackermann_mppi/scripts/sim_navsat_bridge.py"
    spec = importlib.util.spec_from_file_location("simulation_navsat_wgs84", path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


@pytest.mark.parametrize("reference", [(30.5, 114., 20.), (38.968032404, 115.90811917, -4.4333)])
@pytest.mark.parametrize("enu", [(0., 0., 0.), (0., 1000., 0.), (800., -600., 2.), (-1000., 500., -1.)])
def test_gnss_frame_round_trip_without_spherical_scale_error(reference, enu):
    frame = module().Wgs84Enu(*reference)
    lat, lon, alt = frame.geodetic(*enu)
    # PROJ's independent topocentric pipeline is the acceptance reference.
    pipeline = Transformer.from_pipeline(
        f"+proj=pipeline +step +proj=cart +ellps=WGS84 +step +proj=topocentric +ellps=WGS84 "
        f"+lat_0={reference[0]} +lon_0={reference[1]} +h_0={reference[2]}")
    restored = pipeline.transform(lon, lat, alt)
    np.testing.assert_allclose(restored, enu, atol=1e-6)


def test_one_kilometer_north_is_not_several_meters_short():
    frame = module().Wgs84Enu(30.5, 114., 20.)
    lat, lon, unused = frame.geodetic(0, 1000, 0)
    unused1, unused2, distance = Geod(ellps="WGS84").inv(114., 30.5, lon, lat)
    assert distance == pytest.approx(1000, abs=.01)


def test_dedicated_physics_truth_only_changes_the_observation_topic(tmp_path, monkeypatch):
    package = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location("configured_physical_truth", package/"launch/configured_ackermann_sim.launch.py")
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    monkeypatch.setattr(launch.tempfile, "gettempdir", lambda: str(tmp_path))
    source = package/"generated/unity_car3/models/unity_car3.sdf"
    original = source.read_bytes()
    standard = ET.parse(launch._write_gazebo_vehicle_model(source))
    observed = ET.parse(launch._write_gazebo_vehicle_model(source, ground_truth_topic="/validation/physics_truth"))
    query = "model/plugin[@name='configured_ground_truth']/ros/remapping"
    assert standard.findtext(query) == "odom:=base_pose_ground_truth"
    assert observed.findtext(query) == "odom:=/validation/physics_truth"
    observed.find(query).text = standard.findtext(query)
    assert ET.tostring(observed.getroot()) == ET.tostring(standard.getroot())
    assert source.read_bytes() == original
