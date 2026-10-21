"""Optional integration checks against prepared, local-only CAD derivatives."""

import json
import os
from pathlib import Path

import pytest

from agribot_vehicle_config_tools.model import load_and_validate
from agribot_vehicle_config_tools.implements import collision_boxes


def test_fullsize_host_and_optional_header_share_metric_geometry():
    root = os.environ.get("AGRIBOT_FARM_ASSET_ROOT")
    if not root:
        pytest.skip("Set AGRIBOT_FARM_ASSET_ROOT for the local CAD assets")
    root = Path(root)
    host, _ = load_and_validate(root / "tracked_harvester/ros_fullsize_v7/vehicle_config.json")
    combo, _ = load_and_validate(root / "mounted/harvester_header_v7/vehicle_config.json")
    assert host["implements"] == []
    assert host["metadata"]["nominalCadScale"] == 10.
    assert "缩比" not in host["displayName"]
    assert combo["sensors"] == host["sensors"]
    assert combo["geometry"] == host["geometry"]
    assert combo["visuals"][0]["scale"] == [1., 1., 1.]
    assert host["geometry"]["wheelbaseM"] == pytest.approx(3.564)
    assert host["dynamics"]["massKg"] == 14500.
    assert combo["metadata"]["simulationSkidContact"]["lateralSlipMpsPerN"] == .0001
    assert combo["metadata"]["simulationDiffDriveWheelAccelerationMps2"] == 0.
    assert host["geometry"]["bodyExtents"]["leftM"] < 2.5
    assert host["geometry"]["bodyExtents"]["rightM"] < 2.5
    box = collision_boxes(combo)[0]
    assert box[4] - box[1] == pytest.approx(12.726, abs=.01)
    asset = json.loads((root / "combine_header/normalized_v2/implement_asset.json").read_text())
    assert combo["implements"][0]["pose"]["xyz"] == asset["mountPoseXyz"]
    # Reused sensors remain physically sized, not 10x vehicle-size placeholders.
    for sensor in host["sensors"]:
        assert sensor["pose"]["xyz"][2] > 1.
    from agribot_farm_sim.compatibility import installation_review
    assert not installation_review(combo)
    previous, _ = load_and_validate(root / "tracked_harvester/ros_fullsize_v2/vehicle_config.json")
    assert host["geometry"] == previous["geometry"]
    assert host["dynamics"] == previous["dynamics"]
    old = {s["id"]: s for s in previous["sensors"]}
    for sensor in host["sensors"]:
        if sensor["id"] in ("lidar", "right_camera"):
            assert sensor["pose"]["xyz"][:2] == old[sensor["id"]]["pose"]["xyz"][:2]
            assert sensor["pose"]["xyz"][2] - old[sensor["id"]]["pose"]["xyz"][2] == pytest.approx(.6)
        else:
            assert sensor == old[sensor["id"]]
