import json
import os
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ("compact_chisel_plow", "mounted_disc_harrow", "groove_maker", "mounted_cultivator",
          "forest_chain_crusher", "inter_row_milling_cutter", "garlic_planter", "mulch_planter")


@pytest.mark.parametrize("asset", ASSETS)
def test_implement_profile_has_units_preserving_rotation_and_provenance(asset):
    profile = json.loads((ROOT / "config" / f"{asset}_cad.json").read_text())
    assert profile["id"] == asset
    assert profile["sourceArchive"].endswith(".zip")
    assert profile["sourceMember"].lower().endswith((".step", ".stp"))
    assert len(profile["sourceSha256"]) == 64
    int(profile["sourceSha256"], 16)
    assert profile["redistributionLicense"] == "unverified_user_supplied"
    rotation = profile["sourceToFluRotation"]
    for i in range(3):
        for j in range(3):
            assert sum(rotation[i][k] * rotation[j][k] for k in range(3)) == int(i == j)
    determinant = sum(rotation[0][i] * (
        rotation[1][(i + 1) % 3] * rotation[2][(i + 2) % 3] -
        rotation[1][(i + 2) % 3] * rotation[2][(i + 1) % 3]) for i in range(3))
    assert determinant == 1
    assert all(0 < low < high for low, high in profile["expectedSizeRangeM"])


@pytest.mark.parametrize("asset", ASSETS)
def test_local_implement_artifact_is_grounded_and_not_claimed_deployed(asset):
    root = os.environ.get("AGRIBOT_FARM_ASSET_ROOT")
    if not root:
        pytest.skip("Set AGRIBOT_FARM_ASSET_ROOT to inspect local CAD derivatives")
    folder = Path(root) / asset / ("ros_v2" if asset == "mounted_cultivator" else "ros_v1")
    report = json.loads((folder / "implement_asset.json").read_text())
    assert report["coordinateConvention"] == "ROS_FLU"
    assert report["units"] == "metres"
    assert report["originDefinition"] == "front_ground_center"
    assert report["mountingVerified"] is False
    assert report["simulationVerified"] is False
    assert report["visualFaces"] <= report["maxVisualFaces"]
    vertices = []
    faces = 0
    with (folder / "implement_visual.obj").open() as stream:
        for line in stream:
            if line.startswith("v "):
                vertices.append([float(value) for value in line.split()[1:4]])
            elif line.startswith("f "):
                faces += 1
    assert faces == report["visualFaces"]
    minimum = [min(v[axis] for v in vertices) for axis in range(3)]
    maximum = [max(v[axis] for v in vertices) for axis in range(3)]
    assert minimum == pytest.approx(report["minimumM"], abs=1e-5)
    assert maximum == pytest.approx(report["maximumM"], abs=1e-5)
    assert minimum[2] == pytest.approx(0, abs=1e-5)
    assert maximum[0] == pytest.approx(0, abs=1e-5)
    assert minimum[1] == pytest.approx(-maximum[1], abs=1e-5)
    assert (folder / "preview.png").is_file()
