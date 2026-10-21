import json
import math
import os
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("name,solids", [("compact_tractor", 63), ("cab_tractor", 216)])
def test_inspected_tractor_profile(name, solids):
    profile = json.loads((ROOT / f"config/{name}_cad.json").read_text())
    rotation = profile["sourceToFluRotation"]
    for i in range(3):
        for j in range(3):
            assert sum(rotation[i][k] * rotation[j][k] for k in range(3)) == (i == j)
    determinant = sum(rotation[0][i] * (rotation[1][(i + 1) % 3] * rotation[2][(i + 2) % 3]
                      - rotation[1][(i + 2) % 3] * rotation[2][(i + 1) % 3]) for i in range(3))
    assert determinant == 1
    groups = profile["wheelGroups"]
    assert set(groups) == {"FLwheel", "FRwheel", "RLwheel", "RRwheel"}
    indices = [index for group in groups.values() for index in group]
    assert len(indices) == len(set(indices))
    assert all(0 <= index < solids for index in indices)
    assert profile["redistributionLicense"] == "unverified_user_supplied"


def test_prepared_tractor_artifact():
    root = os.environ.get("AGRIBOT_TRACTOR_EXPORT")
    if not root:
        pytest.skip("Set AGRIBOT_TRACTOR_EXPORT to verify a locally licensed CAD conversion")
    folder = Path(root)
    config = json.loads((folder / "vehicle_config.json").read_text())
    report = json.loads((folder / "cad_import.json").read_text())
    assert config["metadata"]["calibrationState"] == "unity_nominal_simulation_only"
    assert report["simulationOnly"] is True
    assert report["visualFaces"] <= 100000
    geometry = config["geometry"]
    assert 1.7 < geometry["wheelbaseM"] < 1.9
    assert 1.3 < geometry["rearTrackM"] < 1.8
    assert 0.55 < geometry["wheelRadiusM"] < 0.65
    contacts = [wheel["position"][2] - wheel["radiusM"] for wheel in geometry["wheelCenters"]]
    assert max(contacts) - min(contacts) < 1e-6
    assert math.isclose(geometry["minTurningRadiusM"],
                        geometry["wheelbaseM"] / math.tan(geometry["maxSteeringAngleRad"]))
    names = {line[2:].strip() for line in (folder / "vehicle_visual.obj").open() if line.startswith("o ")}
    assert {"FLwheel", "FRwheel", "RLwheel", "RRwheel"} <= names
    assert not config["manipulators"]
    assert not config["implements"]
