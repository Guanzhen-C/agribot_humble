"""Contact compliance changes must preserve the nominal vehicle and its traction."""
import ast
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


DESCRIPTION = Path(__file__).resolve().parents[2] / "agribot_vehicle_description"


@pytest.mark.parametrize("vehicle,front_friction,mass", [
    ("tracked_harvester", .07650682033604794, 14500.),
    ("harvester_header", .028494225543134154, 14500.),
])
def test_runtime_harvester_contact_has_finite_compliance_without_geometry_changes(
        vehicle, front_friction, mass):
    authoring = DESCRIPTION / "config/vehicles" / vehicle / "vehicle_config.json"
    generated = DESCRIPTION / "generated" / vehicle
    if not authoring.is_file() or not generated.is_dir():
        pytest.skip("Prepared local-only harvester bundles are unavailable")
    config = json.loads(authoring.read_text())
    assert config == json.loads((generated / "vehicle_config.json").read_text())
    contact = config["metadata"]["simulationSkidContact"]
    assert contact == dict(lateralFriction=.1, longitudinalFriction=1.8,
                           frontLateralFriction=front_friction, rearLateralFriction=.1,
                           lateralSlipMpsPerN=1e-6)
    assert config["geometry"]["wheelbaseM"] == pytest.approx(3.564)
    assert config["dynamics"]["massKg"] == mass
    assert config["dynamics"]["wheelTorqueNm"] == 20000.
    model = ET.parse(generated / "models" / (vehicle + ".sdf")).getroot()
    for wheel in config["geometry"]["wheelCenters"]:
        link = model.find(f"model/link[@name='{wheel['id']}_wheel_link']")
        ode = link.find("collision/surface/friction/ode")
        assert float(ode.findtext("slip1")) == 1e-6
        assert float(ode.findtext("mu")) == pytest.approx(
            front_friction if wheel["id"].startswith("front_") else .1, rel=0, abs=1e-12)
        assert float(ode.findtext("mu2")) == 1.8
        assert float(link.findtext("collision/geometry/cylinder/radius")) == wheel["radiusM"]
    manifest = json.loads((generated / "manifest.json").read_text())
    import hashlib
    for record in manifest["files"]:
        path = generated / record["path"]
        assert path.stat().st_size == record["sizeBytes"]
        assert hashlib.sha256(path.read_bytes()).hexdigest() == record["sha256"]


def test_future_harvester_preparation_uses_the_same_contact_compliance():
    source = Path(__file__).resolve().parents[1] / "scripts/prepare_harvester.py"
    values = [node.value.value for node in ast.walk(ast.parse(source.read_text()))
              if isinstance(node, ast.keyword) and node.arg == "lateralSlipMpsPerN"
              and isinstance(node.value, ast.Constant)]
    assert values == [1e-6]
