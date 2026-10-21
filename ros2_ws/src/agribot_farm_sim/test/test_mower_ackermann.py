"""The requested Car8 conversion must survive both export and runtime generation."""
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from agribot_vehicle_config_tools.generator import generate_sdf, generate_urdf
from agribot_vehicle_config_tools.model import ConfigError, validate

SRC = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("vehicle", ["ride_on_mower", "mower_front_cutter"])
def test_controlled_front_steering_conversion(vehicle):
    folder = SRC / "agribot_vehicle_description/generated" / vehicle
    config = json.loads((folder / "vehicle_config.json").read_text())
    validate(config)
    assert config["vehicleType"] == "ackermann"
    geometry = config["geometry"]
    assert geometry["minTurningRadiusM"] == pytest.approx(
        geometry["wheelbaseM"] / math.tan(geometry["maxSteeringAngleRad"]))
    sdf = ET.fromstring(generate_sdf(config)).find("model")
    urdf = ET.fromstring(generate_urdf(config))
    assert not sdf.findall("plugin[@filename='libgazebo_ros_diff_drive.so']")
    assert sdf.find("plugin[@name='ackermann_drive']") is not None
    for wheel in geometry["wheelCenters"]:
        key = wheel["id"]
        assert "caster" not in wheel
        assert wheel["driven"] == key.startswith("rear")
        assert wheel["steering"] == key.startswith("front")
        if not wheel["steering"]:
            continue
        assembly = wheel["steeringAssembly"]
        assert urdf.find(f"joint[@name='{key}_caster_joint']") is None
        assert sdf.find(f"joint[@name='{key}_caster_joint']") is None
        joint = sdf.find(f"joint[@name='{key}_steering_joint']")
        assert joint.get("type") == "revolute"
        assert 0 < float(joint.findtext("axis/limit/upper")) < 1
        assert list(map(float, joint.findtext("pose").split()[:3])) == pytest.approx(assembly["pivotPosition"])
        assert sdf.findtext(f"link[@name='{key}_steering_link']/visual/geometry/mesh/uri") == assembly["meshUri"]
        spin = urdf.find(f"joint[@name='{key}_wheel_joint']")
        assert list(map(float, spin.find("origin").get("xyz").split())) == pytest.approx(
            [a-b for a,b in zip(wheel["position"], assembly["pivotPosition"])])
    nav = yaml.safe_load((folder / "config/simulation/nav2_params_ackermann.yaml").read_text())
    controller = nav["controller_server"]["ros__parameters"]["FollowPath"]
    assert controller["motion_model"] == "Ackermann"
    assert controller["AckermannConstraints"]["min_turning_r"] == pytest.approx(geometry["minTurningRadiusM"])


def test_controlled_fork_cannot_silently_become_a_differential_caster():
    config = json.loads((SRC / "agribot_vehicle_description/generated/ride_on_mower/vehicle_config.json").read_text())
    config = copy.deepcopy(config)
    config["vehicleType"] = "differential"
    with pytest.raises(ConfigError):
        validate(config)
