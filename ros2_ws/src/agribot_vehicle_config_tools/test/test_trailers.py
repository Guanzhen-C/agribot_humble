import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import jsonschema
import pytest

from agribot_vehicle_config_tools.generator import generate_sdf, generate_urdf
from agribot_vehicle_config_tools.model import ConfigError, load_and_validate, validate
from agribot_vehicle_config_tools.trailers import collision_boxes


ROOT = Path(__file__).parents[1]


def configuration():
    config, _ = load_and_validate(ROOT.parent/"agribot_vehicle_description/config/vehicles/ackermann_current/vehicle_config.json")
    config["implements"] = [dict(id="farm_trailer", implementType="farm_trailer", parentFrameId="base_link",
        frameId="farm_trailer_link", workFrameId="farm_trailer_work_frame", mountMode="articulated_transport",
        meshFile="implement_farm_trailer_visual.obj", simulationOnly=True, massKg=450., workingWidthM=2.9,
        pose=dict(xyz=[-1., 0., .2], rpy=[0., 0., 0.], quaternion=[0., 0., 0., 1.]),
        workOffset=[-3., 0., 0.], yawLimitRad=.6, pitchLimitRad=.15,
        collisionBoxes=[dict(name="body", center=[-3., 0., 1.], size=[5., 2.8, 2.])],
        wheels=[dict(id=axle+"_"+side, position=[x, sign*1.15, -.25], radiusM=.44, widthM=.4,
            massKg=20., meshFile="trailer_"+axle+"_"+side+".obj")
            for axle, x in (("front", -2.9), ("rear", -4.)) for side, sign in (("left", 1), ("right", -1))])]
    return config


def test_passive_trailer_joint_and_visual_tree_matches_between_sdf_and_urdf():
    config = configuration()
    validate(config)
    jsonschema.validate(config, json.loads((ROOT/"schema/vehicle_config.schema.json").read_text()))
    sdf, urdf = ET.fromstring(generate_sdf(config)), ET.fromstring(generate_urdf(config))
    for root, prefix in ((sdf, "./model/"), (urdf, "./")):
        assert root.find(prefix+"joint[@name='farm_trailer_mount_joint']").get("type") == "revolute"
        for wheel in config["implements"][0]["wheels"]:
            name = "farm_trailer_link_"+wheel["id"]
            assert root.find(prefix+f"link[@name='{name}']/visual/geometry/mesh") is not None
            assert root.find(prefix+f"joint[@name='{name}_joint']") is not None
    published = sdf.findall("./model/plugin[@name='trailer_joint_states']/joint_name")
    assert len(published) == 6
    assert "farm_trailer_mount_joint" in [j.text for j in published]
    assert "farm_trailer_pitch_joint" in [j.text for j in published]


def test_swept_envelope_contains_every_allowed_hitch_angle_not_only_neutral():
    config = configuration()
    envelope = collision_boxes(config)[0]
    for step in range(121):
        yaw = -.6 + .01*step
        for x in (-5.5, -.5):
            for y in (-1.4, 1.4):
                point = [math.cos(yaw)*x-math.sin(yaw)*y-1., math.sin(yaw)*x+math.cos(yaw)*y]
                assert all(envelope[i]-1e-9 <= point[i] <= envelope[i+3]+1e-9 for i in range(2))
    assert envelope[1] < -3.5
    assert envelope[4] > 3.5


@pytest.mark.parametrize("fault", ["duplicate_wheel", "unsafe_mesh", "excessive_yaw", "negative_mass", "tilted_hitch"])
def test_invalid_trailer_configuration_is_rejected(fault):
    config = configuration()
    trailer = config["implements"][0]
    if fault == "duplicate_wheel":
        trailer["wheels"][1] = copy.deepcopy(trailer["wheels"][0])
    elif fault == "unsafe_mesh":
        trailer["wheels"][0]["meshFile"] = "../../outside.obj"
    elif fault == "excessive_yaw":
        trailer["yawLimitRad"] = 1.4
    elif fault == "negative_mass":
        trailer["wheels"][0]["massKg"] = -1
    else:
        trailer["pose"]["rpy"][0] = .1
    with pytest.raises(ConfigError):
        validate(config)
