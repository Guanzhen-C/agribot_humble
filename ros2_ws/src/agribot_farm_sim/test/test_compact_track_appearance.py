import json
import math
from pathlib import Path

import pytest

from agribot_vehicle_config_tools.visual_belts import BeltPath


ROOT = Path(__file__).resolve().parents[2] / "agribot_vehicle_description/generated/unity_car3"


def configuration():
    config = json.loads((ROOT / "vehicle_config.json").read_text())
    assert config["metadata"]["simulationAppearanceRevision"] == "compact_articulated_tracks_v1"
    return config


def test_car3_contact_dimensions_sensors_and_appearance_scope():
    config = configuration()
    assert config["geometry"]["wheelbaseM"] == pytest.approx(.402)
    assert config["geometry"]["frontTrackM"] == pytest.approx(.589)
    assert config["geometry"]["wheelRadiusM"] == pytest.approx(.22491844)
    assert config["dynamics"]["massKg"] == 50
    assert config["sensors"][1]["pose"]["xyz"][2] == pytest.approx(.6974544525)
    assert len(config["metadata"]["simulationVisualRotors"]) == 14
    assert len(config["metadata"]["simulationVisualBelts"]) == 2


def test_compact_treads_fit_the_existing_body_width_and_length():
    config = configuration()
    extents = config["geometry"]["bodyExtents"]
    for belt in config["metadata"]["simulationVisualBelts"]:
        path = BeltPath(belt["points"])
        assert len(belt["segments"]) == 64
        for part in belt["segments"]:
            x,z,angle = path.sample(part["distance"])
            ca,sa = math.cos(angle),math.sin(angle)
            with (ROOT / "models" / part["meshFile"]).open() as stream:
                for line in stream:
                    if not line.startswith("v "):
                        continue
                    a,b,c = map(float,line.split()[1:4])
                    px,py = x+ca*a+sa*c,belt["centerY"]+b
                    assert -extents["rearM"]-.001 <= px <= extents["frontM"]+.001
                    assert -extents["rightM"]-.001 <= py <= extents["leftM"]+.001


def test_no_original_fixed_track_objects_remain():
    with (ROOT / "models/vehicle_visual.obj").open() as stream:
        for line in stream:
            if line.startswith("o "):
                prefix = line[2:].split("_",1)[0]
                assert not prefix.isdigit() or not 68 <= int(prefix) <= 109
