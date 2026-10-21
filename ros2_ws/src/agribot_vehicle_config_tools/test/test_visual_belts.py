import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from agribot_vehicle_config_tools.generator import generate_sdf, generate_simulation_sdf
from agribot_vehicle_config_tools.visual_belts import BeltPath, copy_assets


BASELINE = Path(__file__).resolve().parents[2] / "agribot_vehicle_description/config/vehicles/harvester_header/vehicle_config.json"


def configuration():
    config = json.loads(BASELINE.read_text())
    config["metadata"]["simulationVisualBelts"] = [dict(id="left", sourceWheel="front_left", sourceRadius=.862,
        centerY=1.55, points=[[0,0],[0,1],[2,1],[2,0]],
        segments=[dict(index=0,distance=.5,meshFile="belt_left_0.obj")])]
    return config


def test_closed_loop_travels_forward_reverse_and_wraps():
    path = BeltPath([[0,0],[0,1],[2,1],[2,0]])
    assert path.length == 6
    assert path.sample(.5)[:2] == (0,.5)
    assert path.sample(1.5)[:2] == (.5,1)
    assert path.sample(-.5)[:2] == (.5,0)
    assert path.sample(6.5) == path.sample(.5)
    assert path.project(.5,1.05) == pytest.approx((.05,1.5))


@pytest.mark.parametrize("points", [[], [[0,0],[1,0]], [[0,0],[0,0],[1,1]], [[0,0],[1,0],[0,float('nan')]]])
def test_invalid_path_rejected(points):
    with pytest.raises(ValueError):
        BeltPath(points)


def test_belts_are_visual_only_in_both_gazebo_exports(tmp_path):
    config = configuration()
    before = copy.deepcopy(config)
    before["metadata"].pop("simulationVisualBelts")
    for build in (generate_sdf, lambda c: generate_simulation_sdf(c, BASELINE.parents[4], tmp_path)):
        old, new = ET.fromstring(build(before)), ET.fromstring(build(config))
        base = new.find("model/link[@name='base_link']")
        visual = base.find("visual[@name='belt_left_0']")
        assert visual is not None
        assert visual.findtext("plugin/source_link") == "front_left_wheel_link"
        base.remove(visual)
        ET.indent(old,space="  "); ET.indent(new,space="  ")
        assert ET.tostring(old) == ET.tostring(new)


def test_belt_assets_copied(tmp_path):
    source, target = tmp_path/'source', tmp_path/'target'
    source.mkdir(); target.mkdir()
    (source/'belt_left_0.obj').write_text('mesh')
    copy_assets(configuration(), source, target)
    assert (target/'belt_left_0.obj').read_text() == 'mesh'
