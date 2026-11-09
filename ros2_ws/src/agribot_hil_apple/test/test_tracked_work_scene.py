import hashlib
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from launch.actions import RegisterEventHandler


ROOT = Path(__file__).resolve().parents[1]


def load(path):
    spec = importlib.util.spec_from_file_location('hil_test_'+path.stem.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_source_preserved_only_native_actuator_input_no_implement_collision(tmp_path):
    description = ROOT.parent/'agribot_vehicle_description'
    source = description/'generated/unity_car3/models/unity_car3.sdf'
    before = hashlib.sha256(source.read_bytes()).hexdigest()
    cad = tmp_path/'cad'
    cad.mkdir()
    for file in ('provenance.json', 'visual_lod.json'):
        (cad/file).write_text('{}')
    output = tmp_path/'scene'
    load(ROOT/'scripts/prepare_tracked_work_scene.py').prepare(description, cad, output)
    assert before == hashlib.sha256(source.read_bytes()).hexdigest()
    model = ET.parse(output/'vehicle.sdf').find('model')
    assert not model.findall("link[@name='work_plant_frame']/collision")
    assert not any(link.get('name').startswith('manipulator_') for link in model.findall('link'))
    assert model.find("plugin[@name='configured_diff_drive']/ros/remapping").text == 'cmd_vel:=/hil/actuator/cmd_vel'
    assert model.find("plugin[@name='work_motors']") is not None
    assert model.find("link[@name='base_link']/visual[@name='work_mounting_cradle']") is not None
    assert ET.parse(output/'world.sdf').find('world/model/link/collision/geometry/plane') is not None
    assert not (output/'vehicle.sdf').read_text().startswith('<?xml')
    assert json.loads((output/'provenance.json').read_text())['implement_collision'] is False


def test_critical_endpoints_have_exit_handlers():
    pc = load(ROOT/'launch/tracked_work_pc.launch.py').generate_launch_description()
    rdk = load(ROOT/'launch/tracked_work_rdk.launch.py').generate_launch_description()
    assert sum(isinstance(action, RegisterEventHandler) for action in pc.entities) == 1
    assert sum(isinstance(action, RegisterEventHandler) for action in rdk.entities) == 3
