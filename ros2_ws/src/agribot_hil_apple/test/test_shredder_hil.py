import hashlib
import importlib.util
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from agribot_hil_apple.tracked_work_wire import mounted_work, shredder_feedback, validate, work_targets


ROOT = Path(__file__).resolve().parents[1]


def test_only_the_mounted_implement_can_receive_nonzero_commands():
    assert work_targets((0, 0, 3500), 'shredder') == pytest.approx([1260.])
    assert work_targets((0, 0, 10000), 'shredder') == [3600.]
    assert work_targets((3, 4, 0), 'sprayer') == [900., 800.]
    for kind, values in [('sprayer', (0, 0, 1)), ('shredder', (1, 0, 0)),
                         ('shredder', (0, 1, 1)), ('unknown', (0, 0, 0))]:
        with pytest.raises(ValueError):
            mounted_work(values, kind)


def test_shredder_actual_rpm_feedback_matches_original_protocol():
    for rpm in (-1250.4, 0., 1250.4):
        frames = shredder_feedback(rpm, 23)
        assert [identifier for identifier, _ in frames] == [0x5b0, 0x5b4]
        common, motor = [data for _, data in frames]
        assert common[0] == 1 and common[2] == (8 if rpm else 0)
        assert struct.unpack_from('<h', motor, 1)[0] == round(rpm)
        assert motor[0] == 0 and motor[3:5] == bytes(2)
        for _, data in frames:
            validate(data)
            assert data[6] == 7
    assert shredder_feedback(0, 0, True)[0][1][2] == 1
    assert shredder_feedback(0, 0, True)[1][1][0] == 1
    with pytest.raises(ValueError):
        shredder_feedback(math.nan, 0)


def test_separate_shredder_scene_preserves_baseline_physics_and_algorithms(tmp_path):
    baseline = Path.home()/'.cache/agribot/isobus/tracked_orchard'
    cad = Path.home()/'.cache/agribot/isobus/work_models/shredder/rigged'
    if not (baseline/'rdk/mission.json').is_file() or not (cad/'rotor.dae').is_file():
        pytest.skip('Optional original CAD and accepted orchard integration fixture')
    spec = importlib.util.spec_from_file_location('shredder_preparer', ROOT/'scripts/prepare_tracked_shredder.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    before = {p.relative_to(baseline): hashlib.sha256(p.read_bytes()).digest()
              for p in baseline.rglob('*') if p.is_file()}
    output = tmp_path/'shredder'
    manifest = module.build(baseline, cad, output)
    assert manifest['implement_kind'] == 'shredder'
    assert manifest['shredding']['shredder_level'] == 3500
    assert 'spraying' not in manifest
    original = json.loads((baseline/'rdk/mission.json').read_text())
    for key in ('algorithms', 'fixed_route_points', 'goal_tolerance_m', 'spawn'):
        assert manifest[key] == original[key]
    for path, digest in before.items():
        assert hashlib.sha256((baseline/path).read_bytes()).digest() == digest
        if str(path) not in ('vehicle.sdf', 'vehicle.urdf', 'rdk/mission.json'):
            assert hashlib.sha256((output/path).read_bytes()).digest() == digest
    model = ET.parse(output/'vehicle.sdf').find('model')
    old = ET.parse(baseline/'vehicle.sdf').find('model')
    assert module.signature(old) == module.signature(model)
    assert model.find("joint[@name='work_shredder_joint']") is not None
    assert not model.findall(".//plugin[@name='work_spray']")
    assert model.find("plugin[@name='work_spray_obstacles']") is None
    for name in ('work_plant_frame', 'work_shredder_rotor'):
        assert not model.findall(f"link[@name='{name}']/collision")
    parent = list(map(float, model.findtext("link[@name='work_plant_frame']/pose").split()))
    local = list(map(float, model.findtext("link/visual[@name='supplied_shredder_cad']/pose").split()))
    assert module.rotation(parent[3:]) @ local[:3]+parent[:3] == pytest.approx(module.MOUNT)
    mounted_rotation = module.rotation(module.MOUNT_RPY)
    assert module.rotation(parent[3:]) @ module.rotation(local[3:]) == pytest.approx(mounted_rotation, abs=1e-10)
    assert mounted_rotation @ [-1., 0., 0.] == pytest.approx([1., 0., 0.], abs=1e-10)
    rotor_pose = list(map(float, model.findtext("link[@name='work_shredder_rotor']/pose").split()))
    source_parts = json.loads((cad/'parts.json').read_text())
    assert rotor_pose[:3] == pytest.approx(np.asarray(module.MOUNT)+module.SCALE*(mounted_rotation @ source_parts['rotor_origin_m']))
    axis = list(map(float, model.findtext("joint[@name='work_shredder_joint']/axis/xyz").split()))
    assert module.rotation(rotor_pose[3:]) @ axis == pytest.approx([0., 1., 0.], abs=1e-10)
    with pytest.raises(FileExistsError):
        module.build(baseline, cad, output)
