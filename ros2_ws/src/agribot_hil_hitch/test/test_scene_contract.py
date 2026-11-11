"""Inspect the generated scene without launching Gazebo or modifying original assets."""
import os
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


@pytest.fixture
def scene():
    value = os.environ.get('HITCH_TEST_SCENE')
    if not value:
        pytest.skip('Set HITCH_TEST_SCENE to the prepared scene directory')
    return Path(value)


def test_no_full_plot_overlay(scene):
    world = ET.parse(scene/'world.sdf').find('world')
    assert world.find("model[@name='hil_hitch_soil']") is None
    environment = world.find("model[@name='agricultural_environment']")
    uri = environment.findtext('link/visual/geometry/mesh/uri')
    assert uri.endswith('wide_headland/scene.obj')
    assert Path(uri).is_file()


def test_original_soil_texture_on_local_effect(scene):
    model = ET.parse(scene/'vehicle.sdf').find('model')
    effect = model.find("link[@name='hitch_tool_frame']/visual[@name='soil_effect_anchor']/plugin")
    texture = Path(effect.findtext('soil_texture'))
    assert texture.is_file() and texture.name.endswith('brown_mud_diff_2k.png')
    assert float(effect.findtext('ground_z')) == -.002
    assert len(model.findall("joint[@type='revolute']")) >= 17


def test_paint_is_not_white_ambient(scene):
    model = ET.parse(scene/'vehicle.sdf').find('model')
    mesh = Path(model.findtext("link[@name='base_link']/visual[@name='vehicle_body_visual']/geometry/mesh/uri"))
    current = {}
    materials = {}
    for line in mesh.with_suffix('.mtl').read_text().splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == 'newmtl':
            current = materials.setdefault(fields[1], {})
        elif fields[0] in ('Ka', 'Kd', 'Ks'):
            current[fields[0]] = list(map(float, fields[1:4]))
    paint = next(value for name, value in materials.items() if name.startswith('01_'))
    assert paint['Kd'][0] > 3*paint['Kd'][1]
    assert all(a <= d*.26+1e-5 for a, d in zip(paint['Ka'], paint['Kd']))
    assert max(paint['Ks']) <= .1


def test_lower_sleeves_share_lift_frame(scene):
    model = ET.parse(scene/'vehicle.sdf').find('model')
    for side in ('left', 'right'):
        name = 'hitch_adapter_lower_'+side
        assert model.find(f"link[@name='hitch_tool_frame']/visual[@name='{name}']") is not None
        assert model.find(f"link[@name='base_link']/visual[@name='{name}']") is None
        assert model.find(f"plugin[@name='hitch_physical_plant']/adapter/name[.='{name}']") is None


def test_upper_links_use_actual_lift_pose(scene):
    model = ET.parse(scene/'vehicle.sdf').find('model')
    assert model.find("plugin[@name='hitch_physical_plant']/adapter") is None
    for name in ('top_link', 'cylinder_left', 'cylinder_right'):
        item = model.find(f"link[@name='base_link']/visual[@name='hitch_adapter_{name}']")
        assert item.find('geometry/cylinder') is None
        renderer = item.find("plugin[@name='telescopic_linkage']")
        assert renderer.get('filename') == 'libagribot_hil_hitch_linkage.so'
        assert len(renderer.findtext('host').split()) == 3
        assert len(renderer.findtext('tool_eye').split()) == 3


def test_deeper_working_limit_preserves_raised_limit(scene):
    model = ET.parse(scene/'vehicle.sdf').find('model')
    joint = model.find("joint[@name='hitch_lift_joint']")
    lower, upper = [float(joint.findtext('axis/limit/'+key)) for key in ('lower', 'upper')]
    assert lower == -.035 and upper == .4
    plant = model.find("plugin[@name='hitch_physical_plant']")
    assert float(plant.findtext('lift_min')) == lower
    assert float(plant.findtext('lift_span')) == pytest.approx(upper-lower)
    urdf = ET.parse(scene/'vehicle.urdf').getroot().find("joint[@name='hitch_lift_joint']/limit")
    assert float(urdf.get('lower')) == lower and float(urdf.get('upper')) == upper


def test_force_transmission_has_no_velocity_drive_bypass(scene):
    metadata = json.loads((scene/'provenance.json').read_text())
    if not metadata.get('powertrain_force_plant'):
        pytest.skip('Legacy hitch-only scene')
    model = ET.parse(scene/'vehicle.sdf').find('model')
    drive = model.find("plugin[@name='ackermann_drive']")
    assert float(drive.findtext('wheel_torque')) == 0
    plant = model.find("plugin[@name='hil_powertrain_plant']")
    assert plant is not None
    assert plant.get('filename') == 'libagribot_hil_powertrain_plant.so'
    assert metadata['velocity_drive_bypass'] is False


def test_levelled_harrow_has_both_rows_in_soil(scene):
    metadata = json.loads((scene/'provenance.json').read_text())
    if 'implement_level_pitch_rad' not in metadata:
        pytest.skip('Earlier unlevelled harrow scene')
    model = ET.parse(scene/'vehicle.sdf').find('model')
    urdf = ET.parse(scene/'vehicle.urdf').getroot()
    q = metadata['lift_min_rad']
    pivot = [0., 0., -.13]
    def rotate_y(point, angle):
        x, y, z = point
        return [math.cos(angle)*x+math.sin(angle)*z, y, -math.sin(angle)*x+math.cos(angle)*z]
    depths = []
    for disc in model.findall("plugin[@name='hitch_physical_plant']/disc"):
        link = model.find(f"link[@name='{disc.findtext('link')}']")
        pose = list(map(float, link.findtext('pose').split()))
        origin = urdf.find(f"joint[@name='{disc.findtext('joint')}']/origin")
        assert list(map(float, origin.get('rpy').split())) == pytest.approx(pose[3:])
        heights = []
        for probe in disc.findall('probe'):
            point = rotate_y(list(map(float, probe.text.split())), pose[4])
            local = [point[i]+pose[i]-pivot[i] for i in range(3)]
            heights.append(rotate_y(local, q)[2]+pivot[2]+.716)
        depths.append(min(heights))
    assert len(depths) == 16
    assert max(depths) < -.035 and min(depths) > -.15


def test_two_stage_soil_uses_original_rotating_cage(scene):
    metadata = json.loads((scene/'provenance.json').read_text())
    if not metadata.get('rotating_original_cage_roller'):
        pytest.skip('Earlier static cage roller')
    model = ET.parse(scene/'vehicle.sdf').find('model')
    roller = model.find("link[@name='hitch_cage_roller']")
    mesh = Path(roller.findtext('visual/geometry/mesh/uri'))
    assert mesh.name == 'harrow_roller.obj' and mesh.is_file()
    assert roller.find('collision') is None
    joint = model.find("joint[@name='hitch_cage_roller_joint']")
    assert joint.get('type') == 'revolute'
    assert joint.findtext('parent') == 'hitch_tool_frame'
    assert joint.findtext('axis/xyz') == '0 1 0'
    effect = model.find("link[@name='hitch_tool_frame']/visual[@name='soil_effect_anchor']/plugin")
    origins = [list(map(float, item.text.split())) for item in effect.findall('disc_origin')]
    assert len(origins) == 16
    mean_x = sum(v[0] for v in origins)/16
    assert sum(v[0] > mean_x for v in origins) == 8
    roller_x = float(effect.findtext('roller_origin').split()[0])
    assert roller_x < min(v[0] for v in origins)-.4
    assert float(effect.findtext('settled_bed_offset')) < -1.2
    assert metadata['implement_mount_drop_m'] == .085
    assert metadata['soil_effect_stages'] == ['disc_clod_throw', 'cage_roller_crumbling']
    parts = json.loads((mesh.parent/'parts.json').read_text())
    assert len(parts['roller']['source_objects']) == 128
    assert parts['decimation'] is False
