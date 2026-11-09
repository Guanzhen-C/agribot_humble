from collections import Counter
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from agribot_hil_apple.rtk_visual import (
    RTK_LOWERING_M, RTK_SUPPORTS, RTK_SHORT_STEMS, antenna_heads_only, prepare_antenna_heads,
    prepare_rtk_visual, without_vertical_posts)
from agribot_hil_apple.scene import SPAWN, parameters, vehicle_model, vehicle_urdf


def object_faces(obj):
    result, name = {}, ''
    for line in obj.splitlines():
        fields = line.split(maxsplit=1)
        if fields and fields[0] == 'o':
            name = fields[1]
        elif fields and fields[0] == 'f':
            result.setdefault(name, []).append(line)
    return result


def test_only_named_posts_lose_primitives_and_indices_remain_unchanged():
    obj = ('mtllib sensor.mtl\no Cube1\nv 0 0 0\nv 1 0 0\nv 0 1 0\n'
           'usemtl white\nf 1 2 3\no Cube2\nf -3 -2 -1\n'
           'o Antenna With Spaces\nf -3 -2 -1\n')
    result, removed = without_vertical_posts(obj)
    assert removed == {'Cube1': 1, 'Cube2': 1}
    assert object_faces(result) == {'Antenna With Spaces': ['f -3 -2 -1']}
    assert [line for line in result.splitlines() if line.startswith('v ')] == [
        line for line in obj.splitlines() if line.startswith('v ')]
    assert 'mtllib sensor.mtl' in result and 'usemtl white' in result


def test_changed_cad_object_names_fail_instead_of_silently_keeping_posts():
    with pytest.raises(ValueError, match='Expected two RTK'):
        without_vertical_posts('o Antenna\nv 0 0 0\nf 1 1 1\n')


def test_actual_car3_heads_have_no_supports_and_preserve_indices(tmp_path):
    generated = Path(__file__).resolve().parents[2]/'agribot_vehicle_description/generated/unity_car3'
    source = generated/'models/sensor_rtk_master_visual.obj'
    before = source.read_bytes()
    mesh = prepare_antenna_heads(generated, tmp_path)
    assert source.read_bytes() == before
    old_faces, new_faces = object_faces(before.decode()), object_faces(mesh.read_text())
    assert set(old_faces)-set(new_faces) == RTK_SUPPORTS
    assert new_faces == {name: faces for name, faces in old_faces.items() if name not in RTK_SUPPORTS}
    old_vertices = [line for line in before.decode().splitlines() if line.startswith('v ')]
    assert len(old_vertices) == sum(line.startswith('v ') for line in mesh.read_text().splitlines())
    heads, current, head = [[], []], '', -1
    for line in mesh.read_text().splitlines():
        parts = line.split()
        if parts and parts[0] == 'o':
            current = parts[1]
            head += current == 'Base'
        elif parts and parts[0] == 'v' and current not in RTK_SUPPORTS:
            heads[head].append(list(map(float, parts[1:4])))
    bounds = [(np.min(vertices, axis=0), np.max(vertices, axis=0)) for vertices in heads]
    assert (bounds[0][0][1]+bounds[0][1][1])/2 == pytest.approx(-.52, abs=.004)
    assert (bounds[1][0][1]+bounds[1][1][1])/2 == pytest.approx(0, abs=.004)
    for low, high in bounds:
        assert low[2] == pytest.approx(-.038, abs=.0001)
        assert high[2] < .023
    with pytest.raises(ValueError, match='Expected two antenna'):
        antenna_heads_only('o Base\nv 0 0 0\nf 1 1 1\n')


def test_car3_retains_ackermann_style_short_stems_without_tall_posts(tmp_path):
    generated = Path(__file__).resolve().parents[2]/'agribot_vehicle_description/generated/unity_car3'
    source = generated/'models/sensor_rtk_master_visual.obj'
    mesh = prepare_antenna_heads(generated, tmp_path, retain_short_stems=True)
    original_faces, faces = object_faces(source.read_text()), object_faces(mesh.read_text())
    assert set(original_faces)-set(faces) == RTK_SUPPORTS-RTK_SHORT_STEMS
    assert all(faces[name] == original_faces[name] for name in RTK_SHORT_STEMS)
    points = []
    current = ''
    for line in mesh.read_text().splitlines():
        fields = line.split()
        if fields and fields[0] == 'o':
            current = fields[1]
        elif fields and fields[0] == 'v' and current == 'HandleStem':
            points.append(list(map(float, fields[1:4])))
    assert np.max(points, axis=0)[2]-np.min(points, axis=0)[2] == pytest.approx(.062, abs=1e-6)
    assert np.min(points, axis=0)[2] == pytest.approx(-.10)
    report = json.loads((mesh.parent/'provenance.json').read_text())
    assert report['short_stems_retained'] is True


def test_actual_rtk_antennas_lower_onto_deck_without_changing_other_geometry(tmp_path):
    from ament_index_python.packages import get_package_share_directory
    generated = Path(get_package_share_directory('agribot_vehicle_description')) / 'generated/ackermann_current'
    source = generated / 'models/sensor_rtk_master_visual.obj'
    before = source.read_bytes()
    mesh = prepare_rtk_visual(generated, tmp_path)
    old_faces = object_faces(before.decode('utf-8'))
    for name in ('Cube1', 'Cube2'):
        assert len(old_faces.pop(name)) == 12
    assert object_faces(mesh.read_text()) == old_faces
    assert source.read_bytes() == before
    assert (mesh.parent / source.with_suffix('.mtl').name).read_bytes() == source.with_suffix('.mtl').read_bytes()
    report = json.loads((mesh.parent / 'provenance.json').read_text())
    assert report['source_sha256'] == hashlib.sha256(before).hexdigest()
    assert report['removed_primitives'] == {'Cube1': 12, 'Cube2': 12}
    assert report['antenna_lowering_m'] == RTK_LOWERING_M
    original = vehicle_model(generated)
    updated = vehicle_model(generated, mesh)
    old_uri = original.find("model/link/visual[@name='rtk_master_cad_visual']/geometry/mesh/uri")
    new_uri = updated.find("model/link/visual[@name='rtk_master_cad_visual']/geometry/mesh/uri")
    assert new_uri.text == mesh.as_uri()
    new_uri.text = old_uri.text
    path = "model/link/visual[@name='rtk_master_cad_visual']/pose"
    new_pose = list(map(float, updated.findtext(path).split()))
    old_pose = list(map(float, original.findtext(path).split()))
    assert new_pose[2] == pytest.approx(old_pose[2]-RTK_LOWERING_M)
    assert new_pose[:2]+new_pose[3:] == old_pose[:2]+old_pose[3:]
    updated.find(path).text = original.findtext(path)
    assert ET.tostring(updated) == ET.tostring(original)
    urdf = generated / 'urdf/ackermann_current.urdf.xacro'
    original = ET.fromstring(vehicle_urdf(urdf))
    updated = ET.fromstring(vehicle_urdf(urdf, mesh))
    path = "link[@name='rtk_link']/visual/geometry/mesh"
    assert updated.find(path).get('filename') == mesh.as_uri()
    updated.find(path).set('filename', original.find(path).get('filename'))
    for name in ('rtk_master_mount_joint', 'rtk_secondary_mount_joint'):
        path = f"joint[@name='{name}']/origin"
        new_xyz = list(map(float, updated.find(path).get('xyz').split()))
        old_xyz = list(map(float, original.find(path).get('xyz').split()))
        assert new_xyz[:2] == old_xyz[:2]
        assert new_xyz[2] == pytest.approx(old_xyz[2]-RTK_LOWERING_M)
        updated.find(path).set('xyz', original.find(path).get('xyz'))
    assert ET.tostring(updated) == ET.tostring(original)
    assert Counter(line.split()[0] for line in mesh.read_text().splitlines() if line.startswith(('v ', 'vt ', 'vn '))) == Counter(
        line.split()[0] for line in before.decode('utf-8').splitlines() if line.startswith(('v ', 'vt ', 'vn ')))
    before_vertices = np.array([list(map(float, line.split()[1:4])) for line in before.decode('utf-8').splitlines() if line.startswith('v ')])
    after_vertices = np.array([list(map(float, line.split()[1:4])) for line in mesh.read_text().splitlines() if line.startswith('v ')])
    assert np.allclose(after_vertices[:24]-before_vertices[:24], [0., 0., RTK_LOWERING_M])
    assert np.array_equal(after_vertices[24:], before_vertices[24:])
    # The antenna feet now meet the original deck rail (about z=0.135 m).
    feet_z = before_vertices[:, 2][np.isclose(before_vertices[:, 2], -.129138947, atol=1e-8)]
    assert len(feet_z) and np.all((new_pose[2]+feet_z > .13) & (new_pose[2]+feet_z < .16))


def test_lowered_rtk_lever_arm_matches_simulation_and_georeference(tmp_path):
    from ament_index_python.packages import get_package_share_directory
    generated = Path(get_package_share_directory('agribot_vehicle_description'))/'generated/ackermann_current'
    ackermann = Path(get_package_share_directory('agribot_ackermann_mppi'))
    parameters(generated, ackermann, tmp_path)
    fusion = yaml.safe_load((tmp_path/'fusion.yaml').read_text())['fastlivo_rtk_fusion']['ros__parameters']
    original = yaml.safe_load((generated/'config/physical/fastlivo_rtk_fusion.yaml').read_text())['fastlivo_rtk_fusion']['ros__parameters']['base_to_antenna_xyz']
    expected = [*original[:2], original[2]-RTK_LOWERING_M]
    assert fusion['base_to_antenna_xyz'] == expected
    manifest = json.loads((tmp_path/'mission.json').read_text())
    assert manifest['base_to_antenna'] == expected
    georef = yaml.safe_load((tmp_path/'georeference.yaml').read_text())
    assert georef['map_from_enu']['xyz'][2] == pytest.approx(SPAWN[2]+expected[2])
