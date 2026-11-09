import hashlib
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from agribot_hil_apple.tracked_orchard_mission import spray_allowed


ROOT = Path(__file__).resolve().parents[1]


def rpy_rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def test_spraying_only_in_rows_while_moving_not_headland():
    config = dict(minimum_speed_mps=.03, row_y=[36., 32.5], row_half_width_m=.65, x_range=[3., 39.5])
    assert spray_allowed(config, 20, 36, 0, .2, True)
    assert spray_allowed(config, 20, 32.5, 3.14159, .2, True)
    assert not spray_allowed(config, 41, 36, 0, .2, True)
    assert not spray_allowed(config, 20, 36, 1.57, .2, True)
    assert not spray_allowed(config, 20, 36, 0, 0, True)
    assert not spray_allowed(config, 20, 36, 0, .2, False)


def test_factory_spray_positions_match_real_step_bores(monkeypatch):
    provenance = Path.home()/'.cache/agribot/isobus/work_models/plant_protection/provenance.json'
    if not provenance.exists():
        pytest.skip('Original STEP is an optional geometry integration fixture')
    source = Path(json.loads(provenance.read_text())['source'])
    if not source.exists():
        pytest.skip('Original STEP is not installed')
    monkeypatch.syspath_prepend('/usr/lib/freecad/lib')
    pytest.importorskip('FreeCAD')
    part = pytest.importorskip('Part')
    monkeypatch.syspath_prepend(str(ROOT/'scripts'))
    import prepare_tracked_orchard as module
    assembly = part.read(str(source))
    axis = np.array(module.FAN_AXIS)

    def radial_distance(point):
        delta = np.array(point)-module.CAD_FAN_CENTRE_MM
        return np.linalg.norm(delta-(delta @ axis)*axis)

    for solid_id, point in zip((66, 68, 70, 72, 74, 76, 78, 80), module.CAD_NOZZLE_OUTLETS_MM):
        bore = assembly.Solids[solid_id].Faces[0]
        assert bore.Surface.Radius == pytest.approx(.4)
        circles = [tuple(edge.Curve.Center) for edge in bore.Edges
                   if hasattr(edge.Curve, 'Radius') and edge.Curve.Radius == pytest.approx(.4)]
        assert len(circles) == 2
        assert point == pytest.approx(max(circles, key=radial_distance))


def test_orchard_preparation_preserves_sources_and_matches_car3(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT/'scripts'))
    spec = importlib.util.spec_from_file_location('tracked_orchard_preparer', ROOT/'scripts/prepare_tracked_orchard.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original = Path.home()/'.cache/agribot/hil_apple'
    if not original.is_dir():
        pytest.skip('Prepared original orchard is an optional integration fixture')
    before = hashlib.sha256((original/'world.sdf').read_bytes()).hexdigest()
    cad = tmp_path/'cad'
    cad.mkdir()
    for file in ('provenance.json', 'visual_lod.json'):
        (cad/file).write_text(json.dumps({'visual_size_m': [1.2182119437119925, 1.459678696072802,
                                                         .8738763220464185]}) if file == 'provenance.json' else '{}')
    (cad/'textured').mkdir()
    (cad/'textured/visual_textured.dae').write_text('<COLLADA/>')
    output = tmp_path/'orchard'
    manifest = module.build(ROOT.parent/'agribot_vehicle_description', cad, original, output)
    assert before == hashlib.sha256((original/'world.sdf').read_bytes()).hexdigest()
    assert manifest['fixed_route_points'] == [[41., 36., 0.], [41., 32.5, 3.14159], [3., 32.5, 3.14159]]
    nav = yaml.safe_load((output/'rdk/nav2.yaml').read_text())
    controller = nav['controller_server']['ros__parameters']
    assert controller['FollowPath']['motion_model'] == 'DiffDrive'
    assert controller['goal_checker']['xy_goal_tolerance'] == pytest.approx(.8370031098092812/2)
    align = controller['FollowPath']['PathAlignCritic']
    assert align['cost_weight'] == 3.5 and align['trajectory_point_step'] == 4
    assert align['offset_from_furthest'] == 6
    assert controller['FollowPath']['PathFollowCritic']['cost_weight'] == 7.
    assert controller['FollowPath']['PathAngleCritic']['offset_from_furthest'] == 8
    assert controller['FollowPath']['CostCritic']['consider_footprint'] is True
    assert controller['FollowPath']['CostCritic']['collision_cost'] == 1000000.
    fastlivo = yaml.safe_load((output/'rdk/fastlivo.yaml').read_text())['/**']['ros__parameters']
    assert fastlivo['extrin_calib']['extrinsic_T'] == pytest.approx([
        .48019593954086304-module.IMU_XYZ[0], -.007781699765473604,
        .6974544525146484-module.IMU_XYZ[2]])
    assert fastlivo['time_offset']['img_time_offset'] == 0
    assert hashlib.sha256((output/'rdk/map.pgm').read_bytes()).hexdigest() == manifest['map_image_sha256']
    world = ET.parse(output/'world.sdf')
    assert world.find("world/model[@name='harvest_apple_1']/static").text == 'true'
    assert world.find("world/plugin[@name='hil_fruit_attachments']") is None
    model = ET.parse(output/'vehicle.sdf').find('model')
    assert 'physics' not in manifest
    assert float(model.findtext("plugin[@name='configured_diff_drive']/max_wheel_torque")) == 200.
    assert float(model.findtext("link[@name='base_link']/inertial/pose").split()[2]) == pytest.approx(.256492316723)
    slip = model.find("plugin[@name='configured_tracked_wheel_slip']")
    assert float(slip.findtext('straight_compliance')) == .1
    assert float(slip.findtext('turning_slip')) == .003
    for wheel in model.findall('link/collision/surface/friction/ode'):
        assert float(wheel.findtext('mu')) == .36
        assert float(wheel.findtext('mu2')) == 1.8
    for name in ('local_costmap', 'global_costmap'):
        assert nav[name][name]['ros__parameters']['footprint_padding'] == 0.
    assert model.find(".//sensor[@name='hikrobot_right_camera']/camera/image/width").text == '640'
    assert model.find("link[@name='work_plant_frame']/collision") is None
    assert not model.findall("link[@name='base_link']/visual[@name='work_hitch_rail_1']")
    assert model.find("link[@name='base_link']/visual[@name='work_mounting_cradle']") is None
    plant = model.find("link[@name='work_plant_frame']")
    pose = list(map(float, plant.findtext('pose').split()))
    scale = list(map(float, plant.findtext("visual[@name='supplied_plant_cad']/geometry/mesh/scale").split()))
    assert scale == [module.SPRAYER_SCALE]*3
    assert manifest['presentation']['implement_textures'] is True
    assert plant.findtext("visual[@name='supplied_plant_cad']/geometry/mesh/uri") == (cad/'textured/visual_textured.dae').as_uri()
    body = model.find("link/visual[@name='vehicle_body_visual']/geometry/mesh/uri")
    assert body.text == (output/'assets/chassis/vehicle_visual.obj').as_uri()
    original_body = ROOT.parent/'agribot_vehicle_description/generated/unity_car3/models/vehicle_visual.obj'
    assert (output/'assets/chassis/vehicle_visual.obj').read_bytes() == original_body.read_bytes()
    # Preserve the requested horizontal direction while laying mounting tabs flat.
    rotation = rpy_rotation(pose[3:])
    assert rotation @ module.FAN_AXIS == pytest.approx([-1., 0., 0.], abs=1e-10)
    assert rotation @ module.MOUNT_NORMAL == pytest.approx([0., 0., -1.], abs=1e-10)
    low = [pose[i]+scale[i]*module.ALIGNED_CAD_MIN[i] for i in range(3)]
    high = [pose[i]+scale[i]*module.ALIGNED_CAD_MAX[i] for i in range(3)]
    assert low[2] == pytest.approx(module.DECK_Z_M)
    assert low[0] > -.48 and high[0] < .28
    assert low[1] > -.25 and high[1] < .25
    assert manifest['presentation']['layout'] == 'tank_front_fan_rear'
    assert manifest['presentation']['mounting_plate_removed'] is True
    # Outer contact faces of the four supplied STEP mounting tabs, in source mm.
    contact_points = np.array([
        [508.5350082774786, -431.0893246110511, 341.6694355613555],
        [508.53500827747996, -431.0893246110488, -341.66943556135845],
        [867.9667488597738, 235.13723134569767, -303.22828633669326],
        [867.9667488597727, 235.1372313456953, 303.228286336696]])*.001
    normalized = contact_points-np.array([.4302674591044463, .1438545493858059, -.46387632204641795])
    contacts_base = normalized @ rotation.T * scale + pose[:3]
    assert contacts_base[:, 2] == pytest.approx([module.DECK_Z_M]*4, abs=1e-6)
    assert np.all(contacts_base[:, 0] > -.505) and np.all(contacts_base[:, 0] < .63)
    assert np.all(np.abs(contacts_base[:, 1]) < .37)
    emitters = plant.findall("visual[@name='spray_nozzles']/plugin/nozzle")
    assert len(emitters) == module.SPRAY_NOZZLE_COUNT == 8
    obstacles = model.find("plugin[@name='work_spray_obstacles']")
    assert obstacles is not None
    assert obstacles.findtext('link_name') == 'work_plant_frame'
    fields = lambda nodes: [(n.get('name'), n.findtext('position'), n.findtext('direction')) for n in nodes]
    assert fields(obstacles.findall('nozzle')) == fields(emitters)
    spray = plant.find("visual[@name='spray_nozzles']/plugin")
    assert spray.findtext('obstacle_clipping') == 'true'
    for key, value in module.MIST_VISUAL.items():
        assert float(spray.findtext(key)) == pytest.approx(value)
    assert module.MIST_VISUAL['velocity_min'] == 2.
    assert module.MIST_VISUAL['velocity_max'] == 2.8
    assert module.MIST_VISUAL['total_emission_rate'] == 2400
    assert list(map(float, spray.findtext('particle_colour').split())) == module.MIST_COLOUR
    assert module.MIST_COLOUR == [.36, .60, .78, .60]
    assert manifest['presentation']['spray_visual_parameters'] == dict(module.MIST_VISUAL, colour=module.MIST_COLOUR)
    outlets = []
    fan_centre = (np.array(module.CAD_FAN_CENTRE_MM)-module.CAD_ORIGIN_MM)*.001*module.SPRAYER_SCALE
    for emitter, source in zip(emitters, module.CAD_NOZZLE_OUTLETS_MM):
        outlet = np.array(list(map(float, emitter.findtext('position').split())))
        direction = np.array(list(map(float, emitter.findtext('direction').split())))
        outlets.append(outlet)
        assert np.linalg.norm(direction) == pytest.approx(1.)
        assert direction @ module.FAN_AXIS == pytest.approx(0., abs=1e-10)
        assert (rotation @ direction)[0] == pytest.approx(0., abs=1e-10)
        radial = outlet-fan_centre
        radial -= (radial @ module.FAN_AXIS)*np.array(module.FAN_AXIS)
        assert direction == pytest.approx(radial/np.linalg.norm(radial))
        # Emit just outside a real bore mouth, not from the axial fastener plane.
        expected = (np.array(source)-module.CAD_ORIGIN_MM+module.NOZZLE_CLEARANCE_MM*direction)*.001*module.SPRAYER_SCALE
        assert outlet == pytest.approx(expected)
        assert .19 < np.linalg.norm(radial) < .22
    assert not any(node.get('name', '').startswith('water_nozzle_') for node in plant.findall('visual'))
    outlets = np.array(outlets)
    world_outlets = outlets @ rotation.T+pose[:3]
    assert np.min(world_outlets[:, 2]) > module.DECK_Z_M+.006
    assert float(plant.findtext("visual[@name='spray_nozzles']/geometry/sphere/radius")) < .0001
    vehicle = json.loads((output/'vehicle_config.json').read_text())
    imu = next(s for s in vehicle['sensors'] if s['id'] == 'imu')
    assert imu['pose']['xyz'] == module.IMU_XYZ
    assert imu['pose']['xyz'][:2] == [.31, 0.]
    assert imu['pose']['xyz'][2]-.01 == pytest.approx(manifest['presentation']['imu_support_z_m'])
    assert manifest['presentation']['imu_mount_owner'] == 'vehicle_chassis'
    # The whole 30 mm IMU must be clear of the implement, not on its crossmember.
    assert module.IMU_XYZ[0]-.015 > high[0]+.02
    assert module.IMU_SUPPORT_Z_M == pytest.approx(.426081)
    for node in (model.find("link/visual[@name='imu_cad_visual']"),
                 model.find(".//sensor[@name='n300pro_imu']")):
        assert list(map(float, node.findtext('pose').split())) == pytest.approx(module.IMU_XYZ+[0., 0., 0.])
    bridge = yaml.safe_load((output/'rdk/odom_bridge.yaml').read_text())['fastlio_odom_bridge']['ros__parameters']
    assert bridge['base_to_body_xyz'] == module.IMU_XYZ
    assert bridge['base_to_body_rpy'] == imu['pose']['rpy']
    master = next(s for s in vehicle['sensors'] if s['id'] == 'rtk_master')['pose']['xyz']
    fusion = yaml.safe_load((output/'rdk/fusion.yaml').read_text())['fastlivo_rtk_fusion']['ros__parameters']
    assert master == fusion['base_to_antenna_xyz'] == manifest['base_to_antenna']
    assert master == module.ANTENNA_MASTER
    assert master[1] == .29
    secondary = next(s for s in vehicle['sensors'] if s['id'] == 'rtk_secondary')['pose']['xyz']
    assert secondary == [.44, -.29, master[2]]
    rtk_provenance = json.loads((output/'assets/rtk/provenance.json').read_text())
    assert rtk_provenance['baseline_m'] == pytest.approx(.58)
    assert rtk_provenance['short_stems_retained'] is True
    urdf = ET.parse(output/'vehicle.urdf')
    assert urdf.find("link[@name='work_plant_frame']/inertial") is None
    assert len(urdf.findall("link[@name='work_plant_frame']/visual")) == 1
    assert manifest['presentation']['additional_nozzle_geometry'] is False
    assert list(map(float, urdf.find("joint[@name='imu_mount_joint']/origin").get('xyz').split())) == pytest.approx(module.IMU_XYZ)
    assert urdf.find("joint[@name='imu_mount_joint']/parent").get('link') == 'base_link'
    assert list(map(float, urdf.find("joint[@name='work_mount']/origin").get('xyz').split())) == pytest.approx(pose[:3])
    assert list(map(float, urdf.find("joint[@name='work_mount']/origin").get('rpy').split())) == pytest.approx(pose[3:])
    assert list(map(float, urdf.find("joint[@name='rtk_master_mount_joint']/origin").get('xyz').split())) == pytest.approx(master)
    georef = yaml.safe_load((output/'rdk/georeference.yaml').read_text())
    assert georef['map_from_enu']['xyz'] == pytest.approx([manifest['spawn'][i]+master[i] for i in range(3)])
