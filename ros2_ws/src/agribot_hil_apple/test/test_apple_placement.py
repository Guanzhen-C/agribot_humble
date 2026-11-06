import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import math
import yaml
from agribot_hil_apple.apple_asset import bake_stem_up
from agribot_hil_apple.scene import APPLES, BRANCH_ANCHORS, SPAWN, STEM_OFFSETS, parameters
from agribot_hil_apple.fruit_geometry import stem_dimensions


def test_scan_is_baked_around_its_actual_stem_socket(tmp_path):
    source = Path(__file__).resolve().parents[1] / 'models/ycb_apple/textured.dae'
    mesh = bake_stem_up(source, tmp_path)
    report = json.loads((tmp_path / 'alignment.json').read_text())
    assert mesh.is_file() and (tmp_path / 'texture_map.png').is_file()
    assert np.allclose(report['baked_socket'], [0, 0, 0], atol=1e-12)
    assert np.allclose(report['baked_stem_axis'], [0, 0, 1], atol=1e-12)
    assert report['bounds'][0][2] < -.06
    assert report['bounds'][1][2] < .01

    namespace = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    root = ET.parse(mesh)
    assert root.find('c:asset/c:up_axis', namespace).text == 'Z_UP'
    position_path = ".//c:source[@id='shape0-lib-positions']/c:float_array"
    original = np.fromstring(ET.parse(source).find(position_path, namespace).text,
                             sep=' ').reshape(-1, 3)
    baked = np.fromstring(root.find(position_path, namespace).text,
                         sep=' ').reshape(-1, 3)
    assert baked.shape == original.shape
    # Reorientation must not change the fruit's dimensions or distort its mesh.
    assert np.allclose(np.linalg.norm(np.diff(baked, axis=0), axis=1),
                       np.linalg.norm(np.diff(original, axis=0), axis=1), atol=1e-8)
    assert np.mean(baked[:, 2]) < -.02


def test_original_spawn_and_stem_connections_are_preserved():
    assert SPAWN == [2., 36., .1325, 0.]
    for origin, branch, offset in zip(APPLES, BRANCH_ANCHORS, STEM_OFFSETS):
        assert np.allclose(np.asarray(origin) + [0, 0, offset], branch)
        shape = stem_dimensions(offset)
        assert origin[2]+shape['visual_center']+shape['visual_length']/2 > branch[2]
        assert origin[2]+shape['visual_center']-shape['visual_length']/2 < origin[2]


def test_navigation_crop_preserves_all_original_map_pixels(tmp_path):
    from PIL import Image
    from ament_index_python.packages import get_package_share_directory
    from agribot_hil_apple.scene import save_map
    source = Path(get_package_share_directory('scout_navigation'))/'maps/orchard_v2_map6.yaml'
    original_config = yaml.safe_load(source.read_text())
    original = np.asarray(Image.open(source.parent/original_config['image']))
    save_map(tmp_path)
    config = yaml.safe_load((tmp_path/'map.yaml').read_text())
    crop = np.asarray(Image.open(tmp_path/config['image']))
    assert config['resolution'] == original_config['resolution']
    resolution = config['resolution']
    x0 = round((config['origin'][0]-original_config['origin'][0])/resolution)
    y1 = original.shape[0]-round((config['origin'][1]-original_config['origin'][1])/resolution)
    assert np.array_equal(crop, original[y1-crop.shape[0]:y1, x0:x0+crop.shape[1]])


def test_generated_navigation_stops_on_position_and_uses_real_half_wheelbase(tmp_path):
    from ament_index_python.packages import get_package_share_directory
    description = Path(get_package_share_directory('agribot_vehicle_description'))/'generated/ackermann_current'
    ackermann = Path(get_package_share_directory('agribot_ackermann_mppi'))
    parameters(description, ackermann, tmp_path)
    navigation = yaml.safe_load((tmp_path/'nav2.yaml').read_text())
    controller = navigation['controller_server']['ros__parameters']
    planner = navigation['planner_server']['ros__parameters']['GridBased']
    assert planner['change_penalty'] == 2.
    assert planner['reverse_penalty'] == 1.05
    assert planner['motion_model_for_search'] == 'DUBIN'
    checker = controller['goal_checker']
    vehicle = json.loads((description/'vehicle_config.json').read_text())
    assert checker['xy_goal_tolerance'] == vehicle['geometry']['wheelbaseM']/2
    assert checker['yaw_goal_tolerance'] == math.pi
    assert checker['stateful'] is False
    assert controller['FollowPath']['GoalAngleCritic']['enabled'] is False
    assert controller['odom_topic'] == '/wheel/odometry'
    assert controller['FollowPath']['enforce_path_inversion'] is False
    assert controller['FollowPath']['vx_min'] == 0.
    assert controller['FollowPath']['PreferForwardCritic']['enabled'] is True
    assert controller['FollowPath']['PathAngleCritic']['forward_preference'] is True
    assert controller['FollowPath']['inversion_yaw_tolerance'] == math.pi
    assert controller['FollowPath']['inversion_xy_tolerance'] == .05
    assert controller['FollowPath']['PathAlignCritic']['use_path_orientations'] is False
    mppi = controller['FollowPath']
    horizon = mppi['time_steps']*mppi['model_dt']
    assert horizon*mppi['wz_max'] > math.pi/2
    local = yaml.safe_load((tmp_path/'nav2.yaml').read_text())['local_costmap']['local_costmap']['ros__parameters']
    assert horizon*mppi['vx_max'] < min(local['width'], local['height'])/2
    manifest = json.loads((tmp_path/'mission.json').read_text())
    assert manifest['can_topology'] == 'hpm6750_native_bridge'
    assert manifest['hpm6750_enabled'] is True
    assert manifest['final_orientation_required'] is False
    assert len(manifest['pickup_stops']) == 1
    assert manifest['pickup_stops'][0]['park'] == manifest['park']
    assert manifest['pickup_orientation_required'] is True
    assert manifest['pickup_position_tolerance_m'] == .10
    assert manifest['pickup_yaw_tolerance_rad'] == .15
    assert manifest['fruit_socket_to_branch_m']['harvest_apple_2'] == .07
    source_route = yaml.safe_load((ackermann/'config/orchard_waypoints_ackermann_smac.yaml').read_text())['waypoints'][:3]
    fixed = [[p['x'], p['y'], p['yaw']] for p in source_route]
    assert manifest['fixed_route_points'] == fixed
    assert manifest['fixed_route_order'] == [1, 2, 3]
    assert manifest['forward_only'] is True
    assert manifest['outbound_via'][:2] == fixed[:2]
    assert manifest['outbound_via'][-2] == [manifest['park'][0]+1.5, manifest['park'][1], manifest['park'][2]]
    assert manifest['outbound_via'][-1] == manifest['park']
    assert manifest['completion_via'] == [fixed[2]]
    assert manifest['finish'] == fixed[2]
    assert manifest['finish_is_spawn'] is False
    assert manifest['route_sequence'] == [1, 2, 3]
    assert manifest['navigation_phases'] == ['outbound', 'completion']
    assert manifest['pickup_stops'][0]['target_models'] == ['harvest_apple_5', 'harvest_apple_7']
    assert 'return_via' not in manifest
    assert abs(manifest['outbound_via'][1][2]-manifest['park'][2]) < 1e-5
    assert manifest['outbound_via'][-2][0] > manifest['park'][0]
    assert abs(manifest['completion_via'][0][2]-manifest['park'][2]) < 1e-5
    for critic in ('PathAlignCritic', 'PathFollowCritic'):
        assert controller['FollowPath'][critic]['threshold_to_consider'] < manifest['pickup_position_tolerance_m']


def test_tray_has_chassis_attached_supports_in_physics_and_moveit():
    from ament_index_python.packages import get_package_share_directory
    from agribot_hil_apple.scene import CARGO_PARTS, vehicle_model, vehicle_urdf
    source = Path(get_package_share_directory('agribot_vehicle_description'))/'generated/ackermann_current'
    assert vehicle_model(source).find("model/plugin[@name='hil_gripper_transit_hold']").get('filename') == 'libagribot_hil_gripper_hold.so'
    sdf = vehicle_model(source).find("model/link[@name='base_link']")
    urdf = ET.fromstring(vehicle_urdf(source/'urdf/ackermann_current.urdf.xacro')).find("link[@name='base_link']")
    chassis = sdf.find("collision[@name='vehicle_body_collision']")
    center = np.fromstring(chassis.findtext('pose'), sep=' ')[:3]
    half = np.fromstring(chassis.findtext('geometry/box/size'), sep=' ')/2
    floor = next((np.array(xyz), np.array(size)) for name, xyz, size in CARGO_PARTS if name=='cargo_floor')
    supports = [(name, np.array(xyz), np.array(size)) for name, xyz, size in CARGO_PARTS if name.startswith('cargo_support_')]
    assert len(supports) == 4
    for name, xyz, size in supports:
        assert np.all(xyz-size/2 < center+half) and np.all(xyz+size/2 > center-half)
        assert xyz[2]+size[2]/2 == pytest.approx(floor[0][2]-floor[1][2]/2)
        assert np.all(np.abs(xyz[:2]-floor[0][:2])+size[:2]/2 <= floor[1][:2]/2)
        visual = sdf.find(f"visual[@name='{name}_visual']")
        physics = sdf.find(f"collision[@name='{name}_collision']")
        planner = urdf.find(f"collision[@name='{name}']")
        assert np.allclose(np.fromstring(visual.findtext('pose'), sep=' ')[:3], xyz)
        assert visual.findtext('geometry/box/size') == physics.findtext('geometry/box/size')
        assert np.allclose(np.fromstring(planner.find('origin').get('xyz'), sep=' '), xyz)
