"""Build the HIL scene from the existing orchard, Car0 and textured fruit assets."""
import argparse
import json
import math
from pathlib import Path
import copy
import shutil
import xml.etree.ElementTree as ET

import yaml

from .fruit_geometry import stem_dimensions
from .rtk_visual import RTK_LOWERING_M

SPAWN = [2., 36., .1325, 0.]
PARK = [30.57, 31.83, math.pi]


def orchard_catalog():
    path = Path(__file__).resolve().parents[1] / 'config/orchard_fruit_sites.json'
    if not path.is_file():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('agribot_hil_apple')) / 'config/orchard_fruit_sites.json'
    return json.loads(path.read_text())


CATALOG = orchard_catalog()
SECONDARY_BRANCH = CATALOG['secondary_pickup_candidates'][0]
BRANCH_ANCHORS = (CATALOG['pickup_candidates'] + CATALOG['secondary_pickup_candidates'][:2]+
                  CATALOG['easy_pickup_candidates'][:1])
STEM_OFFSETS = [.04]*len(CATALOG['pickup_candidates'])+[.07, .07, .07]
STEM_OFFSETS[1] = .07
APPLES = [[x, y, z-offset] for (x, y, z), offset in zip(BRANCH_ANCHORS, STEM_OFFSETS)]
TREES = []  # The installed orchard mesh owns all trees; do not add substitutes.
BOUNDS = [-1., 28., 46., 39.]
ACTUATOR_TOPIC = '/hil/actuator/cmd_vel'
TRUTH_TOPIC = '/hil/ground_truth'
CARGO_PARTS = [
    ('cargo_floor', [-.06, 0., .205], [.18, .28, .01]),
    ('cargo_rear', [-.155, 0., .24], [.01, .28, .08]),
    ('cargo_front', [.035, 0., .24], [.01, .28, .08]),
    ('cargo_left', [-.06, .145, .24], [.20, .01, .08]),
    ('cargo_right', [-.06, -.145, .24], [.20, .01, .08]),
    # Four fixed brackets overlap the chassis and meet the tray's underside.
    ('cargo_support_rear_left', [-.10, .105, .155], [.024, .024, .09]),
    ('cargo_support_rear_right', [-.10, -.105, .155], [.024, .024, .09]),
    ('cargo_support_front_left', [.015, .105, .155], [.024, .024, .09]),
    ('cargo_support_front_right', [.015, -.105, .155], [.024, .024, .09])]


def text(parent, tag, value):
    child = ET.SubElement(parent, tag)
    child.text = str(value)
    return child


def write_xml(root, path):
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=False)


def resolve_meshes(root):
    from ament_index_python.packages import get_package_share_directory
    for element in root.iter('uri'):
        value = element.text or ''
        if value.startswith('package://'):
            package, relative = value[len('package://'):].split('/', 1)
            source = Path(get_package_share_directory(package)) / relative
            if not source.is_file():
                raise FileNotFoundError(source)
            element.text = source.resolve().as_uri()
        elif value.startswith('model://scout_gazebo/'):
            source = Path(get_package_share_directory('scout_gazebo')) / value[len('model://scout_gazebo/'):]
            if not source.is_file():
                raise FileNotFoundError(source)
            element.text = source.resolve().as_uri()


def material(visual, color):
    node = ET.SubElement(visual, 'material')
    text(node, 'ambient', color)
    text(node, 'diffuse', color)


def box(link, name, xyz, size, color, collision=True):
    pose = ' '.join(map(str, [*xyz, 0, 0, 0]))
    visual = ET.SubElement(link, 'visual', name=name + '_visual')
    text(visual, 'pose', pose)
    text(ET.SubElement(ET.SubElement(visual, 'geometry'), 'box'), 'size', size)
    material(visual, color)
    if collision:
        node = ET.SubElement(link, 'collision', name=name + '_collision')
        text(node, 'pose', pose)
        text(ET.SubElement(ET.SubElement(node, 'geometry'), 'box'), 'size', size)


def fruit_visuals(link, prefix, mesh, xyz=(0., 0., 0.), yaw=0., shadows=True, socket_to_branch=.04):
    stem_shape = stem_dimensions(socket_to_branch)
    visual = ET.SubElement(link, 'visual', name=prefix + '_apple')
    text(visual, 'pose', ' '.join(map(str, [*xyz, 0., 0., yaw])))
    text(visual, 'cast_shadows', str(shadows).lower())
    text(ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh'), 'uri', mesh.resolve().as_uri())
    stem = ET.SubElement(link, 'visual', name=prefix + '_stem')
    text(stem, 'pose', ' '.join(map(str, [xyz[0], xyz[1], xyz[2]+stem_shape['visual_center'], 0., 0., 0.])))
    text(stem, 'cast_shadows', str(shadows).lower())
    geometry = ET.SubElement(ET.SubElement(stem, 'geometry'), 'cylinder')
    text(geometry, 'radius', '.0018')
    text(geometry, 'length', stem_shape['visual_length'])
    material(stem, '.18 .13 .06 1')


def world(ackermann_share, directory):
    from ament_index_python.packages import get_package_share_directory
    from agribot_farm_sim.ground import prepare_orchard_ground
    original = Path(get_package_share_directory('scout_gazebo')) / 'worlds/orchard_barriers.world'
    prepared = prepare_orchard_ground(original, Path.home() / '.cache/agribot/hil_apple/source_orchard')
    root = ET.parse(prepared).getroot()
    world = root.find('world')
    resolve_meshes(root)
    package = Path(get_package_share_directory('agribot_hil_apple'))
    apple_mesh = package / 'models/ycb_apple/textured.dae'
    if not apple_mesh.is_file():
        raise FileNotFoundError(apple_mesh)
    from .apple_asset import bake_stem_up
    apple_mesh = bake_stem_up(apple_mesh, directory / 'assets/ycb_apple')
    lod = directory / 'assets/ycb_apple/background_stem_up.dae'
    shutil.copy2(package / 'models/ycb_apple/background_stem_up.dae', lod)
    background = ET.SubElement(world, 'model', name='orchard_background_apples')
    text(background, 'static', 'true')
    background_link = ET.SubElement(background, 'link', name='fruit_visuals')
    count = 0
    for tree in orchard_catalog()['trees']:
        for anchor in tree['anchors']:
            if any(math.dist(anchor, target) < .13 for target in BRANCH_ANCHORS):
                continue
            xyz = [anchor[0], anchor[1], anchor[2]-.04]
            fruit_visuals(background_link, f'background_{count}', lod, xyz,
                          yaw=(count*2.3999632297) % math.tau, shadows=False)
            count += 1
    for index, xyz in enumerate(APPLES):
        stem_shape = stem_dimensions(STEM_OFFSETS[index])
        apple = ET.SubElement(world, 'model', name=f'harvest_apple_{index + 1}')
        text(apple, 'static', 'false')
        text(apple, 'pose', ' '.join(map(str, [*xyz, 0, 0, 0])))
        link = ET.SubElement(apple, 'link', name='fruit')
        text(link, 'gravity', 'false')  # Enabled after its real tree fixture is attached.
        inertial = ET.SubElement(link, 'inertial')
        text(inertial, 'pose', '0 0 -.032 0 0 0')
        text(inertial, 'mass', '.15')
        inertia = ET.SubElement(inertial, 'inertia')
        for axis in ('ixx', 'iyy', 'izz'):
            text(inertia, axis, '.0000735')
        fruit_visuals(link, 'scanned', apple_mesh, socket_to_branch=STEM_OFFSETS[index])
        body = ET.SubElement(link, 'collision', name='fruit_body')
        text(body, 'pose', '0 0 -.032 0 0 0')
        text(ET.SubElement(ET.SubElement(body, 'geometry'), 'sphere'), 'radius', '.035')
        stem = ET.SubElement(link, 'collision', name='grasp_stem')
        text(stem, 'pose', f"0 0 {stem_shape['collision_center']} 0 0 0")
        geometry = ET.SubElement(ET.SubElement(stem, 'geometry'), 'cylinder')
        text(geometry, 'radius', '.0018')
        text(geometry, 'length', stem_shape['collision_length'])
    attachment = ET.SubElement(world, 'plugin', name='hil_fruit_attachments', filename='libgazebo_link_attacher.so')
    text(attachment, 'enable_gravity_on_attach', 'true')
    plugin = ET.SubElement(world, 'plugin', name='hil_gazebo_state', filename='libgazebo_ros_state.so')
    ros = ET.SubElement(plugin, 'ros')
    text(ros, 'namespace', '/gazebo')
    text(ros, 'remapping', '~/model_states:=/hil/gazebo/model_states')
    text(ros, 'remapping', '~/link_states:=/hil/gazebo/link_states')
    text(plugin, 'update_rate', '30')
    return root


def vehicle_model(generated, rtk_mesh=None):
    root = ET.parse(generated / 'models/ackermann_current.sdf').getroot()
    model = root.find('model')
    model.set('name', 'hil_ackermann')
    if rtk_mesh is not None:
        visual = model.find("link[@name='base_link']/visual[@name='rtk_master_cad_visual']")
        visual.find('geometry/mesh/uri').text = Path(rtk_mesh).resolve().as_uri()
        pose = list(map(float, visual.findtext('pose').split()))
        pose[2] -= RTK_LOWERING_M
        visual.find('pose').text = ' '.join(map(str, pose))
    drive = model.find("plugin[@name='ackermann_drive']")
    drive.find('cmd_vel_topic').text = ACTUATOR_TOPIC
    drive.find('odom_topic').text = TRUTH_TOPIC
    drive.find('odom_frame').text = 'world'
    drive.find('publish_tf').text = 'false'
    drive.find('command_timeout').text = '.25'
    lidar = model.find(".//sensor[@name='lslidar_c16_points']")
    lidar.find('ray/scan/horizontal/samples').text = '720'
    lidar.find('ray/range/max').text = '25'
    lidar.find('plugin/max_range').text = '25'
    camera = model.find(".//sensor[@name='hikrobot_right_camera']")
    camera.find('camera/image/width').text = '640'
    camera.find('camera/image/height').text = '512'
    camera.find('plugin/camera_info_url').text = (
        'package://agribot_ackermann_mppi/config/hikrobot_camera_640_sim.yaml')
    if model.find("link[@name='manipulator_link_base']") is None:
        raise ValueError('Current Car0 export must contain its configured Lite6 arm')
    for index in (1, 2):
        link = model.find(f"link[@name='manipulator_finger{index}']")
        collision = copy.deepcopy(link.find('visual'))
        collision.tag = 'collision'
        collision.set('name', 'finger_pad')
        for material_ in collision.findall('material'):
            collision.remove(material_)
        contact = ET.SubElement(ET.SubElement(ET.SubElement(collision, 'surface'), 'contact'), 'ode')
        text(contact, 'kp', '1000')
        text(contact, 'kd', '1')
        link.append(collision)
        # Official G1 STL inner faces are at +/-11.2 mm. Small soft finger
        # inserts bring the usable aperture to 0..17.8 mm without scaling it.
        box(link, 'stem_soft_pad', [0., .0056 if index == 1 else -.0056, .018],
            '.018 .0112 .018', '.12 .14 .13 1')
        pad = link.find("collision[@name='stem_soft_pad_collision']")
        soft = ET.SubElement(ET.SubElement(ET.SubElement(pad, 'surface'), 'contact'), 'ode')
        text(soft, 'kp', '1000')
        text(soft, 'kd', '1')
        text(soft, 'max_vel', '.02')
        sensor = ET.SubElement(link, 'sensor', name=f'finger{index}_contact', type='contact')
        text(sensor, 'always_on', 'true')
        text(sensor, 'update_rate', '50')
        text(ET.SubElement(sensor, 'contact'), 'collision', 'finger_pad')
        text(sensor.find('contact'), 'collision', 'stem_soft_pad_collision')
        plugin = ET.SubElement(sensor, 'plugin', name=f'finger{index}_contacts', filename='libgazebo_ros_bumper.so')
        text(ET.SubElement(plugin, 'ros'), 'remapping', f'bumper_states:=/hil/gripper/finger{index}/contacts')
    for name, xyz, size in CARGO_PARTS:
        box(model.find("link[@name='base_link']"), name, xyz, ' '.join(map(str, size)), '.20 .28 .32 1')
    ET.SubElement(model, 'plugin', name='hil_gripper_transit_hold', filename='libagribot_hil_gripper_hold.so')
    resolve_meshes(root)
    return root


def vehicle_urdf(source, rtk_mesh=None):
    model = ET.parse(source).getroot()
    if rtk_mesh is not None:
        model.find("link[@name='rtk_link']/visual/geometry/mesh").set('filename', Path(rtk_mesh).resolve().as_uri())
        for name in ('rtk_master_mount_joint', 'rtk_secondary_mount_joint'):
            origin = model.find(f"joint[@name='{name}']/origin")
            xyz = list(map(float, origin.get('xyz').split()))
            xyz[2] -= RTK_LOWERING_M
            origin.set('xyz', ' '.join(map(str, xyz)))
    base = model.find("link[@name='base_link']")
    for name, xyz, size in CARGO_PARTS:
        collision = ET.SubElement(base, 'collision', name=name)
        ET.SubElement(collision, 'origin', xyz=' '.join(map(str, xyz)), rpy='0 0 0')
        ET.SubElement(ET.SubElement(collision, 'geometry'), 'box', size=' '.join(map(str, size)))
    for index in (1, 2):
        link = model.find(f"link[@name='manipulator_finger{index}']")
        collision = ET.SubElement(link, 'collision', name='stem_soft_pad')
        ET.SubElement(collision, 'origin', xyz=f'0 {0.0056 if index == 1 else -0.0056} .018', rpy='0 0 0')
        ET.SubElement(ET.SubElement(collision, 'geometry'), 'box', size='.018 .0112 .018')
    return ET.tostring(model, encoding='unicode')


def save_map(directory):
    from PIL import Image
    from ament_index_python.packages import get_package_share_directory
    source = Path(get_package_share_directory('scout_navigation')) / 'maps/orchard_v2_map6.yaml'
    config = yaml.safe_load(source.read_text())
    original = Image.open(source.parent / config['image'])
    resolution = config['resolution']
    left, bottom, right, top = BOUNDS
    ox, oy, _ = config['origin']
    x0, y0 = round((left-ox)/resolution), round(original.height-(top-oy)/resolution)
    image = original.crop((x0, y0, x0 + round((right-left)/resolution),
                           y0 + round((top-bottom)/resolution)))
    image.save(directory / 'map.pgm')
    config.update(image='map.pgm', mode='trinary',
                  origin=[ox+x0*resolution, oy+(original.height-y0-image.height)*resolution, 0.])
    (directory / 'map.yaml').write_text(yaml.safe_dump(config, sort_keys=False))


def parameters(generated, ackermann_share, directory):
    def load(path):
        return yaml.safe_load(path.read_text())

    def save(name, value):
        (directory / name).write_text(yaml.safe_dump(value, sort_keys=False))

    physical = generated / 'config/physical'
    config = load(physical / 'agribot_c16_astra.yaml')
    values = config['/**']['ros__parameters']
    values['use_sim_time'] = True
    values['common']['lid_topic'] = '/configured/localization_points'
    values['common']['img_topic'] = '/hil/camera/decoded'
    values['time_offset']['img_time_offset'] = 0.
    values['lio']['voxel_size'] = 1.
    values['vio']['img_point_cov'] = 800
    save('fastlivo.yaml', config)
    save('camera.yaml', load(ackermann_share / 'config/fastlivo_hikrobot_640_sim.yaml'))
    config = load(physical / 'fastlivo_bridge.yaml')
    config['fastlio_odom_bridge']['ros__parameters'].update(
        use_sim_time=True, is_simulation=True, twist_from_pose=True)
    save('odom_bridge.yaml', config)
    config = load(physical / 'fastlivo_rtk_fusion.yaml')
    fusion = config['fastlivo_rtk_fusion']['ros__parameters']
    fusion.update(use_sim_time=True, auto_initialize_from_fixed_rtk=True,
                  fix_topic='/navsat/fix', fused_pose_topic='/amcl_pose',
                  initial_base_z_m=SPAWN[2], initial_map_from_odom_yaw_rad=SPAWN[3])
    original_antenna = fusion['base_to_antenna_xyz']
    fusion['base_to_antenna_xyz'] = [*original_antenna[:2], original_antenna[2]-RTK_LOWERING_M]
    save('fusion.yaml', config)
    antenna = fusion['base_to_antenna_xyz']
    georef = load(ackermann_share / 'config/orchard_ackermann_sim_georeference.yaml')
    georef['map']['id'] = 'hil_apple_orchard'
    c, s = math.cos(SPAWN[3]), math.sin(SPAWN[3])
    georef['map_from_enu']['xyz'] = [SPAWN[0]+c*antenna[0]-s*antenna[1],
                                    SPAWN[1]+s*antenna[0]+c*antenna[1], SPAWN[2]+antenna[2]]
    save('georeference.yaml', georef)
    config = load(generated / 'config/simulation/nav2_params_ackermann_fastlio_static.yaml')
    vehicle = json.loads((generated / 'vehicle_config.json').read_text())
    body = vehicle['geometry']['bodyExtents']
    # The generic export has 10-15 cm agricultural clearance. Use the actual
    # Car0 chassis plus 2 cm here; include the small rear fruit tray as well.
    front = body['frontM']+.02
    rear = max(body['rearM'], .16)+.02
    left, right = body['leftM']+.02, body['rightM']+.02
    chassis_footprint = [[front, left], [front, -right], [-rear, -right], [-rear, left]]
    for value in config.values():
        if 'ros__parameters' in value:
            value['ros__parameters']['use_sim_time'] = True
    controller = config['controller_server']['ros__parameters']
    controller['odom_topic'] = '/wheel/odometry'
    return_tolerance = vehicle['geometry']['wheelbaseM']/2.
    controller['goal_checker'].update(xy_goal_tolerance=return_tolerance,
                                      yaw_goal_tolerance=math.pi, stateful=False)
    # The orchard demonstration is a forward circuit, not a reverse return.
    controller['FollowPath'].update(vx_min=0., enforce_path_inversion=False, time_steps=80,
                                  inversion_xy_tolerance=.05, inversion_yaw_tolerance=math.pi)
    controller['FollowPath']['PreferForwardCritic']['enabled'] = True
    controller['FollowPath']['GoalAngleCritic']['enabled'] = False
    for critic in ('PathAlignCritic', 'PathFollowCritic'):
        controller['FollowPath'][critic]['threshold_to_consider'] = .03
    controller['FollowPath']['PathAlignCritic']['use_path_orientations'] = False
    controller['FollowPath']['PathAngleCritic']['forward_preference'] = True
    planner = config['planner_server']['ros__parameters']['GridBased']
    planner.update(motion_model_for_search='DUBIN', max_planning_time=10.,
                   downsample_costmap=False, tolerance=.08,
                   reverse_penalty=1.05, change_penalty=2.)
    for name in ('global_costmap', 'local_costmap'):
        costmap = config[name][name]['ros__parameters']
        costmap['use_sim_time'] = True
        costmap['footprint'] = json.dumps(chassis_footprint)
        marking = costmap['stvl_layer']['lidar_mark']
        marking.update(topic='/configured/obstacle_points',
                       min_obstacle_height=-100., max_obstacle_height=100.)
        costmap['inflation_layer'].update(inflation_radius=.8, cost_scaling_factor=3.)
    save('nav2.yaml', config)
    chassis = load(physical / 'chassis_can.yaml')
    chassis['/**']['ros__parameters'].update(
        use_sim_time=True, can_transport='zqwl_cdc', command_topic='/hil/navigation/cmd_vel',
        require_feedback_before_motion=True, require_localization_ready=True,
        localization_ready_topic='/fastlivo_rtk/ready', localization_ready_timeout_sec=2.5)
    save('chassis.yaml', chassis)
    route_source = ackermann_share / 'config/orchard_waypoints_ackermann_smac.yaml'
    fixed = load(route_source)['waypoints'][:3]
    route = [[float(p['x']), float(p['y']), float(p['yaw'])] for p in fixed]
    if len(route) != 3:
        raise ValueError('The orchard route requires the original first three waypoints')
    vehicle = json.loads((generated / 'vehicle_config.json').read_text())
    manifest = {'schema_version': 1, 'scenario': 'hil_apple_orchard',
                'source_scenario': 'orchard', 'source_world': 'scout_gazebo/worlds/orchard_barriers.world',
                'source_map': 'scout_navigation/maps/orchard_v2_map6.yaml',
                'fruit_row': 2, 'fruit_asset': 'YCB 013_apple/google_16k',
                'background_tree_components': len(orchard_catalog()['trees']),
                'background_apples_before_task_deduplication': sum(len(t['anchors']) for t in orchard_catalog()['trees']),
                'branch_anchors': BRANCH_ANCHORS,
                'fruit_socket_to_branch_m': dict(zip(
                    [f'harvest_apple_{i+1}' for i in range(len(APPLES))], STEM_OFFSETS)),
                'apple_reference': 'stem_socket',
                'grasp_targets': [[x, y, z + .020] for x, y, z in APPLES],
                'grasp_part': 'stem', 'stem_diameter_m': .0036,
                'selection_policy': 'collision-free IK and OMPL from the parked vehicle; skip unreachable fruit',
                'maximum_harvest_count': 3,
                'gripper_max_aperture_m': .0178,
                'gripper_aperture_includes_custom_stem_pads': True,
                'fixed_route_source': 'agribot_ackermann_mppi/config/orchard_waypoints_ackermann_smac.yaml',
                'fixed_route_points': route, 'fixed_route_order': [1, 2, 3],
                'forward_only': True,
                'outbound_via': [route[0], route[1],
                                 [PARK[0]+1.5, PARK[1], PARK[2]], PARK],
                'pickup_stops': [
                    {'park': PARK, 'target_models': ['harvest_apple_5', 'harvest_apple_7']}],
                'pickup_position_tolerance_m': .10,
                'pickup_yaw_tolerance_rad': .15,
                'pickup_orientation_required': True,
                'return_position_tolerance_m': return_tolerance,
                'final_orientation_required': False,
                'control_velocity_topic': '/wheel/odometry',
                'completion_via': route[2:],
                'finish': route[-1], 'finish_is_spawn': False,
                'route_sequence': [1, 2, 3],
                'transit_gripper_opening_m': .0089,
                'navigation_phases': ['outbound', 'completion'],
                'spawn': SPAWN, 'park': PARK, 'apples': APPLES, 'trees': TREES,
                'base_to_antenna': antenna, 'rtk_antenna_lowering_m': RTK_LOWERING_M,
                'manipulator': vehicle['manipulators'][0],
                'minimum_turning_radius_m': vehicle['geometry']['minTurningRadiusM'],
                'can_topology': 'hpm6750_native_bridge', 'hpm6750_enabled': True,
                'can_command_topic': ACTUATOR_TOPIC,
                'can_required_for_motion': True, 'ground_truth_topic': TRUTH_TOPIC,
                'status': 'generated_not_a_full_run_acceptance'}
    (directory / 'mission.json').write_text(json.dumps(manifest, indent=2) + '\n')
    (directory / 'arm_environment.json').write_text(json.dumps(
        orchard_catalog()['manipulation_environment_mesh']) + '\n')


def arm_parameters(urdf, directory):
    """Describe the existing mounted Lite6 for MoveIt's KDL/OMPL plugins."""
    model = ET.fromstring(urdf)
    ET.SubElement(model, 'link', name='manipulator_tcp')
    joint = ET.SubElement(model, 'joint', name='manipulator_tcp_mount', type='fixed')
    ET.SubElement(joint, 'parent', link='manipulator_gripper_shell')
    ET.SubElement(joint, 'child', link='manipulator_tcp')
    ET.SubElement(joint, 'origin', xyz='0 0 .071', rpy='0 0 0')
    from ament_index_python.packages import get_package_share_directory
    source_meshes = Path(get_package_share_directory('agribot_manipulator_sim')) / 'models/lite6/meshes'
    meshes = directory / 'assets/lite6'
    meshes.mkdir(parents=True, exist_ok=True)
    remote_meshes = Path('/home/sunrise/agribot_hil_ws/scenario/assets/lite6')
    # Keep real arm collision meshes; rendering visuals are not needed on X5.
    for link in model.findall('link'):
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if link.attrib['name'].startswith('manipulator_') and mesh is not None:
                filename = Path(mesh.attrib['filename']).name
                shutil.copy2(source_meshes / filename, meshes / filename)
                collision = copy.deepcopy(visual)
                collision.tag = 'collision'
                collision.find('geometry/mesh').set('filename', (remote_meshes / filename).as_uri())
                link.append(collision)
            link.remove(visual)
    semantic = ET.Element('robot', name=model.attrib['name'])
    group = ET.SubElement(semantic, 'group', name='arm')
    ET.SubElement(group, 'chain', base_link='base_link', tip_link='manipulator_tcp')
    for joint in model.findall('joint'):
        ET.SubElement(semantic, 'disable_collisions',
                      link1=joint.find('parent').attrib['link'],
                      link2=joint.find('child').attrib['link'], reason='Adjacent')
    # Non-adjacent never-colliding pairs from xArm's official Lite6 SRDF.
    for first, second in [('link1', 'link3'), ('link2', 'link4'), ('link2', 'link_base'),
                          ('link3', 'link5'), ('link3', 'link6'), ('link4', 'link6'),
                          ('finger1', 'finger2')]:
        ET.SubElement(semantic, 'disable_collisions', link1='manipulator_' + first,
                      link2='manipulator_' + second, reason='Never')
    for tool in ('gripper_shell', 'finger1', 'finger2'):
        for arm in ('link4', 'link5', 'link6'):
            ET.SubElement(semantic, 'disable_collisions', link1='manipulator_' + tool,
                          link2='manipulator_' + arm, reason='Never')
    params = {'use_sim_time': True, 'robot_description': ET.tostring(model, encoding='unicode'),
              'robot_description_semantic': ET.tostring(semantic, encoding='unicode'),
              'robot_description_kinematics': {'arm': {
                  'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                  'kinematics_solver_timeout': .1, 'position_only_ik': False}},
              'robot_description_planning': {'joint_limits': {
                  f'manipulator_joint{i}': {'has_velocity_limits': True, 'max_velocity': .9,
                    'has_acceleration_limits': True, 'max_acceleration': 1.5}
                  for i in range(1, 7)}},
              'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
              'ompl': {'planning_plugin': 'ompl_interface/OMPLPlanner',
                       'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
                       'start_state_max_bounds_error': .1,
                       'planner_configs': {'RRTConnectkConfigDefault': {'type': 'geometric::RRTConnect', 'range': .15}},
                       'arm': {'planner_configs': ['RRTConnectkConfigDefault'], 'longest_valid_segment_fraction': .001}},
              'allow_trajectory_execution': False,
              'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
              'moveit_simple_controller_manager': {
                  'controller_names': ['hil_arm_controller'],
                  'hil_arm_controller': {'type': 'FollowJointTrajectory',
                      'action_ns': 'follow_joint_trajectory', 'default': True,
                      'joints': [f'manipulator_joint{i}' for i in range(1, 7)]}},
              'publish_robot_description_semantic': True,
              'planning_scene_monitor_options': {'joint_state_topic': '/joint_states'}}
    (directory / 'arm_moveit.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}, sort_keys=False))


def build(directory):
    from ament_index_python.packages import get_package_share_directory
    directory = Path(directory).expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=True)
    rdk = directory / 'rdk'
    rdk.mkdir(exist_ok=True)
    description = Path(get_package_share_directory('agribot_vehicle_description'))
    ackermann = Path(get_package_share_directory('agribot_ackermann_mppi'))
    generated = description / 'generated/ackermann_current'
    from .rtk_visual import prepare_rtk_visual
    rtk_mesh = prepare_rtk_visual(generated, directory)
    write_xml(world(ackermann, directory), directory / 'world.sdf')
    write_xml(vehicle_model(generated, rtk_mesh), directory / 'vehicle.sdf')
    urdf = generated / 'urdf/ackermann_current.urdf.xacro'
    urdf_text = vehicle_urdf(urdf, rtk_mesh)
    (directory / 'vehicle.urdf').write_text(urdf_text)
    arm_parameters(urdf_text, rdk)
    save_map(rdk)
    parameters(generated, ackermann, rdk)
    return directory


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path.home() / '.cache/agribot/hil_apple')
    args = parser.parse_args()
    print(build(args.output))


if __name__ == '__main__':
    main()
