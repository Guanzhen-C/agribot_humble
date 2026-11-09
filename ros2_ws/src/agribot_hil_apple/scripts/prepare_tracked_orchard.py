"""Prepare the existing orchard and a separate Car3 ISOBUS spraying scenario."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory as share
from agribot_hil_apple.rtk_visual import prepare_antenna_heads
from agribot_hil_apple.appearance import prepare_grey_chassis
from agribot_vehicle_config_tools.model import effective_sensor_pose, fastlivo_extrinsics

from prepare_tracked_work_scene import prepare


DECK_Z_M = .42665
ANTENNA_MASTER = [.44, .29, DECK_Z_M+.10]
ANTENNA_SECONDARY = [.44, -.29, DECK_Z_M+.10]
# Clear chassis deck ahead of the implement and behind the sensor pedestal.
# The exported chassis surface is z=.426081; the IMU box is 20 mm high.
IMU_SUPPORT_Z_M = .426081
IMU_XYZ = [.31, 0., IMU_SUPPORT_Z_M+.010]
# Outer circular edges of the 0.4 mm bore in STEP solids 66/68/70/72/74/76/78/80,
# face 0 (duplicated by face 106). The axial fasteners 82/86/.../106 are NOT nozzles.
# Positions remain in source millimetres; no CAD geometry is added or replaced.
CAD_ORIGIN_MM = [430.2674591044463, 143.8545493858059, -463.87632204641795]
CAD_NOZZLE_OUTLETS_MM = [
    [688.1285929996077, 662.8093780597421, -351.75996901717525],
    [864.1461771361314, 567.8472273114481, -351.7599690171724],
    [516.1499828532259, 755.592487101482, -273.9718741380096],
    [416.2608853271534, 809.4830320939902, -107.75917678466425],
    [857.4008193457199, 571.4863732893144, 348.66892848259397],
    [675.1409533719618, 669.8162586406523, 342.64358573424084],
    [512.8287885442677, 757.3842839618854, 269.8474607227398],
    [415.934513585286, 809.6591108801136, 106.99385094642543]]
SPRAY_NOZZLE_COUNT = len(CAD_NOZZLE_OUTLETS_MM)
# Measured from the supplied STEP's 350 mm fan cylinder, not its oblique bbox.
FAN_AXIS = (.4748107537414699, .8800879206826198, 0.)
CAD_FAN_CENTRE_MM = [741.2203022555734, 705.7499577707322, 0.]
NOZZLE_CLEARANCE_MM = 1.
MIST_VISUAL = dict(cone_angle=.18, total_emission_rate=2400,
                   velocity_min=2., velocity_max=2.8,
                   lifetime_min=.9, lifetime_max=1.3, growth_rate=.10)
MIST_COLOUR = [.36, .60, .78, .60]
MOUNT_NORMAL = (FAN_AXIS[1], -FAN_AXIS[0], 0.)
# The STEP's common outer mounting faces (solids 29/30/43/44) are normal to
# MOUNT_NORMAL, not CAD Z. Map this outward normal down and the fan axis rearward.
SPRAYER_RPY = [math.pi/2, math.atan2(FAN_AXIS[1], FAN_AXIS[0]), math.pi]
SPRAYER_SCALE = .55
ALIGNED_CAD_MIN = [-.6540611774958374, .0032763159833848476, -.3418718933190567]
ALIGNED_CAD_MAX = [.6448389044556374, .8738763332366943, .451128172746401]
SPRAYER_POSE = [
    -.10-SPRAYER_SCALE*(ALIGNED_CAD_MIN[0]+ALIGNED_CAD_MAX[0])/2,
    -SPRAYER_SCALE*(ALIGNED_CAD_MIN[1]+ALIGNED_CAD_MAX[1])/2,
    DECK_Z_M-SPRAYER_SCALE*ALIGNED_CAD_MIN[2], *SPRAYER_RPY]


def coordinates(values):
    return ' '.join(f'{value:.12g}' for value in values)


def compact_installation(model, generated, output):
    base = model.find("link[@name='base_link']")
    for visual in list(base.findall('visual')):
        if visual.get('name', '').startswith(('work_hitch_rail_', 'work_mounting_cradle')):
            base.remove(visual)
    plant = model.find("link[@name='work_plant_frame']")
    plant.find('pose').text = coordinates(SPRAYER_POSE)
    mesh = plant.find("visual[@name='supplied_plant_cad']/geometry/mesh")
    ET.SubElement(mesh, 'scale').text = coordinates([SPRAYER_SCALE]*3)
    # Nozzle locations follow the scaled CAD fan mouth, in plant-local axes.
    spray_anchor = plant.find("visual[@name='spray_nozzles']")
    spray_anchor.find('geometry/sphere/radius').text = '.00001'
    spray = spray_anchor.find('plugin')
    for key, value in MIST_VISUAL.items():
        ET.SubElement(spray, key).text = str(value)
    ET.SubElement(spray, 'particle_colour').text = coordinates(MIST_COLOUR)
    ET.SubElement(spray, 'obstacle_clipping').text = 'true'
    obstacles = ET.SubElement(model, 'plugin', name='work_spray_obstacles',
                              filename='libagribot_hil_work_spray_obstacles.so')
    ET.SubElement(obstacles, 'link_name').text = 'work_plant_frame'
    for index, source_point in enumerate(CAD_NOZZLE_OUTLETS_MM):
        radial = [source_point[i]-CAD_FAN_CENTRE_MM[i] for i in range(3)]
        projection = sum(radial[i]*FAN_AXIS[i] for i in range(3))
        radial = [radial[i]-projection*FAN_AXIS[i] for i in range(3)]
        radius = math.sqrt(sum(value*value for value in radial))
        direction = [value/radius for value in radial]
        xyz = [SPRAYER_SCALE*.001*(source_point[i]-CAD_ORIGIN_MM[i]
               +NOZZLE_CLEARANCE_MM*direction[i]) for i in range(3)]
        name = f'nozzle_{index:02d}'
        emitter = ET.SubElement(spray, 'nozzle', name=name)
        ET.SubElement(emitter, 'position').text = coordinates(xyz)
        ET.SubElement(emitter, 'direction').text = coordinates(direction)
        obstacles.append(copy.deepcopy(emitter))
    for index, name in enumerate(('fan', 'pump')):
        model.find(f"link[@name='work_{name}_rotor']/pose").text = coordinates(
            [-.20+index*.30, 0., DECK_Z_M+.22, 0., 0., 0.])
    heads = prepare_antenna_heads(generated, output,
                                 baseline_m=ANTENNA_MASTER[1]-ANTENNA_SECONDARY[1],
                                 retain_short_stems=True)
    rtk = base.find("visual[@name='rtk_master_cad_visual']")
    rtk.find('pose').text = coordinates(ANTENNA_MASTER+[0., 0., 0.])
    rtk.find('geometry/mesh/uri').text = heads.as_uri()
    return heads


def load(path):
    return yaml.safe_load(path.read_text())


def save(root, name, values):
    (root/name).write_text(yaml.safe_dump(values, sort_keys=False))


def build(description, cad, orchard, output):
    prepare(description, cad, output)
    generated = description/'generated/unity_car3'
    body_mesh = prepare_grey_chassis(generated, output)
    textured = cad/'textured/visual_textured.dae'
    implement_mesh = textured if textured.is_file() else cad/'visual_lod.dae'
    vehicle = json.loads((generated/'vehicle_config.json').read_text())
    for sensor in vehicle['sensors']:
        if sensor['id'] in ('rtk_master', 'rtk_secondary'):
            sensor['pose'].update(xyz=(ANTENNA_MASTER if sensor['id'] == 'rtk_master'
                                      else ANTENNA_SECONDARY), rpy=[0., 0., 0.])
        elif sensor['id'] == 'imu':
            sensor['pose']['xyz'] = IMU_XYZ
    imu = next(sensor for sensor in vehicle['sensors'] if sensor['id'] == 'imu')
    imu_pose = effective_sensor_pose(imu)
    world = ET.parse(orchard/'world.sdf')
    scene = world.find('world')
    for model in scene.findall('model'):
        if model.get('name', '').startswith('harvest_apple_'):
            model.find('static').text = 'true'
    for plugin in list(scene.findall("plugin[@name='hil_fruit_attachments']")):
        scene.remove(plugin)
    gui = scene.find('gui')
    if gui is not None:
        scene.remove(gui)
    gui = ET.SubElement(scene, 'gui')
    camera = ET.SubElement(gui, 'camera', name='user_camera')
    # Initial view only, with no follow/tracking plugin. User remains in control.
    ET.SubElement(camera, 'pose').text = '5 32 3.4 0 .44 2.2143'
    ET.indent(world, space='  ')
    world.write(output/'world.sdf', encoding='utf-8')

    model_tree = ET.parse(output/'vehicle.sdf')
    model = model_tree.find('model')
    heads = compact_installation(model, generated, output)
    model.find("link/visual[@name='vehicle_body_visual']/geometry/mesh/uri").text = body_mesh.as_uri()
    model.find("link[@name='work_plant_frame']/visual[@name='supplied_plant_cad']/geometry/mesh/uri").text = implement_mesh.as_uri()
    for node in (model.find("link/visual[@name='imu_cad_visual']"),
                 model.find(".//sensor[@name='n300pro_imu']")):
        node.find('pose').text = coordinates(IMU_XYZ+imu['pose']['rpy'])
    lidar = model.find(".//sensor[@name='lslidar_c16_points']")
    lidar.find('ray/scan/horizontal/samples').text = '720'
    lidar.find('ray/range/max').text = '25'
    lidar.find('plugin/max_range').text = '25'
    camera = model.find(".//sensor[@name='hikrobot_right_camera']")
    camera.find('camera/image/width').text = '640'
    camera.find('camera/image/height').text = '512'
    camera.find('plugin/camera_info_url').text = 'package://agribot_ackermann_mppi/config/hikrobot_camera_640_sim.yaml'
    ET.indent(model_tree, space='  ')
    model_tree.write(output/'vehicle.sdf', encoding='utf-8')

    urdf = ET.parse(generated/'urdf/unity_car3.urdf.xacro')
    robot = urdf.getroot()
    robot.find("link[@name='base_link']/visual[@name='vehicle_body']/geometry/mesh").set('filename', body_mesh.as_uri())
    for child in list(robot):
        if child.get('name', '').startswith(('manipulator_', 'vehicle_manipulator_')) or \
                child.get('reference', '').startswith('manipulator_'):
            robot.remove(child)
    for name, xyz in (('imu_mount_joint', IMU_XYZ),
                      ('rtk_master_mount_joint', ANTENNA_MASTER),
                      ('rtk_secondary_mount_joint', ANTENNA_SECONDARY)):
        robot.find(f"joint[@name='{name}']/origin").set('xyz', coordinates(xyz))
    robot.find("link[@name='rtk_link']/visual/geometry/mesh").set('filename', heads.as_uri())
    plant = ET.SubElement(robot, 'link', name='work_plant_frame')
    visual = ET.SubElement(plant, 'visual')
    geometry = ET.SubElement(visual, 'geometry')
    ET.SubElement(geometry, 'mesh', filename=str(implement_mesh),
                  scale=coordinates([SPRAYER_SCALE]*3))
    joint = ET.SubElement(robot, 'joint', name='work_mount', type='fixed')
    ET.SubElement(joint, 'parent', link='base_link')
    ET.SubElement(joint, 'child', link='work_plant_frame')
    ET.SubElement(joint, 'origin', xyz=coordinates(SPRAYER_POSE[:3]),
                  rpy=coordinates(SPRAYER_POSE[3:]))
    ET.indent(urdf, space='  ')
    urdf.write(output/'vehicle.urdf', encoding='utf-8')

    rdk = output/'rdk'
    rdk.mkdir()
    for file in ('map.pgm', 'map.yaml'):
        shutil.copy2(orchard/'rdk'/file, rdk/file)
    physical = generated/'config/physical'
    fastlivo = load(physical/'agribot_c16_astra.yaml')
    values = fastlivo['/**']['ros__parameters']
    values['use_sim_time'] = True
    values['common'].update(lid_topic='/configured/localization_points', img_topic='/hil/camera/decoded')
    values['lio']['voxel_size'] = 1.
    values['vio']['img_point_cov'] = 800
    values['time_offset']['img_time_offset'] = 0.
    values['extrin_calib'] = fastlivo_extrinsics(vehicle)
    save(rdk, 'fastlivo.yaml', fastlivo)
    shutil.copy2(orchard/'rdk/camera.yaml', rdk/'camera.yaml')
    odom = load(physical/'fastlivo_bridge.yaml')
    odom['fastlio_odom_bridge']['ros__parameters'].update(
        use_sim_time=True, is_simulation=True, twist_from_pose=True,
        base_to_body_xyz=imu_pose['xyz'], base_to_body_rpy=imu_pose['rpy'])
    save(rdk, 'odom_bridge.yaml', odom)
    spawn = [2., 36., .24, 0.]
    fusion = load(physical/'fastlivo_rtk_fusion.yaml')
    f = fusion['fastlivo_rtk_fusion']['ros__parameters']
    f.update(use_sim_time=True, auto_initialize_from_fixed_rtk=True, fix_topic='/navsat/fix',
             fused_pose_topic='/amcl_pose', initial_base_z_m=spawn[2], initial_map_from_odom_yaw_rad=0.,
             base_to_antenna_xyz=ANTENNA_MASTER)
    save(rdk, 'fusion.yaml', fusion)
    antenna = f['base_to_antenna_xyz']
    georef = load(orchard/'rdk/georeference.yaml')
    georef['map']['id'] = 'hil_tracked_sprayer_orchard'
    georef['map_from_enu']['xyz'] = [spawn[i]+antenna[i] for i in range(3)]
    georef['calibration']['version'] = 'car3-orchard-sim-v6-imu-on-chassis'
    save(rdk, 'georeference.yaml', georef)

    nav = load(generated/'config/simulation/nav2_params_ackermann_fastlio_static.yaml')
    for value in nav.values():
        if 'ros__parameters' in value:
            value['ros__parameters']['use_sim_time'] = True
    controller = nav['controller_server']['ros__parameters']
    tolerance = vehicle['geometry']['wheelbaseM']/2.
    controller['odom_topic'] = '/hil/wheel/odometry'
    controller['goal_checker'].update(xy_goal_tolerance=tolerance, yaw_goal_tolerance=math.pi, stateful=False)
    follow = controller['FollowPath']
    follow.update(motion_model='DiffDrive', vx_min=0., vx_max=.30, wz_max=.60, wz_std=.25,
                  time_steps=60, enforce_path_inversion=False)
    follow['GoalAngleCritic']['enabled'] = False
    follow['PathAlignCritic'].update(use_path_orientations=False, threshold_to_consider=.03)
    follow['PathFollowCritic']['threshold_to_consider'] = .03
    follow['PathAngleCritic']['forward_preference'] = True
    nav['planner_server']['ros__parameters']['GridBased'].update(
        motion_model_for_search='DUBIN', minimum_turning_radius=.50, max_planning_time=10.,
        tolerance=.08, downsample_costmap=False, cache_obstacle_heuristic=True)
    body = vehicle['geometry']['bodyExtents']
    front, rear, left, right = (body[name]+.02 for name in ('frontM', 'rearM', 'leftM', 'rightM'))
    footprint = [[front, left], [front, -right], [-rear, -right], [-rear, left]]
    for name in ('global_costmap', 'local_costmap'):
        costmap = nav[name][name]['ros__parameters']
        costmap.update(use_sim_time=True, footprint=json.dumps(footprint), footprint_padding=0.)
        costmap['stvl_layer']['lidar_mark'].update(
            topic='/configured/obstacle_points', min_obstacle_height=-100., max_obstacle_height=100.)
        costmap['inflation_layer'].update(inflation_radius=.85, cost_scaling_factor=3.)
    save(rdk, 'nav2.yaml', nav)
    source_mission = json.loads((orchard/'rdk/mission.json').read_text())
    points = source_mission['fixed_route_points']
    manifest = dict(schema_version=1, scenario='hil_tracked_sprayer_orchard', vehicle='Car3',
        spawn=spawn, base_to_antenna=antenna, fixed_route_order=[1, 2, 3],
        fixed_route_points=points, outbound_via=points, finish=points[-1],
        finish_is_spawn=False, goal_tolerance_m=tolerance,
        world_source_sha256=hashlib.sha256((orchard/'world.sdf').read_bytes()).hexdigest(),
        map_image_sha256=hashlib.sha256((rdk/'map.pgm').read_bytes()).hexdigest(),
        can_topology='x5_mixed_isobus_hpm6750_native_pc',
        work_control='ISOBUS commands converted by implement ECU HPM6750',
        algorithms=['FAST-LIVO2 + RTK', 'Smac Hybrid-A*', 'MPPI DiffDrive'],
        spraying=dict(fan_gear=3, pump_gear=4, minimum_speed_mps=.03,
                      row_y=[36., 32.5], row_half_width_m=.65, x_range=[3., 39.5]),
        implement_collision=False,
        presentation=dict(sprayer_pose=SPRAYER_POSE, sprayer_scale=SPRAYER_SCALE,
                          deck_z_m=DECK_Z_M, rtk_master=ANTENNA_MASTER,
                          imu_xyz=IMU_XYZ, imu_support_z_m=IMU_SUPPORT_Z_M,
                          imu_mount_owner='vehicle_chassis',
                          rtk_secondary=ANTENNA_SECONDARY, fan_direction='rear (-X)',
                          layout='tank_front_fan_rear', mounting_plate_removed=True,
                          mount_normal_base=[0., 0., -1.], mounting_contact_z_m=DECK_Z_M,
                          rtk_short_stems_retained=True, rtk_stem_height_m=.062,
                          water_mist='soft particles driven by actual pump shaft speed',
                          spray_nozzles=SPRAY_NOZZLE_COUNT,
                          spray_outlets_source='STEP solids 66/68/70/72/74/76/78/80, outer 0.4 mm bore edges',
                          spray_direction='Radially outward, perpendicular to fan axis',
                          spray_visual_parameters=dict(MIST_VISUAL, colour=MIST_COLOUR),
                          chassis_finish='grey powder-coated shell, scenario-local materials',
                          implement_textures=textured.is_file(),
                          implement_visual=str(implement_mesh),
                          additional_nozzle_geometry=False,
                          limitation='Scaled simulation mount, not a calibrated real installation'))
    (rdk/'mission.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (output/'vehicle_config.json').write_text(json.dumps(vehicle, indent=2)+'\n')
    provenance = json.loads((output/'provenance.json').read_text())
    provenance.update(mount_xyz_m=SPRAYER_POSE[:3], mount_yaw_rad=SPRAYER_POSE[5],
                      mount_rpy_rad=SPRAYER_RPY,
                      visual_scale=SPRAYER_SCALE, presentation=manifest['presentation'],
                      purpose='Scaled Car3 orchard spraying HIL presentation',
                      limitations=['Illustrative 10 kg inertia and motor curves, not calibrated',
                                   'Scaled simulation mounting only; original STEP unchanged',
                                   'Motor feedback from integrated shaft physics; no fluid dynamics claim',
                                   'Complete orchard navigation route not yet accepted'])
    (output/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--description', type=Path, default=Path(share('agribot_vehicle_description')))
    parser.add_argument('--cad', type=Path, default=Path.home()/'.cache/agribot/isobus/work_models/plant_protection')
    parser.add_argument('--orchard', type=Path, default=Path.home()/'.cache/agribot/hil_apple')
    parser.add_argument('--output', type=Path, default=Path.home()/'.cache/agribot/isobus/tracked_orchard')
    args = parser.parse_args()
    print(json.dumps(build(args.description, args.cad, args.orchard, args.output), indent=2))


if __name__ == '__main__':
    main()
