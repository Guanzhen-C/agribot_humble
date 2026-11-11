"""Prepare isolated X5 navigation parameters using the existing vehicle profile generator."""
import argparse
import copy
import importlib.util
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import yaml


def write(root, name, data):
    (root/name).write_text(yaml.safe_dump(data, sort_keys=False))


def main(args):
    output = args.scene/'rdk'
    if output.exists():
        raise FileExistsError(output)
    output.mkdir()
    generated = args.description/'generated/car4_harrow'
    vehicle = json.loads((generated/'vehicle_config.json').read_text())
    spec = importlib.util.spec_from_file_location('dedicated_hitch_profile_generator',
        args.description/'launch/configured_ackermann_sim.launch.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    config = args.description/'config'
    path = module._write_nav2_profile_config(
        str(generated/'config/simulation/nav2_params_ackermann_navsat_static.yaml'),
        str(config/'rpp_controller.yaml'), str(config/'dwb_controller.yaml'),
        'hitch_hil_navsat', 'stvl', 'mppi', 'smac_hybrid', True,
        module._body_navigation_footprint(vehicle), module._final_position_tolerance(vehicle))
    nav = yaml.safe_load(Path(path).read_text())

    def sim_time(value):
        if not isinstance(value, dict):
            return
        if 'ros__parameters' in value:
            value['ros__parameters']['use_sim_time'] = True
        for item in value.values():
            sim_time(item)
    sim_time(nav)
    controller = nav['controller_server']['ros__parameters']
    controller['goal_checker_plugins'] = ['goal_checker', 'work_goal_checker']
    controller['work_goal_checker'] = copy.deepcopy(controller['goal_checker'])
    controller['work_goal_checker']['xy_goal_tolerance'] = .4
    write(output, 'nav2.yaml', nav)
    ekf = yaml.safe_load((generated/'config/physical/kf_gins_n300pro.yaml').read_text())
    params = ekf['rtk_eskf_localization']['ros__parameters']
    params.update(use_sim_time=True, pose_topic='/navsat/fix', pose_message_type='navsat_fix',
                  auto_reference_from_first_navsat_fix=False, align_measurement_to_initial_pose=True,
                  reference_lat_deg=30.5, reference_lon_deg=114., reference_alt_m=20.,
                  initial_pose_x=-20., initial_pose_y=0., initial_pose_z=.74, initial_pose_yaw=0.)
    write(output, 'kf_gins.yaml', ekf)
    for name in ('map.pgm', 'map.yaml'):
        shutil.copy2(args.scene/name, output/name)
    write(output, 'gnss_sim.yaml', {'sim_navsat_bridge': {'ros__parameters': dict(
        use_sim_time=True, odom_topic='/hil_hitch/ground_truth',
        ground_truth_topic='/hil_hitch/evaluation_truth', origin_x=-20., origin_y=0., origin_z=.74,
        origin_yaw=0., reference_lat=30.5, reference_lon=114., reference_alt=20.,
        base_to_antenna_m=[a+b for a, b in zip(params['base_to_imu_m'], params['antlever_m'])],
        publish_imu=False, fix_rate_hz=10., heading_rate_hz=10.)}})
    robot = ET.parse(generated/'urdf/car4_harrow.urdf.xacro')
    root = robot.getroot(); root.set('name', 'hil_hitch_tractor')
    for item in list(root):
        if item.get('name', '').startswith('mounted_disc_harrow') or item.get('reference', '').startswith('mounted_disc_harrow'):
            root.remove(item)
    sdf_model = ET.parse(args.scene/'vehicle.sdf').find('model')
    root.find("link[@name='base_link']/visual[@name='vehicle_body']/geometry/mesh").set(
        'filename', sdf_model.findtext("link[@name='base_link']/visual[@name='vehicle_body_visual']/geometry/mesh/uri"))
    for link in sdf_model.findall('link'):
        name = link.get('name')
        if not name.startswith('hitch_'):
            continue
        new = ET.SubElement(root, 'link', name=name)
        for visual in link.findall('visual'):
            uri = visual.findtext('geometry/mesh/uri')
            if uri:
                item = ET.SubElement(new, 'visual', name=visual.get('name'))
                values = (visual.findtext('pose') or '0 0 0 0 0 0').split()
                ET.SubElement(item, 'origin', xyz=' '.join(values[:3]), rpy=' '.join(values[3:]))
                ET.SubElement(ET.SubElement(item, 'geometry'), 'mesh', filename=uri)
    pivot = list(map(float, sdf_model.find("link[@name='hitch_tool_frame']/pose").text.split()[:3]))
    for joint in sdf_model.findall('joint'):
        name = joint.get('name')
        if not name.startswith('hitch_'):
            continue
        new = ET.SubElement(root, 'joint', name=name, type='revolute' if name == 'hitch_lift_joint' else 'continuous')
        ET.SubElement(new, 'parent', link=joint.findtext('parent'))
        child = joint.findtext('child'); ET.SubElement(new, 'child', link=child)
        child_pose = sdf_model.find(f"link[@name='{child}']/pose").text.split()
        xyz = list(map(float, child_pose[:3]))
        if name != 'hitch_lift_joint':
            xyz = [a-b for a, b in zip(xyz, pivot)]
        ET.SubElement(new, 'origin', xyz=' '.join(map(str, xyz)), rpy=' '.join(child_pose[3:]))
        ET.SubElement(new, 'axis', xyz=joint.findtext('axis/xyz'))
        if name == 'hitch_lift_joint':
            limits = joint.find('axis/limit')
            ET.SubElement(new, 'limit', **{key: limits.findtext(key)
                for key in ('lower', 'upper', 'effort', 'velocity')})
    ET.indent(robot, space='  '); robot.write(args.scene/'vehicle.urdf', encoding='utf-8')
    (output/'mission.json').write_text(json.dumps(dict(
        vehicle='Car4', implement='mounted_disc_harrow', protocol='hitch-hil-0x21',
        algorithm='STVL + NavSat ESKF + Smac Hybrid-A* + MPPI Ackermann',
        goals=[dict(name='approach', x=-16., y=0., yaw=0., height=9000),
               dict(name='work_row_1', x=14., y=0., yaw=0., height=0),
               dict(name='headland_turn', x=14., y=8., yaw=3.141592653589793, height=9000),
               dict(name='work_row_2', x=-16., y=8., yaw=3.141592653589793, height=0),
               dict(name='park', x=-20., y=8., yaw=3.141592653589793, height=9000)],
        final_tolerance_m=module._final_position_tolerance(vehicle)), indent=2)+'\n')
    print(json.dumps(dict(output=str(output), motion_model=controller['FollowPath']['motion_model'],
                          turning_radius=controller['FollowPath']['AckermannConstraints']['min_turning_r'],
                          final_tolerance_m=controller['goal_checker']['xy_goal_tolerance'])))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--description', type=Path, required=True)
    main(parser.parse_args())
