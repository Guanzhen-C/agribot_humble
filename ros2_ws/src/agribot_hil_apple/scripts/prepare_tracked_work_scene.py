"""Derive an isolated Car3 spraying bench, preserving all source CAD/config."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def element(parent, tag, value=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if value is not None:
        child.text = str(value)
    return child


def inertia(link, mass, value):
    data = element(link, 'inertial')
    element(data, 'mass', mass)
    values = element(data, 'inertia')
    for key in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'):
        element(values, key, value if key in ('ixx', 'iyy', 'izz') else 0)


def prepare(description, cad, output):
    if output.exists():
        raise FileExistsError(output)
    source = description/'generated/unity_car3/models/unity_car3.sdf'
    tree = ET.parse(source)
    model = tree.find('model')
    model.set('name', 'hil_tracked_sprayer')
    config = json.loads((description/'generated/unity_car3/vehicle_config.json').read_text())
    for node in list(model):
        name = node.get('name', '')
        if name.startswith(('manipulator_', 'vehicle_manipulator_')):
            model.remove(node)
    spec = importlib.util.spec_from_file_location('hil_original_track_contact',
        description/'launch/configured_ackermann_sim.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module._configure_tracked_contact(model, config, description/'config/tracked_contact_simulation.yaml')
    for mapping in model.findall('plugin/ros/remapping'):
        if mapping.text == 'cmd_vel:=/nav2/cmd_vel':
            mapping.text = 'cmd_vel:=/hil/actuator/cmd_vel'
        if mapping.text == 'odom:=base_pose_ground_truth':
            mapping.text = 'odom:=/hil/ground_truth'
    # Rear-mounted at original metre scale. No implement collision/footprint.
    # Mount transform is a HIL presentation choice, not a measured installation.
    mount = (-1.10, 0., .22)
    base = model.find("link[@name='base_link']")
    cradle = element(base, 'visual', name='work_mounting_cradle')
    element(cradle, 'pose', '-1.10 0 .20 0 0 0')
    element(element(element(cradle, 'geometry'), 'box'), 'size', '1.26 1.50 .04')
    cradle_material = element(cradle, 'material')
    element(cradle_material, 'ambient', '.25 .28 .25 1')
    element(cradle_material, 'diffuse', '.35 .38 .35 1')
    for side in (-1, 1):
        support = element(base, 'visual', name=f'work_hitch_rail_{side}')
        element(support, 'pose', f'-.75 {side*.26} .22 0 0 0')
        element(element(element(support, 'geometry'), 'box'), 'size', '.70 .04 .04')
        material = element(support, 'material')
        element(material, 'ambient', '.35 .35 .35 1')
        element(material, 'diffuse', '.4 .4 .4 1')
    body = element(model, 'link', name='work_plant_frame')
    element(body, 'pose', f'{mount[0]} {mount[1]} {mount[2]} 0 0 3.141592653589793')
    inertia(body, 10, .6)
    visual = element(body, 'visual', name='supplied_plant_cad')
    mesh = element(element(visual, 'geometry'), 'mesh')
    element(mesh, 'uri', str(cad/'visual_lod.dae'))
    spray_visual = element(body, 'visual', name='spray_nozzles')
    element(element(element(spray_visual, 'geometry'), 'sphere'), 'radius', .012)
    element(spray_visual, 'plugin', name='work_spray', filename='libagribot_hil_work_spray_visual.so')
    joint = element(model, 'joint', name='work_mount', type='fixed')
    element(joint, 'parent', 'base_link'); element(joint, 'child', 'work_plant_frame')
    for index, name in enumerate(('fan', 'pump')):
        rotor = element(model, 'link', name=f'work_{name}_rotor')
        element(rotor, 'pose', f'{mount[0]} {(-.20 if index else .20)} .65 0 0 0')
        inertia(rotor, .05, .001)
        # Abstract shaft inertia only; supplied CAD is not spuriously rotated.
        joint = element(model, 'joint', name=f'work_{name}_joint', type='revolute')
        element(joint, 'parent', 'work_plant_frame')
        element(joint, 'child', f'work_{name}_rotor')
        axis = element(joint, 'axis')
        element(axis, 'xyz', '1 0 0')
        limits = element(axis, 'limit')
        for key, value in (('lower', -1e16), ('upper', 1e16), ('effort', .5), ('velocity', 300)):
            element(limits, key, value)
        element(element(axis, 'dynamics'), 'damping', .0001)
    element(model, 'plugin', name='work_motors', filename='libagribot_hil_work_motors.so')
    # Resolve package resources without copying or simplifying vehicle assets.
    from ament_index_python.packages import get_package_share_directory
    for uri in model.findall('.//uri'):
        if (uri.text or '').startswith('package://'):
            package, relative = uri.text[len('package://'):].split('/', 1)
            uri.text = str(Path(get_package_share_directory(package))/relative)
    output.mkdir(parents=True)
    ET.indent(tree, space='  ')
    tree.write(output/'vehicle.sdf', encoding='utf-8', xml_declaration=False)
    world = ET.Element('sdf', version='1.7')
    scene = element(world, 'world', name='tracked_work_bench')
    physics = element(scene, 'physics', name='ode', type='ode')
    element(physics, 'max_step_size', .001); element(physics, 'real_time_update_rate', 1000)
    ground = element(scene, 'model', name='ground')
    element(ground, 'static', 'true')
    link = element(ground, 'link', name='ground_link')
    for kind in ('visual', 'collision'):
        plane = element(element(element(link, kind, name='ground_'+kind), 'geometry'), 'plane')
        element(plane, 'normal', '0 0 1'); element(plane, 'size', '100 100')
    light = element(scene, 'light', name='sun', type='directional')
    element(light, 'pose', '0 0 10 0 0 0')
    element(light, 'diffuse', '.9 .9 .9 1'); element(light, 'specular', '.2 .2 .2 1')
    element(light, 'direction', '-.3 -.4 -1')
    tree = ET.ElementTree(world)
    ET.indent(tree, space='  ')
    tree.write(output/'world.sdf', encoding='utf-8', xml_declaration=False)
    report = dict(vehicle='Car3', purpose='ISOBUS spraying HIL commissioning',
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        plant=json.loads((cad/'provenance.json').read_text()),
        visual_lod=json.loads((cad/'visual_lod.json').read_text()),
        mount_xyz_m=mount, mount_yaw_rad=3.141592653589793, implement_collision=False,
        limitations=['Illustrative 10 kg inertia, motor curves and nozzle placement, not calibrated',
                     'Motor feedback from integrated shaft physics; no fluid dynamics claim',
                     'Flat commissioning pad, not a navigation/localization validation'])
    (output/'provenance.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


def main():
    from ament_index_python.packages import get_package_share_directory
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--description', type=Path,
                        default=Path(get_package_share_directory('agribot_vehicle_description')))
    parser.add_argument('--cad', type=Path, default=Path.home()/'.cache/agribot/isobus/work_models/plant_protection')
    parser.add_argument('--output', type=Path, default=Path.home()/'.cache/agribot/isobus/tracked_work_scene')
    args = parser.parse_args()
    print(json.dumps(prepare(args.description, args.cad, args.output), indent=2))


if __name__ == '__main__':
    main()
