"""Derive a separate shredder HIL scene without retuning the accepted sprayer baseline."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np


SCALE = .55
MOUNT = [-.10, 0., .42665]
MOUNT_RPY = [0., 0., math.pi]


def coordinates(values):
    return ' '.join(f'{value:.12g}' for value in values)


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def relative_pose(parent_pose, xyz, desired_rpy=(0., 0., 0.)):
    inverse = rotation(parent_pose[3:]).T
    translation = inverse @ (np.asarray(xyz)-parent_pose[:3])
    matrix = inverse @ rotation(desired_rpy)
    rpy = [math.atan2(matrix[2, 1], matrix[2, 2]),
           math.atan2(-matrix[2, 0], math.hypot(matrix[2, 1], matrix[2, 2])),
           math.atan2(matrix[1, 0], matrix[0, 0])]
    return list(translation)+rpy


def signature(model):
    result = copy.deepcopy(model)
    for child in list(result):
        if child.get('name', '').startswith('work_'):
            result.remove(child)
    result.attrib.pop('name', None)
    for node in result.iter():
        if node.text is not None:
            node.text = node.text.strip() or None
        node.tail = None
    return ET.tostring(result)


def build(baseline, cad, output):
    baseline, cad, output = (Path(path).resolve() for path in (baseline, cad, output))
    if output.exists() or output == baseline or baseline in output.parents:
        raise FileExistsError('Use a new scene directory outside the accepted baseline')
    parts = json.loads((cad/'parts.json').read_text())
    for name in ('body.dae', 'rotor.dae'):
        if not (cad/name).is_file():
            raise FileNotFoundError(cad/name)
    original = ET.parse(baseline/'vehicle.sdf')
    source_model = original.find('model')
    tree = copy.deepcopy(original)
    model = tree.find('model')
    model.set('name', 'hil_tracked_shredder')
    plant = model.find("link[@name='work_plant_frame']")
    parent_pose = list(map(float, plant.findtext('pose').split()))
    for child in list(plant):
        if child.tag in ('visual', 'collision', 'sensor'):
            plant.remove(child)
    visual = ET.SubElement(plant, 'visual', name='supplied_shredder_cad')
    local_pose = relative_pose(parent_pose, MOUNT, MOUNT_RPY)
    ET.SubElement(visual, 'pose').text = coordinates(local_pose)
    mesh = ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh')
    ET.SubElement(mesh, 'uri').text = (cad/'body.dae').as_uri()
    ET.SubElement(mesh, 'scale').text = coordinates([SCALE]*3)
    for child in list(model):
        if child.get('name') in ('work_fan_rotor', 'work_pump_rotor', 'work_fan_joint',
                                 'work_pump_joint', 'work_spray_obstacles'):
            model.remove(child)
    mount_rotation = rotation(MOUNT_RPY)
    rotor_xyz = (np.asarray(MOUNT)+SCALE*(mount_rotation @ parts['rotor_origin_m'])).tolist()
    rotor = ET.SubElement(model, 'link', name='work_shredder_rotor')
    ET.SubElement(rotor, 'pose').text = coordinates(rotor_xyz+MOUNT_RPY)
    inertial = ET.SubElement(rotor, 'inertial')
    ET.SubElement(inertial, 'mass').text = '.1'
    inertia = ET.SubElement(inertial, 'inertia')
    for key in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'):
        ET.SubElement(inertia, key).text = '.003' if key in ('ixx', 'iyy', 'izz') else '0'
    visual = ET.SubElement(rotor, 'visual', name='original_cutter_and_pulley')
    mesh = ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh')
    ET.SubElement(mesh, 'uri').text = (cad/'rotor.dae').as_uri()
    ET.SubElement(mesh, 'scale').text = coordinates([SCALE]*3)
    joint = ET.SubElement(model, 'joint', name='work_shredder_joint', type='revolute')
    ET.SubElement(joint, 'parent').text = 'work_plant_frame'
    ET.SubElement(joint, 'child').text = 'work_shredder_rotor'
    axis = ET.SubElement(joint, 'axis')
    ET.SubElement(axis, 'xyz').text = coordinates(parts['rotor_axis'])
    limit = ET.SubElement(axis, 'limit')
    for key, value in dict(lower='-1e16', upper='1e16', effort='.5', velocity='400').items():
        ET.SubElement(limit, key).text = value
    ET.SubElement(ET.SubElement(axis, 'dynamics'), 'damping').text = '.0001'
    ET.SubElement(model.find("plugin[@name='work_motors']"), 'implement_kind').text = 'shredder'
    if signature(source_model) != signature(model):
        raise ValueError('Unexpected chassis, contact, sensor or non-implement plugin change')
    if model.findall("link[@name='work_plant_frame']/collision") or rotor.findall('collision'):
        raise ValueError('Implement must not contribute collision geometry')

    urdf = ET.parse(baseline/'vehicle.urdf')
    robot = urdf.getroot()
    plant_urdf = robot.find("link[@name='work_plant_frame']")
    for child in list(plant_urdf):
        if child.tag == 'visual':
            plant_urdf.remove(child)
    visual = ET.SubElement(plant_urdf, 'visual')
    ET.SubElement(visual, 'origin', xyz=coordinates(local_pose[:3]), rpy=coordinates(local_pose[3:]))
    ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh',
                  filename=(cad/'body.dae').as_uri(), scale=coordinates([SCALE]*3))
    rotor_urdf = ET.SubElement(robot, 'link', name='work_shredder_rotor')
    visual = ET.SubElement(rotor_urdf, 'visual')
    ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh',
                  filename=(cad/'rotor.dae').as_uri(), scale=coordinates([SCALE]*3))
    rotor_pose = relative_pose(parent_pose, rotor_xyz, MOUNT_RPY)
    joint = ET.SubElement(robot, 'joint', name='work_shredder_joint', type='continuous')
    ET.SubElement(joint, 'parent', link='work_plant_frame')
    ET.SubElement(joint, 'child', link='work_shredder_rotor')
    ET.SubElement(joint, 'origin', xyz=coordinates(rotor_pose[:3]), rpy=coordinates(rotor_pose[3:]))
    ET.SubElement(joint, 'axis', xyz=coordinates(parts['rotor_axis']))
    ET.SubElement(joint, 'limit', effort='.5', velocity='400')

    manifest = json.loads((baseline/'rdk/mission.json').read_text())
    manifest.update(scenario='hil_tracked_shredder_orchard', implement_kind='shredder')
    work = manifest.pop('spraying')
    work.pop('fan_gear'); work.pop('pump_gear')
    work['shredder_level'] = 3500
    manifest['shredding'] = work
    old_presentation = manifest['presentation']
    manifest['presentation'] = dict(shredder_mount=MOUNT, shredder_mount_rpy=MOUNT_RPY,
        feed_direction='front (+X)', discharge_direction='rear (-X)', shredder_scale=SCALE,
        dimensions_m=[SCALE*value for value in parts['actual_size_m']],
        rotor_base_xyz=rotor_xyz, rotor_axis=parts['rotor_axis'],
        rotor_axis_frame='work_shredder_rotor',
        rotor_axis_base=(mount_rotation @ parts['rotor_axis']).tolist(),
        rtk_master=old_presentation['rtk_master'], rtk_secondary=old_presentation['rtk_secondary'],
        imu_xyz=old_presentation['imu_xyz'], chassis_finish=old_presentation['chassis_finish'],
        limitation='Scaled illustrative mount/rotor response; no branch fracture or real implement calibration')
    frozen = {str(path.relative_to(baseline)): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in baseline.rglob('*') if path.is_file()}
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name+'.preparing')
    if temporary.exists():
        raise FileExistsError(temporary)
    shutil.copytree(baseline, temporary)
    for doc, filename in ((tree, 'vehicle.sdf'), (urdf, 'vehicle.urdf')):
        ET.indent(doc, space='  ')
        doc.write(temporary/filename, encoding='utf-8')
    (temporary/'rdk/mission.json').write_text(json.dumps(manifest, indent=2)+'\n')
    evidence = dict(baseline=str(baseline), baseline_sha256=frozen,
                    cad_source_sha256=parts['source_sha256'], mount=manifest['presentation'],
                    chassis_contacts_sensors_unchanged=True,
                    unchanged_files=[name for name, digest in frozen.items()
                        if hashlib.sha256((temporary/name).read_bytes()).hexdigest() == digest])
    (temporary/'shredder-provenance.json').write_text(json.dumps(evidence, indent=2)+'\n')
    temporary.rename(output)
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, default=Path.home()/'.cache/agribot/isobus/tracked_orchard')
    parser.add_argument('--cad', type=Path, default=Path.home()/'.cache/agribot/isobus/work_models/shredder/rigged')
    parser.add_argument('--output', type=Path, default=Path.home()/'.cache/agribot/isobus/tracked_shredder_orchard')
    args = parser.parse_args()
    print(json.dumps(build(args.baseline, args.cad, args.output), indent=2))
