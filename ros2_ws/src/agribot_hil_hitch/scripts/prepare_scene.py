"""Derive the dedicated Car4 rear-hitch bench; never modify original exports."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def add(parent, tag, value=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if value is not None:
        node.text = str(value)
    return node


def vector(values):
    return ' '.join(f'{x:.12g}' for x in values)


def pitch_point(values, angle):
    x, y, z = values
    return [math.cos(angle)*x+math.sin(angle)*z, y, -math.sin(angle)*x+math.cos(angle)*z]


def inertial(link, mass, xyz, moments):
    item = add(link, 'inertial')
    add(item, 'pose', vector([*xyz, 0, 0, 0]))
    add(item, 'mass', mass)
    moment = add(item, 'inertia')
    for key, value in zip(('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'), [*moments, 0, 0, 0]):
        add(moment, key, value)


def visual(link, name, mesh, pose=(0, 0, 0, 0, 0, 0), colour=None):
    item = add(link, 'visual', name=name)
    add(item, 'pose', vector(pose))
    add(add(add(item, 'geometry'), 'mesh'), 'uri', str(mesh))
    if colour is not None:
        material = add(item, 'material')
        add(material, 'ambient', vector([*[v*.6 for v in colour], 1]))
        add(material, 'diffuse', vector([*colour, 1]))
    return item


def prepare(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    source = args.description/'generated/car4_harrow/models/car4_harrow.sdf'
    before = hashlib.sha256(source.read_bytes()).hexdigest()
    parts = json.loads((args.meshes/'parts.json').read_text())
    tree = ET.parse(source)
    model = tree.find('model')
    model.set('name', 'hil_hitch_tractor')
    for item in list(model):
        if item.get('name', '').startswith('mounted_disc_harrow'):
            model.remove(item)
    base = model.find("link[@name='base_link']")
    base.find("visual[@name='vehicle_body_visual']/geometry/mesh/uri").text = str(args.meshes/'tractor_body.obj')
    drive = model.find("plugin[@name='ackermann_drive']")
    drive.find('cmd_vel_topic').text = '/hil_hitch/actuator/cmd_vel'
    drive.find('odom_topic').text = '/hil_hitch/sim_wheel_odom'
    drive.find('publish_tf').text = 'false'
    if args.powertrain:
        drive.find('wheel_torque').text = '0'
        add(model, 'plugin', name='hil_powertrain_plant', filename='libagribot_hil_powertrain_plant.so')
    camera = base.find("sensor[@type='camera']")
    camera.find('camera/image/width').text = '640'
    camera.find('camera/image/height').text = '512'
    camera.find('plugin/camera_info_url').text = 'package://agribot_ackermann_mppi/config/hikrobot_camera_640_sim.yaml'
    base.find("sensor[@type='ray']/ray/scan/horizontal/samples").text = '720'
    base.find("sensor[@type='ray']/ray/range/max").text = '30'
    base.find("sensor[@type='ray']/plugin/max_range").text = '30'
    truth = add(model, 'plugin', name='hil_hitch_ground_truth', filename='libgazebo_ros_p3d.so')
    ros = add(truth, 'ros'); add(ros, 'remapping', 'odom:=/hil_hitch/ground_truth')
    add(truth, 'body_name', 'base_link'); add(truth, 'frame_name', 'world'); add(truth, 'update_rate', 30)
    # Single equivalent lift coordinate avoids an overconstrained ODE linkage.
    # Original geometry, masses and physical wheel/steering joints are retained.
    pivot = [0., 0., -.13]
    mount = [-1.3075548580371181, 0., -.8410066730860496]
    implement_pitch = .035
    tool = add(model, 'link', name='hitch_tool_frame')
    add(tool, 'pose', vector([*pivot, 0, 0, 0]))
    add(tool, 'self_collide', 'false')
    inertial(tool, 92., [-1.5, 0., 0.], [55., 105., 140.])
    offset = [a-b for a, b in zip(mount, pivot)]
    visual(tool, 'harrow_frame', args.meshes/'harrow_frame.obj', [*offset, 0, implement_pitch, 0], [.12, .25, .15])
    visual(tool, 'original_lift_arms', args.meshes/'tractor_lift_parts.obj', [-pivot[0], 0, -pivot[2], 0, 0, 0])
    # The renderer is fed only actual physical engagement from HitchPlant.
    anchor = add(tool, 'visual', name='soil_effect_anchor')
    add(add(add(anchor, 'geometry'), 'sphere'), 'radius', .001)
    add(anchor, 'transparency', 1)
    soil_effect = add(anchor, 'plugin', name='soil_tillage', filename='libagribot_hil_hitch_tillage.so')
    add(soil_effect, 'soil_texture', args.farm_assets/'textures/09fa174109_668785e108_brown_mud_diff_2k.png')
    add(soil_effect, 'ground_z', -.002)
    for disc in parts['discs']:
        point = [a+b-c for a, b, c in zip(mount, pitch_point(disc['center'], implement_pitch), pivot)]
        add(soil_effect, 'disc_origin', vector(point))
    roller = parts.get('roller')
    if roller is None:
        raise ValueError('Rebuild meshes to separate the original rear cage roller')
    roller_origin = [a+b for a, b in zip(mount, pitch_point(roller['center'], implement_pitch))]
    add(soil_effect, 'roller_origin', vector([a-b for a, b in zip(roller_origin, pivot)]))
    add(soil_effect, 'roller_radius', roller['radius_m'])
    add(soil_effect, 'working_width', roller['width_m'])
    disc_mean_x = sum(disc['center'][0] for disc in parts['discs'])/len(parts['discs'])
    add(soil_effect, 'settled_bed_offset', roller['center'][0]-disc_mean_x-.15)
    joint = add(model, 'joint', name='hitch_lift_joint', type='revolute')
    add(joint, 'parent', 'base_link'); add(joint, 'child', 'hitch_tool_frame')
    axis = add(joint, 'axis'); add(axis, 'xyz', '0 1 0')
    limit = add(axis, 'limit')
    lift_min, lift_max = -.035, .4
    for key, value in (('lower', lift_min), ('upper', lift_max), ('effort', 6000), ('velocity', .16)):
        add(limit, key, value)
    add(add(axis, 'dynamics'), 'damping', 20.)
    plugin = add(model, 'plugin', name='hitch_physical_plant', filename='libagribot_hil_hitch_plant.so')
    add(plugin, 'lift_min', lift_min)
    add(plugin, 'lift_span', lift_max-lift_min)
    add(plugin, 'zone_min', '-28 -10 0'); add(plugin, 'zone_max', '28 10 0')
    roller_link = add(model, 'link', name='hitch_cage_roller')
    add(roller_link, 'pose', vector([*roller_origin, 0, implement_pitch, 0]))
    add(roller_link, 'self_collide', 'false')
    inertial(roller_link, 4., [0, 0, 0], [2.2, .16, 2.2])
    visual(roller_link, 'original_cage_roller', args.meshes/roller['file'], colour=[.12, .25, .15])
    roller_joint = add(model, 'joint', name='hitch_cage_roller_joint', type='revolute')
    add(roller_joint, 'parent', 'hitch_tool_frame'); add(roller_joint, 'child', 'hitch_cage_roller')
    axis = add(roller_joint, 'axis'); add(axis, 'xyz', vector(roller['axis']))
    limit = add(axis, 'limit')
    for key, value in (('lower', -1e16), ('upper', 1e16), ('effort', 20), ('velocity', 30)):
        add(limit, key, value)
    add(add(axis, 'dynamics'), 'damping', .02)
    setting = add(plugin, 'roller')
    add(setting, 'joint', 'hitch_cage_roller_joint'); add(setting, 'link', 'hitch_cage_roller')
    add(setting, 'radius', roller['radius_m'])
    adapters = [
        ('lower_left', [-.964, .4526, -.115], [-.06, .2913, .565], True),
        ('lower_right', [-.964, -.4548, -.119], [-.06, -.2917, .565], True),
        ('top_link', [-.26, 0., .36], [-.0623, -.0002, 1.0408], False),
        ('cylinder_left', [-.48, .34, .46], [.85, .36, .626006673086], False),
        ('cylinder_right', [-.48, -.34, .46], [.85, -.36, .626006673086], False),
    ]
    for name, host, point, from_tool in adapters:
        target = [a+b for a, b in zip(mount, pitch_point(point, implement_pitch))]
        if not from_tool:
            anchor = add(base, 'visual', name='hitch_adapter_'+name)
            add(add(add(anchor, 'geometry'), 'sphere'), 'radius', .001)
            renderer = add(anchor, 'plugin', name='telescopic_linkage', filename='libagribot_hil_hitch_linkage.so')
            add(renderer, 'host', vector(host))
            add(renderer, 'tool_eye', vector([a-b for a, b in zip(target, pivot)]))
            add(renderer, 'radius', .025 if name == 'top_link' else .035)
            add(renderer, 'initial_tool_pose', vector([*pivot, 0, 0, 0]))
            continue
        delta = [b-a for a, b in zip(host, target)]
        length = math.sqrt(sum(v*v for v in delta))
        parent = tool if from_tool else base
        rod = add(parent, 'visual', name='hitch_adapter_'+name)
        midpoint = [(a+b)/2-(pivot[i] if from_tool else 0) for i, (a, b) in enumerate(zip(host, target))]
        add(rod, 'pose', vector([*midpoint,
                                0, math.atan2(math.hypot(*delta[:2]), delta[2]), math.atan2(delta[1], delta[0])]))
        cylinder = add(add(rod, 'geometry'), 'cylinder')
        add(cylinder, 'radius', .025 if 'cylinder' not in name else .035)
        add(cylinder, 'length', length)
        material = add(rod, 'material'); add(material, 'ambient', '.22 .24 .23 1'); add(material, 'diffuse', '.4 .43 .4 1')
    for index, disc in enumerate(parts['discs']):
        name = f'hitch_disc_{index:02d}'
        origin = [a+b for a, b in zip(mount, pitch_point(disc['center'], implement_pitch))]
        link = add(model, 'link', name=name)
        add(link, 'pose', vector([*origin, 0, implement_pitch, 0]))
        add(link, 'self_collide', 'false')
        inertial(link, .5, [0, 0, 0], [.008, .012, .008])
        visual(link, name+'_cad', args.meshes/disc['file'], colour=[.23, .26, .28])
        joint = add(model, 'joint', name=name+'_joint', type='revolute')
        add(joint, 'parent', 'hitch_tool_frame'); add(joint, 'child', name)
        axis = add(joint, 'axis'); add(axis, 'xyz', vector(disc['axis']))
        limit = add(axis, 'limit')
        for key, value in (('lower', -1e16), ('upper', 1e16), ('effort', 20), ('velocity', 30)):
            add(limit, key, value)
        add(add(axis, 'dynamics'), 'damping', .02)
        setting = add(plugin, 'disc')
        add(setting, 'joint', name+'_joint'); add(setting, 'link', name); add(setting, 'radius', disc['radius_m'])
        for point in disc['probes']:
            add(setting, 'probe', vector(point))
    for uri in model.findall('.//uri'):
        if (uri.text or '').startswith('package://'):
            package, relative = uri.text[len('package://'):].split('/', 1)
            from ament_index_python.packages import get_package_share_directory
            uri.text = str(Path(get_package_share_directory(package))/relative)
    args.output.mkdir(parents=True)
    ET.indent(tree, space='  '); tree.write(args.output/'vehicle.sdf', encoding='utf-8')
    world_tree = ET.parse(args.world)
    world = world_tree.find('world'); world.set('name', 'hitch_farmland')
    for include in list(world.findall('include')):
        if include.findtext('uri') == 'model://sun':
            world.remove(include)
    light = add(world, 'light', name='hitch_sun', type='directional')
    add(light, 'pose', '0 0 20 0 0 0'); add(light, 'direction', '-.3 -.4 -1')
    add(light, 'diffuse', '.9 .9 .85 1'); add(light, 'specular', '.15 .15 .15 1')
    add(light, 'cast_shadows', 'true')
    # Resolve the existing farm asset through this checkout, not another install.
    environment = world.find("model[@name='agricultural_environment']/link/visual/geometry/mesh/uri")
    if environment is not None:
        environment.text = str(args.farm_assets/'scene.obj')
    camera = world.find('gui/camera/pose')
    if camera is not None:
        camera.text = '-28 -10 8 0 .40 .65'
    included = copy.deepcopy(model)
    add(included, 'pose', '-20 0 .74 0 0 0')
    world.append(included)
    ET.indent(world_tree, space='  '); world_tree.write(args.output/'world.sdf', encoding='utf-8')
    shutil.copy2(args.farm_assets/'map.pgm', args.output/'map.pgm')
    shutil.copy2(args.farm_assets/'map.yaml', args.output/'map.yaml')
    report = dict(source=str(source), source_sha256=before, vehicle='Car4', implement='mounted_disc_harrow',
                  scene='wide_headland original soil, local tillage swaths', disc_count=16,
                  lift_min_rad=lift_min, lift_max_rad=lift_max, lift_span_rad=lift_max-lift_min,
                  implement_level_pitch_rad=implement_pitch, implement_mount_drop_m=.085,
                  rotating_original_cage_roller=True, soil_effect_stages=['disc_clod_throw', 'cage_roller_crumbling'],
                  source_unchanged=True, implement_collision=False, native_protocol='hitch-hil-0x21',
                  full_plot_overlay=False, tillage_texture='Original soil texture, shallow crumbled harrow bed',
                  powertrain_force_plant=args.powertrain, velocity_drive_bypass=not args.powertrain,
                  limitations=['Equivalent single-coordinate lift, not calibrated hydraulic linkage',
                               'Animated soil disturbance, not DEM or agronomic depth/force validation'])
    (args.output/'provenance.json').write_text(json.dumps(report, indent=2)+'\n')
    assert hashlib.sha256(source.read_bytes()).hexdigest() == before
    print(json.dumps(report))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--description', type=Path, required=True)
    parser.add_argument('--meshes', type=Path, required=True)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--farm-assets', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--powertrain', action='store_true')
    prepare(parser.parse_args())
