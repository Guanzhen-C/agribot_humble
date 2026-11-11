"""Derive articulated hitch visuals with Blender, without decimating source CAD."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import bpy
import numpy as np
from bpy_extras.node_shader_utils import PrincipledBSDFWrapper
from mathutils import Vector


def load(path):
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    bpy.ops.import_scene.obj(filepath=str(path), axis_forward='Y', axis_up='Z',
                             use_split_objects=True, use_split_groups=False)
    return [o for o in bpy.context.scene.objects if o.type == 'MESH']


def export(objects, path, origin=(0, 0, 0)):
    bpy.ops.object.select_all(action='DESELECT')
    shift = Vector(origin)
    for obj in objects:
        obj.select_set(True)
        obj.location -= shift
    bpy.ops.export_scene.obj(filepath=str(path), use_selection=True, axis_forward='Y', axis_up='Z',
                             use_materials=True, use_triangles=True, use_mesh_modifiers=False,
                             path_mode='COPY')
    # Blender's OBJ exporter writes white Ka/Ks for imported legacy materials.
    # Preserve the CAD diffuse colours without washing red paint into pink.
    materials = {slot.material.name: slot.material for obj in objects for slot in obj.material_slots
                 if slot.material is not None}
    with path.with_suffix('.mtl').open('w') as stream:
        for name, material in sorted(materials.items()):
            if material.use_nodes and any(node.type == 'TEX_IMAGE' for node in material.node_tree.nodes):
                raise ValueError('This CAD material writer must not discard image textures')
            rgb = tuple(PrincipledBSDFWrapper(material).base_color[:3])
            stream.write('newmtl '+name.replace(' ', '_')+'\n')
            stream.write('Ka '+' '.join(f'{v*.25:.6f}' for v in rgb)+'\n')
            stream.write('Kd '+' '.join(f'{v:.6f}' for v in rgb)+'\n')
            stream.write('Ks 0.08 0.08 0.08\nNs 36\nKe 0 0 0\nd 1\nillum 2\n\n')
    for obj in objects:
        obj.location += shift


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    sources = [args.source/'vehicle_visual.obj', args.source/'implement_mounted_disc_harrow_visual.obj']
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    body = load(sources[0])
    moving = [o for o in body if o.name.startswith(('0042_', '0044_'))]
    rods = [o for o in body if o.name.startswith(('0045_', '0046_'))]
    moving_names = [o.name for o in moving]
    export([o for o in body if o not in moving and o not in rods], args.output/'tractor_body.obj')
    # The lift frame retains the supplied lower arms, rather than covering them
    # with an unrelated high-poly replacement or moving a second fixed copy.
    export(moving, args.output/'tractor_lift_parts.obj')
    parts = load(sources[1])
    disc_ids = {127, 215, 303, 391, 479, 567, 655, 743,
                1204, 1292, 1382, 1472, 1562, 1652, 1742, 1832}
    discs = [o for o in parts if o.name.startswith('solid_') and int(o.name.split('_')[1]) in disc_ids]
    if len(discs) != 16:
        raise ValueError('Expected the inspected sixteen disc blades')
    # These two cage halves and their spiral bars share the inspected rear axle.
    # Keep bearing housings and support arms fixed, rather than spinning the frame.
    roller = [o for o in parts if o.name.startswith('solid_') and
              786 <= int(o.name.split('_')[1]) <= 913]
    if len(roller) != 128:
        raise ValueError('Expected both original cage-roller halves and spiral bars')
    frame = [o for o in parts if o not in discs and o not in roller and not o.name.startswith('hitch_')]
    paint = bpy.data.materials.new('HarrowFrameGreen')
    paint.diffuse_color = (.12, .25, .15, 1.)
    steel = bpy.data.materials.new('HarrowDiscSteel')
    steel.diffuse_color = (.25, .28, .30, 1.)
    for obj in frame+roller:
        obj.data.materials.clear()
        obj.data.materials.append(paint)
    for obj in discs:
        obj.data.materials.clear()
        obj.data.materials.append(steel)
    export(frame, args.output/'harrow_frame.obj')
    roller_points = np.concatenate([
        np.asarray([tuple(o.matrix_world @ v.co) for v in o.data.vertices]) for o in roller])
    roller_low, roller_high = roller_points.min(axis=0), roller_points.max(axis=0)
    roller_center = (roller_low+roller_high)/2
    export(roller, args.output/'harrow_roller.obj', roller_center)
    roller_report = dict(file='harrow_roller.obj', center=roller_center.tolist(), axis=[0, 1, 0],
                         radius_m=float((roller_high[2]-roller_low[2])/2),
                         width_m=float(roller_high[1]-roller_low[1]),
                         source_objects=[o.name for o in roller])
    report = []
    for index, obj in enumerate(discs):
        points = np.asarray([tuple(obj.matrix_world @ v.co) for v in obj.data.vertices])
        center = (points.min(axis=0) + points.max(axis=0)) / 2
        values, axes = np.linalg.eigh(np.cov((points - points.mean(axis=0)).T))
        axis = axes[:, 0]
        if axis[1] < 0:
            axis = -axis
        filename = f'disc_{index:02d}.obj'
        export([obj], args.output/filename, center)
        report.append(dict(name=obj.name, file=filename, center=center.tolist(), axis=axis.tolist(),
                           radius_m=.27, probes=(points-center).tolist(), vertices=len(points)))
    (args.output/'parts.json').write_text(json.dumps(dict(
        source_sha256=hashes, discs=report, roller=roller_report, moving_tractor_parts=moving_names,
        decimation=False, unit='m',
        limitation='Nominal equivalent lift mechanism; not calibrated hydraulic or soil dynamics'), indent=2)+'\n')
    for p in sources:
        assert hashlib.sha256(p.read_bytes()).hexdigest() == hashes[str(p)]
    print('Hitch CAD prepared: original body, lift parts, frame and sixteen original disc blades')


if __name__ == '__main__':
    main()
