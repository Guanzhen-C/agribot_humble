"""Blender offline mesh LOD, retaining the stem socket origin and source UVs."""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import bpy
import numpy as np
from scipy.spatial.transform import Rotation


parser = argparse.ArgumentParser()
parser.add_argument('--source', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--alignment', type=Path)
args = parser.parse_args(sys.argv[sys.argv.index('--')+1:])
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
if args.source.suffix == '.obj':
    bpy.ops.import_scene.obj(filepath=str(args.source), axis_forward='Y', axis_up='Z')
    report = json.loads(args.alignment.read_text())
    axis = np.asarray(report['source_stem_axis'])
    tangent = np.array([1., 0., 0.]) - axis * axis[0]
    tangent /= np.linalg.norm(tangent)
    rotation, _ = Rotation.align_vectors([[0., 0., 1.], [1., 0., 0.]], [axis, tangent])
    socket = np.asarray(report['source_stem_socket'])
    for obj in bpy.context.scene.objects:
        if obj.type == 'MESH':
            for vertex in obj.data.vertices:
                vertex.co = rotation.apply(np.asarray(vertex.co) - socket)
else:
    bpy.ops.wm.collada_import(filepath=str(args.source))
for obj in bpy.context.scene.objects:
    if obj.type != 'MESH':
        continue
    bpy.context.view_layer.objects.active = obj
    modifier = obj.modifiers.new('background_lod', 'DECIMATE')
    modifier.ratio = .125
    bpy.ops.object.modifier_apply(modifier=modifier.name)
    for polygon in obj.data.polygons:
        polygon.use_smooth = True
if not any(obj.type == 'MESH' for obj in bpy.context.scene.objects):
    raise RuntimeError('Apple mesh import was empty')
args.output.parent.mkdir(parents=True, exist_ok=True)
bpy.ops.wm.collada_export(filepath=str(args.output), include_animations=False,
                         apply_modifiers=True, triangulate=True, use_texture_copies=False)
tree = ET.parse(args.output)
namespace = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
for path in tree.findall('.//c:library_images/c:image/c:init_from', namespace):
    path.text = 'texture_map.png'
ET.register_namespace('', namespace['c'])
tree.write(args.output, encoding='utf-8', xml_declaration=True)
print('LOD_WRITTEN', args.output)
