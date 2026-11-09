"""Apply the material atlas to labelled CAD parts; retain the inspected metre frame."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

import bpy
import numpy as np


ROLES = {
    'white_hdpe': ((0., .5), (.86, .86, .84, 1.), 0., .6),
    'green_steel': ((.5, .5), (.025, .12, .065, 1.), .35, .48),
    'satin_steel': ((0., 0.), (.58, .60, .62, 1.), .75, .32),
    'black_polymer': ((.5, 0.), (.035, .04, .045, 1.), 0., .75),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--atlas', type=Path, required=True)
    args = parser.parse_args(sys.argv[sys.argv.index('--')+1:])
    root = args.directory.resolve()
    target = root/'visual_textured.dae'
    if target.exists():
        raise FileExistsError(target)
    source = json.loads((root/'parts.json').read_text())
    arrays = np.load(root/'parts.npz', allow_pickle=False)
    shutil.copy2(args.atlas, root/'material_atlas.png')
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    image = bpy.data.images.load(str(root/'material_atlas.png'))
    materials = {}
    for role, (_, colour, metallic, roughness) in ROLES.items():
        material = bpy.data.materials.new(role)
        material.diffuse_color = colour
        material.use_nodes = True
        nodes = material.node_tree.nodes
        surface = nodes['Principled BSDF']
        surface.inputs['Metallic'].default_value = metallic
        surface.inputs['Roughness'].default_value = roughness
        surface.inputs['Base Color'].default_value = colour
        texture = nodes.new('ShaderNodeTexImage')
        texture.image = image
        material.node_tree.links.new(texture.outputs['Color'], surface.inputs['Base Color'])
        materials[role] = material
    original_count = sum(part['triangles'] for part in source['parts'])
    objects = []
    for part in source['parts']:
        index, role = part['index'], part['material']
        mesh = bpy.data.meshes.new(f'cad_{index:03d}')
        mesh.from_pydata(arrays[f'vertices_{index}'].tolist(), [], arrays[f'faces_{index}'].tolist())
        mesh.update()
        obj = bpy.data.objects.new(mesh.name, mesh)
        bpy.context.collection.objects.link(obj)
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        modifier = obj.modifiers.new('Visual-only LOD', 'DECIMATE')
        modifier.ratio = min(1., max(150000/original_count, 32/len(mesh.polygons)))
        bpy.ops.object.modifier_apply(modifier=modifier.name)
        mesh = obj.data
        mesh.materials.append(materials[role])
        uv = mesh.uv_layers.new(name='MaterialAtlas')
        coordinates = np.array([tuple(v.co) for v in mesh.vertices])
        low, span = coordinates.min(axis=0), np.maximum(coordinates.ptp(axis=0), 1e-6)
        corner = np.array(ROLES[role][0])
        # Box projection stays inside one atlas quadrant, with a seam-safe margin.
        for polygon in mesh.polygons:
            axes = [axis for axis in range(3) if axis != np.argmax(np.abs(polygon.normal))]
            for loop_index in polygon.loop_indices:
                point = (coordinates[mesh.loops[loop_index].vertex_index]-low)/span
                uv.data[loop_index].uv = corner+.02+.46*point[axes]
        mesh.use_auto_smooth = True
        for polygon in mesh.polygons:
            polygon.use_smooth = True
        objects.append(obj)
        obj.select_set(False)
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    bpy.ops.object.join()
    result = bpy.context.object
    vertices = np.array([tuple(v.co) for v in result.data.vertices])
    bpy.ops.wm.collada_export(filepath=str(target), selected=True,
                             use_texture_copies=True, include_animations=False)
    report = dict(source_sha256=source['source_sha256'], source_triangles=original_count,
                  visual_triangles=len(result.data.polygons), parts=len(source['parts']),
                  roles=list(ROLES), output_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
                  atlas_sha256=hashlib.sha256((root/'material_atlas.png').read_bytes()).hexdigest(),
                  bounds_m=[vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()],
                  scale=1., note='Same normalized metre frame as original visual, visual only')
    (root/'appearance.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
