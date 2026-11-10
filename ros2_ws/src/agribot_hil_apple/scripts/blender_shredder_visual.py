"""Export a neutral painted shredder body and a separately animated original CAD rotor."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import bpy
from mathutils import Vector
import numpy as np


def material(name, colour, metallic=.25):
    m = bpy.data.materials.new(name)
    m.diffuse_color = (*colour, 1)
    m.use_nodes = True
    node = m.node_tree.nodes['Principled BSDF']
    node.inputs['Base Color'].default_value = (*colour, 1)
    node.inputs['Metallic'].default_value = metallic
    node.inputs['Roughness'].default_value = .45
    return m


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args(sys.argv[sys.argv.index('--')+1:])
    root = args.directory
    if (root/'body.dae').exists() or (root/'rotor.dae').exists():
        raise FileExistsError(root)
    config = json.loads((root/'parts.json').read_text())
    arrays = np.load(root/'parts.npz')
    bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete(use_global=False)
    paint = material('Shredder green coated steel', (.12, .29, .19))
    steel = material('Shredder satin steel', (.42, .45, .44), .7)
    black = material('Shredder pulley dark finish', (.035, .04, .035))
    marker = material('Pulley paint witness mark, visual only', (.85, .87, .82), .05)
    total = sum(p['triangles'] for p in config['parts'])
    groups = {False: [], True: []}
    for part in config['parts']:
        index = part['index']
        mesh = bpy.data.meshes.new(f'original_solid_{index}')
        mesh.from_pydata(arrays[f'vertices_{index}'], [], arrays[f'faces_{index}'])
        mesh.update()
        obj = bpy.data.objects.new(mesh.name, mesh)
        bpy.context.collection.objects.link(obj)
        obj.data.materials.append(black if index == 55 else steel if part['moving'] else paint)
        if index == 55:
            obj.data.materials.append(marker)
            for polygon in mesh.polygons:
                center = polygon.center
                if abs(math.atan2(center.z, center.x)) < .15 and math.hypot(center.x, center.z) > .025:
                    polygon.material_index = 1
        bpy.context.view_layer.objects.active=obj
        modifier=obj.modifiers.new('Visual-only LOD', 'DECIMATE')
        modifier.ratio=min(1., 150000/total)
        bpy.ops.object.modifier_apply(modifier=modifier.name)
        groups[part['moving']].append(obj)
    outputs = {}
    for moving, name in ((False, 'body'), (True, 'rotor')):
        bpy.ops.object.select_all(action='DESELECT')
        for obj in groups[moving]: obj.select_set(True)
        bpy.context.view_layer.objects.active=groups[moving][0]
        bpy.ops.object.join()
        obj=bpy.context.object; obj.name='shredder_'+name
        bpy.ops.wm.collada_export(filepath=str(root/(name+'.dae')), selected=True)
        outputs[name]=dict(triangles=len(obj.data.polygons), sha256=hashlib.sha256((root/(name+'.dae')).read_bytes()).hexdigest())
        if moving: obj.location=Vector(config['rotor_origin_m'])
    size=max(config['actual_size_m'])
    center=Vector((0,0,config['actual_size_m'][2]/2))
    bpy.ops.object.camera_add(location=center+Vector((1.6,-1.9,1.1))*size)
    camera=bpy.context.object
    camera.rotation_euler=(center-camera.location).to_track_quat('-Z','Y').to_euler()
    camera.data.type='ORTHO'; camera.data.ortho_scale=size*1.55
    scene=bpy.context.scene; scene.camera=camera
    bpy.ops.object.light_add(type='AREA', location=(0,-2,3))
    bpy.context.object.data.energy=500; bpy.context.object.data.size=4
    scene.render.engine='BLENDER_EEVEE'; scene.world.color=(.25,.25,.25)
    scene.render.resolution_x=1200; scene.render.resolution_y=900; scene.render.resolution_percentage=100
    scene.render.filepath=str(root/'preview.png'); bpy.ops.render.render(write_still=True)
    (root/'visual.json').write_text(json.dumps(dict(outputs=outputs, source=config['source_sha256'],
        appearance='Illustrative green paint and pulley witness mark; original geometry retained'),indent=2)+'\n')


if __name__ == '__main__':
    main()
