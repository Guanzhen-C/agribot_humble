"""Blender-only visual LOD/render; never changes supplied STEP or collision."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import bpy
from mathutils import Vector


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    root = args.directory.resolve()
    output = root/'visual_lod.dae'
    if output.exists():
        raise FileExistsError(output)
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    bpy.ops.import_mesh.stl(filepath=str(root/'visual.stl'))
    mesh = bpy.context.object
    original = len(mesh.data.polygons)
    decimate = mesh.modifiers.new('HIL presentation LOD', 'DECIMATE')
    decimate.ratio = min(1., 150000 / original)
    bpy.ops.object.modifier_apply(modifier=decimate.name)
    material = bpy.data.materials.new('Neutral CAD finish, original paint unknown')
    material.diffuse_color = (.36, .43, .38, 1)
    material.use_nodes = True
    material.node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value = (.36, .43, .38, 1)
    material.node_tree.nodes['Principled BSDF'].inputs['Metallic'].default_value = .35
    material.node_tree.nodes['Principled BSDF'].inputs['Roughness'].default_value = .4
    mesh.data.materials.append(material)
    bpy.ops.wm.collada_export(filepath=str(output), selected=True)
    bpy.ops.object.camera_add(location=(2.1, -2.5, 1.7))
    camera = bpy.context.object
    camera.rotation_euler = (Vector((0, 0, .4)) - camera.location).to_track_quat('-Z', 'Y').to_euler()
    camera.data.type = 'ORTHO'
    camera.data.ortho_scale = 2.6
    bpy.context.scene.camera = camera
    bpy.ops.object.light_add(type='AREA', location=(0, -2, 3))
    bpy.context.object.data.energy = 600
    bpy.context.object.data.size = 4
    scene = bpy.context.scene
    scene.render.engine = 'BLENDER_EEVEE'
    scene.world.color = (.3, .3, .3)
    scene.render.resolution_x = 1100
    scene.render.resolution_y = 900
    scene.render.resolution_percentage = 100
    scene.render.filepath = str(root/'cad-preview.png')
    bpy.ops.render.render(write_still=True)
    report = dict(source_triangles=original, visual_triangles=len(mesh.data.polygons),
        output_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        scale=1., note='Visual-only LOD, metre dimensions unchanged; neutral finish not original paint')
    (root/'visual_lod.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
