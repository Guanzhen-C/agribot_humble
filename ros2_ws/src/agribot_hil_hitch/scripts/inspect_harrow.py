"""Inspect supplied CAD object bounds and principal axes inside Blender."""
import argparse
import json
from pathlib import Path
import sys

import bpy
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('source', type=Path)
parser.add_argument('output', type=Path)
args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
bpy.ops.import_scene.obj(filepath=str(args.source), axis_forward='Y', axis_up='Z',
                         use_split_objects=True, use_split_groups=False)
parts = []
for obj in bpy.context.scene.objects:
    if obj.type != 'MESH' or not obj.data.vertices:
        continue
    points = np.asarray([tuple(obj.matrix_world @ v.co) for v in obj.data.vertices])
    low, high = points.min(axis=0), points.max(axis=0)
    center = points.mean(axis=0)
    values, vectors = np.linalg.eigh(np.cov((points-center).T))
    parts.append(dict(name=obj.name, vertices=len(points), low=low.tolist(), high=high.tolist(),
                      dimensions=(high-low).tolist(), center=center.tolist(),
                      eigenvalues=values.tolist(), thin_axis=vectors[:, 0].tolist()))
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(parts, indent=2)+'\n')
print(json.dumps([p for p in parts if .35 < max(p['dimensions']) < .9 and
                  p['eigenvalues'][1] > .01 and p['eigenvalues'][0] < .002], indent=2))
