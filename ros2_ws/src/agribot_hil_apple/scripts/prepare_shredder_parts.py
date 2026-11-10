"""Split the supplied shredder CAD at its actual cutter/pulley axis; source stays intact."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, '/usr/lib/freecad/lib')
import FreeCAD
import Part


MOVING_SOLIDS = set(range(15, 24)) | {55}
ALIGN = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    before = hashlib.sha256(args.source.read_bytes()).hexdigest()
    assembly = Part.read(str(args.source))
    if len(assembly.Solids) != 134:
        raise ValueError('Expected the inspected 134-solid shredder assembly')
    cylinders = [f.Surface for f in assembly.Solids[55].Faces if isinstance(f.Surface, Part.Cylinder)]
    axis_surface = max(cylinders, key=lambda c: c.Radius)
    axis = ALIGN @ np.array(tuple(axis_surface.Axis))
    if abs(axis[1]) < .999:
        raise ValueError('Inspected pulley axis changed')
    axis_point = ALIGN @ np.array(tuple(axis_surface.Center)) * .001
    arrays, parts, all_vertices = {}, [], []
    for index, solid in enumerate(assembly.Solids):
        vertices, faces = solid.tessellate(.6)
        vertices = np.asarray([tuple(v) for v in vertices]) @ ALIGN.T * .001
        faces = np.asarray(faces, dtype=np.int32).reshape(-1, 3)
        if not len(vertices) or not len(faces):
            continue
        arrays[f'vertices_{index}'] = vertices
        arrays[f'faces_{index}'] = faces
        all_vertices.append(vertices)
        parts.append(dict(index=index, moving=index in MOVING_SOLIDS, triangles=len(faces)))
    points = np.concatenate(all_vertices)
    low, high = points.min(axis=0), points.max(axis=0)
    # STEP spline control-point bounds overestimate this CAD by several metres.
    # Use real tessellated geometry for placement, never the untrimmed BRep bbox.
    origin = np.array([(low[0]+high[0])/2, (low[1]+high[1])/2, low[2]])
    pivot = axis_point-origin
    for part in parts:
        key = f'vertices_{part["index"]}'
        arrays[key] -= origin + (pivot if part['moving'] else 0)
    args.output.mkdir(parents=True)
    np.savez_compressed(args.output/'parts.npz', **arrays)
    report = dict(source=str(args.source), source_sha256=before, source_solids=134,
                  moving_solids=sorted(MOVING_SOLIDS), parts=parts, output_unit='m',
                  aligned_origin_m=origin.tolist(), actual_size_m=(high-low).tolist(),
                  rotor_origin_m=pivot.tolist(), rotor_axis=axis.tolist(),
                  alignment='CAD +Y up mapped to simulation +Z; CAD +X remains vehicle +X',
                  limitation='Visual rotor grouping and scaled mount, not real implement calibration')
    (args.output/'parts.json').write_text(json.dumps(report, indent=2)+'\n')
    assert hashlib.sha256(args.source.read_bytes()).hexdigest() == before
    print(json.dumps({k: report[k] for k in ('actual_size_m', 'rotor_origin_m', 'rotor_axis')}))


if __name__ == '__main__':
    main()
