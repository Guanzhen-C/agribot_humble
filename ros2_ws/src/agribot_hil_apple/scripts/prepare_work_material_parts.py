"""Tessellate the supplied STEP into labelled material parts, without changing it."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, '/usr/lib/freecad/lib')
import FreeCAD
import Part


def material_role(index):
    if index == 109:
        return 'white_hdpe'
    if 110 <= index <= 116:
        return 'black_polymer'
    if index in (47, 48, 64) or 65 <= index <= 108:
        return 'satin_steel'
    return 'green_steel'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cad', type=Path)
    args = parser.parse_args()
    provenance = json.loads((args.cad/'provenance.json').read_text())
    source = Path(provenance['source'])
    before = hashlib.sha256(source.read_bytes()).hexdigest()
    if before != provenance['source_sha256']:
        raise ValueError('STEP source differs from the inspected installation')
    destination = args.cad/'textured'
    destination.mkdir(exist_ok=False)
    assembly = Part.read(str(source))
    bounds = assembly.BoundBox
    origin = np.array([(bounds.XMin+bounds.XMax)/2,
                       (bounds.YMin+bounds.YMax)/2, bounds.ZMin])
    arrays, parts = {}, []
    for index, solid in enumerate(assembly.Solids):
        vertices, faces = solid.tessellate(.6)
        arrays[f'vertices_{index}'] = (np.array([tuple(v) for v in vertices])-origin)*.001
        arrays[f'faces_{index}'] = np.array(faces, dtype=np.int32)
        parts.append(dict(index=index, material=material_role(index), triangles=len(faces)))
    np.savez_compressed(destination/'parts.npz', **arrays)
    report = dict(source=str(source), source_sha256=before, origin_mm=origin.tolist(),
                  parts=parts, output_unit='m',
                  appearance='User-selected white HDPE, green paint, steel and black polymer; not original paint')
    (destination/'parts.json').write_text(json.dumps(report, indent=2)+'\n')
    assert hashlib.sha256(source.read_bytes()).hexdigest() == before
    print(json.dumps(dict(parts=len(parts), triangles=sum(p['triangles'] for p in parts))))


if __name__ == '__main__':
    main()
