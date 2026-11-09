"""Derive a millimetre-tessellated, metre-scale visual mesh from supplied STEP."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, '/usr/lib/freecad/lib')
import FreeCAD
import Part
import Mesh


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    before = hashlib.sha256(args.source.read_bytes()).hexdigest()
    shape = Part.read(str(args.source))
    bounds = shape.BoundBox
    mesh = Mesh.Mesh(shape.tessellate(.6))
    transform = FreeCAD.Matrix()
    transform.A11 = transform.A22 = transform.A33 = .001
    transform.A14 = -(bounds.XMin + bounds.XMax) * .0005
    transform.A24 = -(bounds.YMin + bounds.YMax) * .0005
    transform.A34 = -bounds.ZMin * .001
    mesh.transform(transform)
    args.output.mkdir(parents=True)
    target = args.output/'visual.stl'
    mesh.write(str(target))
    assert before == hashlib.sha256(args.source.read_bytes()).hexdigest()
    report = dict(source=str(args.source), source_sha256=before,
        visual_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
        linear_deflection_source_mm=.6, output_unit='m',
        source_bbox_mm=[bounds.XMin, bounds.YMin, bounds.ZMin, bounds.XMax, bounds.YMax, bounds.ZMax],
        visual_size_m=[bounds.XLength*.001, bounds.YLength*.001, bounds.ZLength*.001],
        triangles=mesh.CountFacets, solids=len(shape.Solids),
        transform_note='CAD XY bounding-box centre and lowest Z normalized; no orientation or mount inferred',
        limitation='Geometry visual only, no original paint colours or internal motor axes inferred')
    (args.output/'provenance.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
