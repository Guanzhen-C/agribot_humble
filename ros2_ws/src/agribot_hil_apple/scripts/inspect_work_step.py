"""Inspect supplied STEP assemblies with FreeCAD; never modify their source files."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, '/usr/lib/freecad/lib')
import FreeCAD
import Part


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    before = hashlib.sha256(args.source.read_bytes()).hexdigest()
    shape = Part.read(str(args.source))
    shapes = []
    for index, item in enumerate(shape.Solids):
        bound = item.BoundBox
        shapes.append(dict(solid_index=index,
            faces=len(item.Faces),
            bbox_mm=[bound.XMin, bound.YMin, bound.ZMin, bound.XMax, bound.YMax, bound.ZMax]))
    bound = shape.BoundBox
    report = dict(source=str(args.source), sha256=before, freecad=FreeCAD.Version(), objects=shapes,
                  bbox_mm=[bound.XMin, bound.YMin, bound.ZMin, bound.XMax, bound.YMax, bound.ZMax],
                  scope='Geometry-only inspection; source assembly labels and motor axes are not inferred')
    assert before == hashlib.sha256(args.source.read_bytes()).hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(objects=len(shapes), output=str(args.output)), ensure_ascii=False))


if __name__ == '__main__':
    main()
