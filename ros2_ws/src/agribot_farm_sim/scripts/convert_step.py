#!/usr/bin/env python3
"""Convert a local STEP/IGES assembly to a metric mesh and inspectable metadata.

Run with the system Python and FreeCAD libraries. The source axes are retained;
mounting and wheel anchors must be defined separately after visual inspection.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--deflection-mm", type=float, default=3.0)
    parser.add_argument("--angular-deflection-rad", type=float, default=0.35)
    parser.add_argument("--include-open-surfaces", action="store_true",
                        help="Also retain open CAD faces omitted by Solid-only tessellation")
    args = parser.parse_args()
    if args.input.suffix.lower() not in (".step", ".stp", ".iges", ".igs"):
        parser.error("input must be a STEP or IGES file")
    if not math.isfinite(args.deflection_mm) or args.deflection_mm <= 0:
        parser.error("deflection must be positive")
    if not math.isfinite(args.angular_deflection_rad) or not 0 < args.angular_deflection_rad < math.pi:
        parser.error("angular deflection must be between zero and pi radians")
    sys.path.insert(0, "/usr/lib/freecad-python3/lib")
    import FreeCAD  # Initialize the application before loading Part's extension.
    import Part
    import MeshPart

    shape = Part.Shape()
    shape.read(str(args.input.resolve()))
    if shape.isNull():
        raise ValueError("Empty CAD geometry")
    args.output.mkdir(parents=True, exist_ok=True)
    vertices, faces = [], []
    solids = list(shape.Solids) or [shape]
    open_faces = []
    if args.include_open_surfaces and shape.Solids:
        covered = {}
        for solid in shape.Solids:
            for face in solid.Faces:
                covered.setdefault(face.hashCode(), []).append(face)
        open_faces = [face for face in shape.Faces if not any(
            face.isSame(known) for known in covered.get(face.hashCode(), []))]
        if open_faces:
            # Preserve existing solid indices used by inspected wheel mappings.
            solids.append(Part.makeCompound(open_faces))
    components = []
    for index, solid in enumerate(solids):
        mesh = MeshPart.meshFromShape(Shape=solid, LinearDeflection=args.deflection_mm,
                                      AngularDeflection=args.angular_deflection_rad, Relative=False)
        points, triangles = mesh.Topology
        offset = len(vertices)
        vertices.extend(tuple(float(value) / 1000 for value in point) for point in points)
        faces.extend((index, tuple(vertex + offset + 1 for vertex in face)) for face in triangles)
        box = solid.BoundBox
        components.append({"name": f"solid_{index}", "faces": len(triangles),
                           "geometryType": "open_surfaces" if open_faces and index == len(solids)-1 else solid.ShapeType,
                           "minimumM": [box.XMin / 1000, box.YMin / 1000, box.ZMin / 1000],
                           "maximumM": [box.XMax / 1000, box.YMax / 1000, box.ZMax / 1000]})
    with (args.output / "visual.obj").open("w") as stream:
        stream.write("# FreeCAD STEP coordinates in metres; original axes retained\nmtllib visual.mtl\n")
        for vertex in vertices:
            stream.write("v " + " ".join(f"{value:.7f}" for value in vertex) + "\n")
        previous = None
        for index, triangle in faces:
            if index != previous:
                stream.write(f"o solid_{index}\nusemtl cad_default\n")
                previous = index
            stream.write("f " + " ".join(map(str, triangle)) + "\n")
    (args.output / "visual.mtl").write_text("newmtl cad_default\nKd 0.55 0.62 0.66\nKa 0.15 0.15 0.15\nd 1\n")
    box = shape.BoundBox
    info = {"sourceSha256": hashlib.sha256(args.input.read_bytes()).hexdigest(),
            "sourceFile": str(args.input.resolve()), "units": "metres", "axes": "source_CAD_unverified",
            "status": "mesh_converted_mounting_unverified", "vertices": len(vertices), "faces": len(faces),
            "linearDeflectionMm": args.deflection_mm,
            "angularDeflectionRad": args.angular_deflection_rad,
            "includedOpenFaces": len(open_faces),
            "sizeM": [box.XLength / 1000, box.YLength / 1000, box.ZLength / 1000],
            "components": components}
    (args.output / "mesh_info.json").write_text(json.dumps(info, indent=2) + "\n")
    print(json.dumps({key: value for key, value in info.items() if key != "components"}), flush=True)


if __name__ == "__main__":
    main()
