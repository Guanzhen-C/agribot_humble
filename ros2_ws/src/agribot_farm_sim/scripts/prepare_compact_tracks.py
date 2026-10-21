#!/usr/bin/env python3
"""Fit inspected combine tread components to the existing Car3 visual envelope."""

import argparse
import json
import math
from pathlib import Path
import shutil
import sys

import bpy
from mathutils import Vector
from mathutils.geometry import convex_hull_2d

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "agribot_vehicle_config_tools"))
from prepare_harvester_appearance import export
from agribot_vehicle_config_tools.visual_belts import BeltPath


def load(path):
    before = set(bpy.context.scene.objects)
    bpy.ops.import_scene.obj(filepath=str(path), axis_forward="Y", axis_up="Z", use_split_objects=True)
    return [obj for obj in bpy.context.scene.objects if obj not in before and obj.type == "MESH"]


def bounds(objects):
    points = [v.co for obj in objects for v in obj.data.vertices]
    return Vector([min(p[i] for p in points) for i in range(3)]), Vector([max(p[i] for p in points) for i in range(3)])


def material(name, color):
    result = bpy.data.materials.new(name)
    result.diffuse_color = (*color, 1)
    return result


def track_shoe(name, pitch, width, rubber):
    vertices, faces = [], []
    def box(center, size):
        start = len(vertices)
        vertices.extend((center[0]+sx*size[0]/2,center[1]+sy*size[1]/2,center[2]+sz*size[2]/2)
                        for sx,sy,sz in ((-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),
                                         (-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)))
        faces.extend(tuple(start+i for i in f) for f in ((3,2,1,0),(4,5,6,7),(0,1,5,4),
                                                        (1,2,6,5),(2,3,7,6),(3,0,4,7)))
    box((0,0,-.0015),(pitch*.98,width,.021))
    box((0,0,.0125),(pitch*.26,width*.96,.007))
    box((0,0,-.017),(pitch*.45,width*.13,.011))
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(vertices,[],faces)
    mesh.materials.append(rubber)
    obj = bpy.data.objects.new(name,mesh)
    bpy.context.collection.objects.link(obj)
    return obj


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("donor", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args(sys.argv[sys.argv.index("--")+1:])
    if args.output.exists():
        parser.error("Use a new output directory")
    config = json.loads(args.config.read_text())
    if config["vehicleId"] != "unity_car3" or config["vehicleType"] != "tracked":
        raise ValueError("This inspected visual profile is only for Car3")
    donor = json.loads((args.donor / "vehicle_config.json").read_text())
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    body = load(args.source / "vehicle_visual.obj")
    old_tracks = [obj for obj in body if obj.name.split("_", 1)[0].isdigit()
                  and 68 <= int(obj.name.split("_", 1)[0]) <= 107]
    if len(old_tracks) != 40:
        raise ValueError("Car3 CAD component layout changed; re-inspect before replacing tracks")
    envelopes = {side: bounds([obj for obj in old_tracks if (obj.data.vertices[0].co.y > 0) == (side == "left")])
                 for side in ("left", "right")}
    for obj in old_tracks:
        body.remove(obj)
        bpy.data.objects.remove(obj, do_unlink=True)
    for obj in list(body):
        if obj.name.split("_",1)[0] in ("0108","0109"):
            body.remove(obj)
            bpy.data.objects.remove(obj,do_unlink=True)
    for mat in bpy.data.materials:
        if "BlackPowder" in mat.name:
            mat.diffuse_color = (.12, .14, .15, 1)
    rubber = material("compact_track_rubber", (.045, .052, .058))
    rim = material("compact_wheel_warm_white", (.72, .74, .70))
    steel = material("compact_track_steel", (.22, .26, .28))
    args.output.mkdir(parents=True)
    for path in args.source.iterdir():
        if path.suffix in (".obj", ".mtl") and path.name.startswith("sensor_"):
            shutil.copy2(path, args.output / path.name)
    rotors, belts, audit = [], [], {}
    for side, (low, high) in envelopes.items():
        center_y = (low.y+high.y)/2
        reference = next(b for b in donor["metadata"]["simulationVisualBelts"] if b["id"] == side)
        scale = (high.z-low.z)/1.725
        width_scale = (high.y-low.y)/.7645
        end_outer = .445*scale
        back, front = low.x+end_outer, high.x-end_outer
        bottom_z = low.z+end_outer
        upper_z = low.z+(1.21-.599+.515)*scale
        upper_x = (back+front)/2
        circles = [(back,bottom_z,(.445-.06)*scale), (front,bottom_z,(.445-.06)*scale),
                   (upper_x,upper_z,(.599-.06)*scale)]
        points = [Vector((x+r*math.cos(a*math.tau/64), z+r*math.sin(a*math.tau/64)))
                  for x,z,r in circles for a in range(64)]
        hull = [points[i] for i in convex_hull_2d(points)]
        if sum(a.x*b.y-b.x*a.y for a,b in zip(hull,hull[1:]+hull[:1])) > 0:
            hull.reverse()
        path = BeltPath([list(p) for p in hull])
        source_wheel = next(w for w in config["geometry"]["wheelCenters"] if w["id"] == "front_"+side)
        donor_rotors = [r for r in donor["metadata"]["simulationVisualRotors"]
                        if r["id"].startswith(side+"_")]
        ends = [r["center"][0] for r in donor_rotors if "idler" in r["id"]]
        for rotor in donor_rotors:
            objects = load(args.donor / rotor["meshFile"])
            old_radius = reference["sourceRadius"]/rotor["ratio"]
            for obj in objects:
                obj.data.materials.clear()
                obj.data.materials.append(rubber)
                obj.data.materials.append(rim)
                for face in obj.data.polygons:
                    radius = math.hypot(face.center.x,face.center.z)
                    face.material_index = 0 if radius > old_radius*.79 else 1
                for vertex in obj.data.vertices:
                    vertex.co = Vector((vertex.co.x*scale,vertex.co.y*width_scale,vertex.co.z*scale))
            export(objects, args.output / rotor["meshFile"])
            ratio_x = (rotor["center"][0]-min(ends))/(max(ends)-min(ends))
            center = [back+ratio_x*(front-back), center_y+(rotor["center"][1]-reference["centerY"])*width_scale,
                      low.z+(rotor["center"][2]+.515)*scale]
            rotor_low, rotor_high = bounds(objects)
            center[1] = max(low.y-rotor_low.y, min(high.y-rotor_high.y, center[1]))
            rotors.append(dict(id=rotor["id"],sourceWheel="front_"+side,center=center,
                               ratio=source_wheel["radiusM"]/(old_radius*scale),meshFile=rotor["meshFile"]))
            for obj in objects:
                bpy.data.objects.remove(obj,do_unlink=True)
        count = 64
        parts = []
        for index in range(count):
            objects = [track_shoe(f"compact_{side}_shoe_{index}",path.length/count,high.y-low.y,rubber)]
            filename = f"belt_{side}_{index}.obj"
            export(objects, args.output / filename)
            parts.append(dict(index=index,distance=(index+.5)*path.length/count,meshFile=filename))
            for obj in objects:
                bpy.data.objects.remove(obj,do_unlink=True)
        belts.append(dict(id=side,sourceWheel="front_"+side,sourceRadius=source_wheel["radiusM"],
                          centerY=center_y,points=path.points,segments=parts))
        # A recessed support rail connects the rollers to the existing side panel.
        for number, (a,b) in enumerate(((circles[0],circles[1]),(circles[0],circles[2]),(circles[1],circles[2]))):
            bpy.ops.mesh.primitive_cube_add(size=1)
            obj = bpy.context.object
            obj.name = f"compact_track_support_{side}_{number}"
            dx,dz = b[0]-a[0],b[1]-a[1]
            obj.location = ((a[0]+b[0])/2,center_y,(a[1]+b[1])/2)
            obj.scale = (math.hypot(dx,dz),.045,.025)
            obj.rotation_euler.y = -math.atan2(dz,dx)
            obj.data.materials.append(steel)
            body.append(obj)
        audit[side] = dict(originalMinimum=list(low),originalMaximum=list(high),
                           nominalLoopLengthM=path.length,trackWidthM=high.y-low.y,wheelScale=scale)
    export(body,args.output / "vehicle_visual.obj")
    pieces = rotors+[p for belt in belts for p in belt["segments"]]
    materials = {}
    for file in [args.output/"vehicle_visual.mtl",*[args.output/part["meshFile"].replace(".obj",".mtl") for part in pieces]]:
        for group in file.read_text().split("newmtl ")[1:]:
            materials[group.splitlines()[0]] = "newmtl "+group.strip()+"\n\n"
    for part in pieces:
        file=args.output/part["meshFile"]
        file.write_text("\n".join("mtllib vehicle_visual.mtl" if line.startswith("mtllib ") else line
                                 for line in file.read_text().splitlines())+"\n")
        file.with_suffix(".mtl").unlink()
    (args.output/"vehicle_visual.mtl").write_text("".join(materials.values()).rstrip()+"\n")
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
    from agribot_farm_sim.materials import normalize_ambient
    normalize_ambient(args.output/"vehicle_visual.mtl")
    config["metadata"].update(simulationVisualRotors=rotors,simulationVisualBelts=belts,
        simulationAppearanceRevision="compact_articulated_tracks_v1",simulationTrackVisualFit=audit)
    (args.output/"vehicle_config.json").write_text(json.dumps(config,ensure_ascii=False,indent=2)+"\n")
    print(json.dumps(dict(rotors=len(rotors),belts=len(belts),physicsChanged=False,fit=audit)))


if __name__ == "__main__":
    main()
