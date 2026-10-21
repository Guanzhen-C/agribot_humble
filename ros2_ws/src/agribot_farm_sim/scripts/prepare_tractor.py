#!/usr/bin/env python3
"""Normalize the locally purchased tractor CAD to the existing Unity/ROS contract.

Input is convert_step.py's directory, not an arbitrary OBJ: verify the source
digest before applying the explicit solid-group mapping. Keep meshes local.
"""

import argparse
import copy
import json
import math
from pathlib import Path
import sys

import bpy
import bmesh
from mathutils import Matrix, Vector


WHEELS = {"FLwheel": "front_left", "FRwheel": "front_right",
          "RLwheel": "rear_left", "RRwheel": "rear_right"}


def bounds(objects):
    points = [obj.matrix_world @ Vector(corner) for obj in objects for corner in obj.bound_box]
    return (Vector([min(point[i] for point in points) for i in range(3)]),
            Vector([max(point[i] for point in points) for i in range(3)]))


def select(objects):
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]


def material(name, rgba):
    value = bpy.data.materials.new(name)
    value.diffuse_color = rgba
    value.use_nodes = True
    shader = value.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Base Color"].default_value = rgba
    shader.inputs["Roughness"].default_value = .7 if name == "rubber" else .45
    return value


def pose(xyz, rpy=None, quaternion=None):
    return {"xyz": list(xyz), "rpy": rpy or [0., 0., 0.],
            "quaternion": quaternion or [0., 0., 0., 1.]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--template", type=Path, required=True)
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    if args.output.exists():
        parser.error("Use a new output directory; existing exports are not overwritten")
    profile = json.loads(args.profile.read_text())
    info = json.loads((args.input / "mesh_info.json").read_text())
    if info["sourceSha256"] != profile["sourceSha256"]:
        parser.error("The CAD digest does not match the inspected solid-group mapping")
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    bpy.ops.import_scene.obj(filepath=str((args.input / "visual.obj").resolve()),
                             axis_forward="Y", axis_up="Z", use_split_objects=True)
    objects = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    by_id = {int(obj.name.removeprefix("solid_")): obj for obj in objects}
    expected_solids = profile.get("solidCount", 63)
    if len(by_id) != expected_solids:
        raise ValueError(f"Expected the inspected {expected_solids}-solid tractor assembly")
    origin = Vector(profile["sourceOriginM"])
    rotation = Matrix(profile["sourceToFluRotation"])
    base_material = material("frame", (.16, .19, .20, 1.))
    colours = {name: material(name, item["rgba"]) for name, item in profile["materials"].items()}
    original_faces = sum(len(obj.data.polygons) for obj in objects)
    ratio = min(1., profile["maxVisualFaces"] / original_faces)
    for index, obj in by_id.items():
        for vertex in obj.data.vertices:
            vertex.co = rotation @ (vertex.co - origin)
        if profile.get("recalculateNormals", False):
            mesh = bmesh.new()
            mesh.from_mesh(obj.data)
            bmesh.ops.recalc_face_normals(mesh, faces=list(mesh.faces))
            mesh.to_mesh(obj.data)
            mesh.free()
        obj.data.materials.clear()
        colour = next((colours[name] for name, entry in profile["materials"].items()
                       if index in entry["solids"]), base_material)
        obj.data.materials.append(colour)
        if ratio < 1 and len(obj.data.polygons) > 32:
            bpy.context.view_layer.objects.active = obj
            modifier = obj.modifiers.new("VisualLOD", "DECIMATE")
            modifier.ratio = ratio
            bpy.ops.object.modifier_apply(modifier=modifier.name)
    caster_profiles = profile.get("casters", {})
    caster_pivots = {}
    for name, caster in caster_profiles.items():
        pivot = rotation @ (Vector(caster["sourcePivotM"]) - origin)
        caster_pivots[name] = pivot
        neutral = Matrix.Rotation(caster.get("neutralYawRad", 0.), 3, "Z")
        for index in caster["forkSolids"] + profile["wheelGroups"][name]:
            for vertex in by_id[index].data.vertices:
                vertex.co = pivot + neutral @ (vertex.co - pivot)
    bpy.context.view_layer.update()
    wheel_data = {}
    for name, indices in profile["wheelGroups"].items():
        group = [by_id[index] for index in indices]
        select(group)
        bpy.ops.object.join()
        obj = bpy.context.object
        obj.name = name
        obj.data.name = name
        lo, hi = bounds([obj])
        center = (lo + hi) / 2
        correction = profile.get("wheelNeutralYawRad", {}).get(name, 0.)
        if correction:
            neutral = Matrix.Rotation(correction, 3, "Z")
            for vertex in obj.data.vertices:
                vertex.co = center + neutral @ (vertex.co - center)
            bpy.context.view_layer.update()
            lo, hi = bounds([obj])
            center = (lo + hi) / 2
        radius = max(hi.x - lo.x, hi.z - lo.z) / 2
        wheel_data[name] = {"object": obj, "position": list(center),
                            "radiusM": radius, "widthM": hi.y - lo.y}
    rear_radius = sum(wheel_data[name]["radiusM"] for name in ("RLwheel", "RRwheel")) / 2
    adjustments = {}
    axle_x = {prefix: sum(wheel_data[name]["position"][0] for name in WHEELS if name.startswith(prefix)) / 2
              for prefix in ("F", "R")}
    for name, wheel in wheel_data.items():
        target_z = wheel["radiusM"] - rear_radius
        adjustments[name] = target_z - wheel["position"][2]
        for vertex in wheel["object"].data.vertices:
            vertex.co.z += adjustments[name]
        wheel["position"][2] = target_z
        if name in caster_profiles:
            caster_pivots[name].z += adjustments[name]
            for index in caster_profiles[name]["forkSolids"]:
                for vertex in by_id[index].data.vertices:
                    vertex.co.z += adjustments[name]
        if profile.get("alignAxleCenters", False):
            target_x = axle_x[name[0]] if name.startswith("F") else 0.
            dx = target_x - wheel["position"][0]
            for vertex in wheel["object"].data.vertices:
                vertex.co.x += dx
            wheel["position"][0] = target_x
            if name in caster_profiles:
                caster_pivots[name].x += dx
                for index in caster_profiles[name]["forkSolids"]:
                    for vertex in by_id[index].data.vertices:
                        vertex.co.x += dx
    bpy.context.view_layer.update()
    objects = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    lo, hi = bounds(objects)
    config = copy.deepcopy(json.loads(args.template.read_text()))
    config["vehicleId"] = profile["id"]
    config["displayName"] = profile["displayName"]
    config["vehicleType"] = profile.get("vehicleType", "ackermann")
    config["manipulators"] = []
    config["implements"] = []
    geom = config["geometry"]
    geom["bodyExtents"] = dict(frontM=hi.x, rearM=-lo.x, leftM=hi.y, rightM=-lo.y)
    geom["wheelbaseM"] = sum(wheel_data[key]["position"][0] for key in ("FLwheel", "FRwheel")) / 2
    geom["frontTrackM"] = wheel_data["FLwheel"]["position"][1] - wheel_data["FRwheel"]["position"][1]
    geom["rearTrackM"] = wheel_data["RLwheel"]["position"][1] - wheel_data["RRwheel"]["position"][1]
    geom["wheelRadiusM"] = rear_radius
    geom["wheelWidthM"] = wheel_data["RLwheel"]["widthM"]
    sim = profile["simulation"]
    geom["maxSteeringAngleRad"] = sim["maxSteeringAngleRad"]
    geom["minTurningRadiusM"] = sim.get("minTurningRadiusM",
        geom["wheelbaseM"] / math.tan(geom["maxSteeringAngleRad"]))
    geom["wheelCenters"] = [dict(id=WHEELS[name], position=wheel["position"],
                                      radiusM=wheel["radiusM"], widthM=wheel["widthM"],
                                      massKg=sim["frontWheelMassKg" if name.startswith("F") else "rearWheelMassKg"],
                                      steering=name.startswith("F") and config["vehicleType"] == "ackermann",
                                      driven=name.startswith("R") or sim.get("frontWheelDriven", False))
                                for name, wheel in wheel_data.items()]
    for wheel in geom["wheelCenters"]:
        if wheel["steering"]:
            wheel["steeringMassKg"] = sim["steeringMassKg"]
        name = next(name for name, wheel_id in WHEELS.items() if wheel_id == wheel["id"])
        if name in caster_profiles:
            key = "steeringAssembly" if wheel["steering"] else "caster"
            wheel[key] = dict(pivotPosition=list(caster_pivots[name]), damping=.02,
                meshUri=f"package://agribot_vehicle_description/generated/{profile['id']}/models/{wheel['id']}_caster.obj")
            if key == "caster":
                wheel[key]["massKg"] = caster_profiles[name]["massKg"]
    speed = sim["maxLinearSpeedMps"]
    config["navigation"].update(maxLinearSpeedMps=speed,
        maxAngularSpeedRadps=speed / geom["minTurningRadiusM"],
        inflationRadiusM=sim.get("inflationRadiusM", .8), costScalingFactor=3.,
        obstacleMinHeightM=sim["obstacleMinHeightM"], obstacleMaxHeightM=sim["obstacleMaxHeightM"])
    config["chassis"].update(commandMaxLinearSpeedMps=speed,
        commandMaxAngularSpeedRadps=speed / geom["minTurningRadiusM"])
    if config["vehicleType"] != "ackermann":
        angular_speed = sim.get("maxAngularSpeedRadps", .45)
        config["navigation"]["maxAngularSpeedRadps"] = angular_speed
        config["chassis"]["commandMaxAngularSpeedRadps"] = angular_speed
    size, center = sim["bodyCollisionSize"], sim["bodyCollisionCenter"]
    mass = sim["massKg"]
    diagonal = [mass * sum(size[j] ** 2 for j in range(3) if j != i) / 12 for i in range(3)]
    config["collision"] = dict(center=center, size=size)
    config["dynamics"].update(massKg=mass, centerOfMass=center, maxAccelerationMps2=.5,
        inertia=dict(ixx=diagonal[0], iyy=diagonal[1], izz=diagonal[2], ixy=0., ixz=0., iyz=0.))
    if "wheelTorqueNm" in sim:
        config["dynamics"]["wheelTorqueNm"] = sim["wheelTorqueNm"]
    config["visuals"] = [dict(name="vehicle_body", pose=pose([0., 0., 0.]), scale=[1., 1., 1.],
        meshUri=f"package://agribot_vehicle_description/generated/{profile['id']}/models/vehicle_visual.obj")]
    for sensor in config["sensors"]:
        sensor["pose"] = pose(sim["sensorPositions"][sensor["id"]])
        if sensor["sensorType"] == "camera":
            sensor["measurementFramePose"] = pose([0., 0., 0.],
                [-math.pi / 2, 0., -math.pi / 2], [-.5, .5, -.5, .5])
    config["metadata"] = dict(authoringTool="inspected_local_CAD_profile", profileVersion="1.0.0",
        calibrationState="unity_nominal_simulation_only", simulationVelocityFromPose=True,
        notes=" ".join(profile["notes"]))
    args.output.mkdir(parents=True)
    fork_objects = []
    for name, caster in caster_profiles.items():
        group = [by_id[index] for index in caster["forkSolids"]]
        select(group)
        bpy.ops.object.join()
        fork = bpy.context.object
        fork_objects.append(fork)
        for vertex in fork.data.vertices:
            vertex.co -= caster_pivots[name]
        filename = WHEELS[name] + "_caster.obj"
        bpy.ops.export_scene.obj(filepath=str((args.output / filename).resolve()),
            use_selection=True, axis_forward="Y", axis_up="Z", use_triangles=True, use_materials=True)
    objects = [obj for obj in bpy.context.scene.objects if obj.type == "MESH" and obj not in fork_objects]
    select(objects)
    bpy.ops.export_scene.obj(filepath=str((args.output / "vehicle_visual.obj").resolve()),
        use_selection=True, axis_forward="Y", axis_up="Z", use_triangles=True, use_materials=True)
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from agribot_farm_sim.materials import normalize_ambient
    normalize_ambient(args.output / "vehicle_visual.mtl")
    for name, fork in zip(caster_profiles, fork_objects):
        normalize_ambient(args.output / (WHEELS[name] + "_caster.mtl"))
        for vertex in fork.data.vertices:
            vertex.co += caster_pivots[name]
    (args.output / "vehicle_config.json").write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n")
    report = dict(sourceSha256=info["sourceSha256"], sourceFaces=original_faces,
        visualFaces=sum(len(obj.data.polygons) for obj in objects + fork_objects),
        sourceOriginM=list(origin), sourceToFluRotation=profile["sourceToFluRotation"],
        wheelNeutralYawRad=profile.get("wheelNeutralYawRad", {}),
        frontWheelLevellingM=adjustments, minimumM=list(lo), maximumM=list(hi),
        redistributionLicense=profile["redistributionLicense"], simulationOnly=True)
    (args.output / "cad_import.json").write_text(json.dumps(report, indent=2) + "\n")
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.color_type = "MATERIAL"
    scene.display.shading.show_cavity = True
    scene.display.shading.light = "STUDIO"
    scene.world.color = (.8, .8, .8)
    scene.render.resolution_x, scene.render.resolution_y = 1200, 900
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    camera_data = bpy.data.cameras.new("Preview")
    camera = bpy.data.objects.new("Preview", camera_data)
    scene.collection.objects.link(camera)
    camera_data.type = "ORTHO"
    camera_data.ortho_scale = (hi - lo).length * 1.05
    center = (hi + lo) / 2
    camera.location = center + Vector((4., -5., 3.))
    camera.rotation_euler = (center - camera.location).to_track_quat("-Z", "Y").to_euler()
    scene.camera = camera
    scene.render.filepath = str((args.output / "preview.png").resolve())
    bpy.ops.render.render(write_still=True)
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
