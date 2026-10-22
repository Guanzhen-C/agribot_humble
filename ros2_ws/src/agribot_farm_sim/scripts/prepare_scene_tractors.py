#!/usr/bin/env python3
"""Build nominal metric tractor bundles without changing authored Unity models.

Lengths/masses are appearance-based simulation assumptions authorized by the
user, not manufacturer dimensions. Sensors retain their existing metric meshes.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil

from agribot_vehicle_config_tools.model import validate
from agribot_farm_sim.mesh_surface import load_surfaces


PROFILES = {
    "unity_car1": (4.8, 6000., "Car1 full-size cab tractor"),
    "unity_car2": (5.2, 7000., "Car2 full-size cab tractor"),
    "unity_car4": (3.6, 2500., "Car4 utility tractor"),
    "unity_car5": (3.4, 1400., "Car5 compact tractor"),
}
WHEELS = dict(zip(("FLwheel", "FRwheel", "RLwheel", "RRwheel"),
                  ("front_left", "front_right", "rear_left", "rear_right")))
SENSOR_MOUNTS = {
    "unity_car1": dict(x=2.8, hood="SM2_John_Deere_6195M_Nose", riser=.14,
                       rtk_x=.6, rtk_surface="SM2_John_Deere_6195M_Cab"),
    "unity_car2": dict(x=2.7, hood="Carrenagem_do_Motor", riser=.14,
                       rtk_x=.6, rtk_surface="Teto_da_Capota"),
    "unity_car4": dict(x=1.95, hood="0036_Hood_Default<As_Machined>", riser=.43,
                       rtk_x=1.6, rtk_surface="0036_Hood_Default<As_Machined>"),
    "unity_car5": dict(x=1.6, hood="0030_Product1_AllCATPart", riser=.14,
                       rtk_x=1.1, rtk_surface="0030_Product1_AllCATPart"),
}


def append_box(stream, vertex_count, name, center, size):
    stream.write(f"\no SensorMount_{name}\nusemtl NominalSensorMount\n")
    for dx, dy, dz in ((-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),
                       (-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)):
        point = [center[i] + sign * size[i] / 2 for i, sign in enumerate((dx,dy,dz))]
        stream.write("v " + " ".join(format(v, ".10g") for v in point) + "\n")
    for face in ((1,4,3,2),(5,6,7,8),(1,2,6,5),(2,3,7,6),(3,4,8,7),(4,1,5,8)):
        stream.write("f " + " ".join(str(vertex_count+i) for i in face) + "\n")
    return vertex_count + 8


def prepare(source, sensor_source, output, vehicle_id):
    if output.exists():
        raise FileExistsError(output)
    source_config = source / "vehicle_config.json"
    config = json.loads(source_config.read_text())
    if config["vehicleId"] != vehicle_id:
        raise ValueError("source vehicle ID mismatch")
    length, mass, label = PROFILES[vehicle_id]
    geometry = config["geometry"]
    extents = geometry["bodyExtents"]
    factor = length / (extents["frontM"] + extents["rearM"])
    for name in extents:
        extents[name] *= factor
    for key in ("wheelbaseM", "frontTrackM", "rearTrackM", "wheelRadiusM", "wheelWidthM", "minTurningRadiusM"):
        geometry[key] *= factor
    corrections = {}
    for wheel in geometry["wheelCenters"]:
        wheel["position"] = [v * factor for v in wheel["position"]]
        wheel["radiusM"] *= factor
        wheel["widthM"] *= factor
        new_z = wheel["radiusM"] - geometry["wheelRadiusM"]
        corrections[wheel["id"]] = new_z - wheel["position"][2]
        wheel["position"][2] = new_z
        wheel["massKg"] = mass * .025
        if wheel["steering"]:
            wheel["steeringMassKg"] = mass * .002
    for key in ("center", "size"):
        config["collision"][key] = [v * factor for v in config["collision"][key]]
    size = config["collision"]["size"]
    config["dynamics"].update(massKg=mass,
        centerOfMass=[geometry["wheelbaseM"] * .45, 0., .15],
        inertia={"ixx": mass / 12 * (size[1]**2 + size[2]**2),
                 "iyy": mass / 12 * (size[0]**2 + size[2]**2),
                 "izz": mass / 12 * (size[0]**2 + size[1]**2),
                 "ixy": 0., "ixz": 0., "iyz": 0.},
        maxAccelerationMps2=.5, wheelTorqueNm=mass*geometry["wheelRadiusM"]*.5)
    config["navigation"].update(maxLinearSpeedMps=.5,
        maxAngularSpeedRadps=.5 / geometry["minTurningRadiusM"],
        inflationRadiusM=2., costScalingFactor=3.,
        obstacleMinHeightM=-geometry["wheelRadiusM"]+.15,
        obstacleMaxHeightM=config["collision"]["center"][2]+size[2]/2+.5)
    config["chassis"].update(commandMaxLinearSpeedMps=.5,
        commandMaxAngularSpeedRadps=.5 / geometry["minTurningRadiusM"])
    # Use the same measured-size sensor assets as the existing farm vehicles.
    sensor_config = json.loads((sensor_source / "vehicle_config.json").read_text())
    config["sensors"] = copy.deepcopy(sensor_config["sensors"])
    config["displayName"] = label
    config.setdefault("metadata", {}).update(
        calibrationState="unity_nominal_simulation_only", simulationVelocityFromPose=True,
        nominalDimensions={"basis": "user-authorized appearance estimate; not manufacturer or measured data",
                           "targetBodyLengthM": length, "sourceScale": factor,
                           "sourceConfigSha256": hashlib.sha256(source_config.read_bytes()).hexdigest()},
        notes="Metric body/wheels; original sensor meshes unchanged in size; inspected hood mounts with surface-contact feet.")
    config.pop("implements", None)
    config.pop("manipulators", None)
    output.mkdir(parents=True)
    shutil.copy2(source / "vehicle_visual.mtl", output / "vehicle_visual.mtl")
    with (output / "vehicle_visual.mtl").open("a") as material:
        material.write("\nnewmtl NominalSensorMount\nKd 0.18 0.20 0.21\nKa 0.03 0.03 0.03\n")
    from agribot_farm_sim.materials import normalize_ambient
    normalize_ambient(output / "vehicle_visual.mtl")
    wheel_id = None
    vertex_count = 0
    with (source / "vehicle_visual.obj").open() as src, (output / "vehicle_visual.obj").open("w") as dst:
        for line in src:
            if line.startswith("o "):
                wheel_id = WHEELS.get(line[2:].strip().split("__", 1)[0])
            if line.startswith("v "):
                values = [float(v)*factor for v in line.split()[1:4]]
                values[2] += corrections.get(wheel_id, 0.)
                line = "v " + " ".join(format(v, ".10g") for v in values) + "\n"
                vertex_count += 1
            dst.write(line)
    profile = SENSOR_MOUNTS[vehicle_id]
    front, rtk_x = profile["x"], profile["rtk_x"]
    surfaces = load_surfaces(output / "vehicle_visual.obj", {profile["hood"], profile["rtk_surface"]})
    hood, rtk_surface = surfaces[profile["hood"]], surfaces[profile["rtk_surface"]]
    feet = [(x, y) for x in (front-.28, front+.07) for y in (-.10, .10)]
    # Sample the platform footprint too: a curved hood must not penetrate it.
    top = max(hood.height(front-.3 + i*.04, y) for i in range(11)
              for y in (-.12, -.06, 0., .06, .12)) + .025
    rtk_top = max(rtk_surface.height(rtk_x, y) for y in (-.10, 0., .10)) + .04
    lidar_z = top + profile["riser"] + .039030008
    positions = {"imu": [front-.23, 0., top+.03],
                 "lidar": [front, 0., lidar_z],
                 "right_camera": [front+.0253, 0., lidar_z+.118587],
                 "rtk_master": [rtk_x, .325, rtk_top+.02+.5202005],
                 "rtk_secondary": [rtk_x, -.325, rtk_top+.02+.5202005]}
    for sensor in config["sensors"]:
        sensor["pose"]["xyz"] = positions[sensor["id"]]
    contacts = []
    with (output / "vehicle_visual.obj").open("a") as dst:
        def post(name, x, y, upper, surface, source_part):
            surface_z = surface.height(x, y)
            lower = surface_z - .01
            if upper <= surface_z:
                raise ValueError(f"mounting platform intersects {source_part}")
            nonlocal vertex_count
            vertex_count = append_box(dst, vertex_count, name, [x,y,(upper+lower)/2], [.035,.035,upper-lower])
            contacts.append(dict(name=name, xyz=[x,y,surface_z], sourcePart=source_part))
        for x, y in feet:
            post(f"front_{x}_{y}",x,y,top,hood,profile["hood"])
        vertex_count = append_box(dst,vertex_count,"front_plate",[front-.105,0,top+.01],[.43,.26,.02])
        vertex_count = append_box(dst,vertex_count,"lidar_riser",[front,0,top+profile["riser"]/2],[.04,.04,profile["riser"]])
        for y in (-.10,.10):
            post(f"rtk_{y}",rtk_x,y,rtk_top,rtk_surface,profile["rtk_surface"])
        vertex_count = append_box(dst,vertex_count,"rtk_crossbar",[rtk_x,0,rtk_top+.01],[.06,.72,.02])
    config["metadata"]["sensorMountContacts"] = contacts
    config["metadata"]["sensorMountBasis"] = "Nominal CAD surface intersections, not measured hardware calibration."
    validate(config)
    for path in sensor_source.glob("sensor_*_visual.*"):
        if path.suffix in (".obj", ".mtl"):
            shutil.copy2(path, output / path.name)
    (output / "vehicle_config.json").write_text(json.dumps(config, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps({"vehicle": vehicle_id, "scale": factor, "length": length,
                      "width": extents["leftM"]+extents["rightM"], "mass": mass}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("sensor_source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    for vehicle_id in PROFILES:
        prepare(args.source / vehicle_id, args.sensor_source, args.output / vehicle_id, vehicle_id)
