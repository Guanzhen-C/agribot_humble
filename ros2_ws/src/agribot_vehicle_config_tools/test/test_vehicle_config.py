import copy
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from agribot_vehicle_config_tools.generator import (
    _simplify_obj_vertex_clusters,
    _split_integrated_visual_obj,
    generate_bundle,
    generate_simulation_sdf,
    generate_sdf,
    generate_urdf,
)
from agribot_vehicle_config_tools.model import (
    ConfigError,
    fastlivo_extrinsics,
    footprint,
    load_and_validate,
)
from agribot_vehicle_config_tools.verifier import verify_current
from agribot_vehicle_config_tools.cli import main as config_main


PACKAGE_ROOT = Path(__file__).parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parent
BASELINE = (
    WORKSPACE_SRC
    / "agribot_vehicle_description"
    / "config"
    / "vehicles"
    / "ackermann_current"
    / "vehicle_config.json"
)


def _load():
    return load_and_validate(BASELINE)[0]


def test_current_ackermann_configuration_is_valid():
    config, warnings = load_and_validate(BASELINE)
    assert config["vehicleId"] == "ackermann_current"
    assert warnings == []


def test_ackermann_radius_is_consistent():
    config = _load()
    geometry = config["geometry"]
    calculated = geometry["wheelbaseM"] / math.tan(geometry["maxSteeringAngleRad"])
    assert calculated == pytest.approx(geometry["minTurningRadiusM"], abs=1e-6)


def test_navigation_footprint_includes_explicit_clearance():
    expected = [
        [0.754818, 0.485974],
        [0.754818, -0.485974],
        [-0.2275, -0.485974],
        [-0.2275, 0.485974],
    ]
    for actual_point, expected_point in zip(footprint(_load()), expected):
        assert actual_point == pytest.approx(expected_point)


def test_fastlivo_extrinsics_are_derived_from_base_link_sensor_poses():
    extrinsics = fastlivo_extrinsics(_load())
    assert extrinsics["extrinsic_T"] == pytest.approx(
        [0.338308452, 0.000050633, 0.086911672], abs=2e-6
    )
    # Preserve the serialized mount rotations rather than rounding them to axes.
    assert extrinsics["Pcl"] == pytest.approx(
        [0.000000122800367, -0.079002376748972, -0.095500094196799], abs=2e-6)


def test_inconsistent_turning_radius_is_rejected(tmp_path):
    config = _load()
    config["geometry"]["minTurningRadiusM"] = 2.0
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps(config), encoding="utf-8")
    with pytest.raises(ConfigError, match="minTurningRadiusM"):
        load_and_validate(path)


def test_vehicle_id_cannot_escape_its_export_directory(tmp_path):
    config = _load()
    config["vehicleId"] = "../ackermann_current"
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps(config))
    with pytest.raises(ConfigError, match="vehicleId"):
        load_and_validate(path)


@pytest.mark.parametrize("fault", (
    "missing_geometry", "negative_radius", "nan_radius", "infinite_speed",
    "duplicate_sensor", "duplicate_frame", "wrong_vector", "bad_quaternion",
    "negative_infinite_speed", "boolean_radius", "long_quaternion",
))
def test_documented_invalid_inputs_are_rejected(tmp_path, fault):
    config = _load()
    if fault == "missing_geometry":
        del config["geometry"]
    elif fault == "negative_radius":
        config["geometry"]["wheelRadiusM"] = -1
    elif fault == "nan_radius":
        config["geometry"]["wheelRadiusM"] = math.nan
    elif fault == "infinite_speed":
        config["navigation"]["maxLinearSpeedMps"] = math.inf
    elif fault == "negative_infinite_speed":
        config["navigation"]["maxLinearSpeedMps"] = -math.inf
    elif fault == "boolean_radius":
        config["geometry"]["wheelRadiusM"] = True
    elif fault == "duplicate_sensor":
        config["sensors"][1]["id"] = config["sensors"][0]["id"]
    elif fault == "duplicate_frame":
        config["sensors"][1]["frameId"] = config["sensors"][0]["frameId"]
    elif fault == "wrong_vector":
        config["sensors"][0]["pose"]["xyz"] = [0, 0]
    elif fault == "long_quaternion":
        config["sensors"][0]["pose"]["quaternion"] = [0, 0, 0, 1, 0]
    else:
        config["sensors"][0]["pose"]["quaternion"] = [0, 0, 0, 0]
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps(config), encoding="utf-8")
    with pytest.raises(ConfigError):
        load_and_validate(path)


@pytest.mark.parametrize("missing", ("vehicle_visual.obj", "vehicle_visual.mtl"))
def test_import_missing_visual_resource_preserves_previous_config(tmp_path, capsys, missing):
    export = tmp_path / "unity"
    export.mkdir()
    (export / "vehicle_config.json").write_text(json.dumps(_load()), encoding="utf-8")
    for name in ("vehicle_visual.obj", "vehicle_visual.mtl"):
        if name != missing:
            (export / name).write_text("fixture", encoding="utf-8")
    canonical = tmp_path / "current.json"
    canonical.write_text("original", encoding="utf-8")
    output = tmp_path / "generated"
    result = config_main(["import-unity", str(export), "--canonical", str(canonical),
                          "--output", str(output), "--workspace-src", str(WORKSPACE_SRC)])
    assert result == 2
    assert missing in capsys.readouterr().err
    assert canonical.read_text() == "original"
    assert not output.exists()


def test_front_and_rear_wheel_radii_are_preserved(tmp_path):
    config = _load()
    for wheel in config["geometry"]["wheelCenters"]:
        wheel["radiusM"] = 0.09 if wheel["id"].startswith("front") else 0.14
        wheel["widthM"] = 0.06
    for sdf in (generate_sdf(config), generate_simulation_sdf(config, WORKSPACE_SRC, tmp_path)):
        root = ET.fromstring(sdf)
        for wheel in config["geometry"]["wheelCenters"]:
            cylinder = root.find(f".//link[@name='{wheel['id']}_wheel_link']/collision/geometry/cylinder")
            assert float(cylinder.findtext("radius")) == pytest.approx(wheel["radiusM"])
            assert float(cylinder.findtext("length")) == pytest.approx(wheel["widthM"])


def test_nominal_cad_collision_proxies_do_not_occlude_simulated_lidar(tmp_path):
    config = _load()
    original = ET.fromstring(generate_simulation_sdf(config, WORKSPACE_SRC, tmp_path))
    assert original.find(".//plugin[@name='configured_sensor_collision_filter']") is None
    config.setdefault("metadata", {})["calibrationState"] = "unity_nominal_simulation_only"
    model = ET.fromstring(generate_simulation_sdf(config, WORKSPACE_SRC, tmp_path))
    plugin = model.find(".//plugin[@name='configured_sensor_collision_filter']")
    assert plugin.get("filename") == "libagribot_simulation_sensor_collision_filter.so"
    assert ET.tostring(model.find(".//link[@name='base_link']/collision")) == ET.tostring(
        original.find(".//link[@name='base_link']/collision"))
    urdf = ET.fromstring(generate_urdf(config))
    for sensor in config["sensors"]:
        assert urdf.find(f"./link[@name='{sensor['frameId']}']/visual") is not None


@pytest.mark.parametrize("vehicle_type", ["differential", "tracked"])
def test_differential_sdf_uses_four_wheel_drive_without_steering(tmp_path, vehicle_type):
    config = _load()
    config["vehicleType"] = vehicle_type
    for wheel in config["geometry"]["wheelCenters"]:
        wheel["steering"] = False
        wheel["driven"] = True
    (tmp_path / "vehicle_visual.obj").touch()
    for sdf in (generate_sdf(config), generate_simulation_sdf(config, WORKSPACE_SRC, tmp_path)):
        root = ET.fromstring(sdf)
        assert root.find(".//plugin[@name='ackermann_drive']") is None
        drive = root.find(".//plugin[@name='configured_diff_drive']")
        assert drive.get("filename") == "libgazebo_ros_diff_drive.so"
        assert drive.findtext("num_wheel_pairs") == "2"
        assert len(drive.findall("left_joint")) == len(drive.findall("right_joint")) == 2
        assert root.find(".//joint[@name='front_left_steering_joint']") is None
        for wheel in config["geometry"]["wheelCenters"]:
            link = root.find(f".//link[@name='{wheel['id']}_wheel_link']")
            assert root.find(f".//joint[@name='{wheel['id']}_wheel_joint']/parent").text == "base_link"
            if vehicle_type == "tracked":
                assert link.find("visual") is None


def test_nominal_diff_drive_can_disable_wheel_acceleration_integrator(tmp_path):
    config = _load()
    config["vehicleType"] = "tracked"
    for wheel in config["geometry"]["wheelCenters"]:
        wheel["steering"] = False
        wheel["driven"] = True
    original_acceleration = config["dynamics"]["maxAccelerationMps2"]
    config.setdefault("metadata", {})["simulationDiffDriveWheelAccelerationMps2"] = 0.0
    root = ET.fromstring(generate_simulation_sdf(config, WORKSPACE_SRC, tmp_path))
    drive = root.find(".//plugin[@name='configured_diff_drive']")
    assert float(drive.findtext("max_wheel_acceleration")) == 0.0
    assert config["dynamics"]["maxAccelerationMps2"] == original_acceleration
    config["metadata"]["simulationDiffDriveWheelAccelerationMps2"] = -1
    with pytest.raises(ValueError, match="nonnegative"):
        generate_simulation_sdf(config, WORKSPACE_SRC, tmp_path)


def test_ackermann_simulation_keeps_original_contact_friction(tmp_path):
    config = _load()
    root = ET.fromstring(generate_simulation_sdf(config, WORKSPACE_SRC, tmp_path))
    assert root.find(".//plugin[@name='configured_diff_drive']") is None
    for wheel in config["geometry"]["wheelCenters"]:
        contact = root.find(f".//link[@name='{wheel['id']}_wheel_link']/collision/surface/friction/ode")
        assert float(contact.findtext("mu")) == 1.8
        assert float(contact.findtext("mu2")) == 1.5


def test_nominal_skid_contact_uses_axle_aligned_lateral_friction(tmp_path):
    config = _load()
    config["vehicleType"] = "tracked"
    for wheel in config["geometry"]["wheelCenters"]:
        wheel["steering"] = False
        wheel["driven"] = True
    config.setdefault("metadata", {})["simulationSkidContact"] = {
        "lateralFriction": .1, "longitudinalFriction": 1.8, "frontLateralFriction": .03,
        "lateralSlipMpsPerN": .0001}
    root = ET.fromstring(generate_simulation_sdf(config, WORKSPACE_SRC, tmp_path))
    for wheel in config["geometry"]["wheelCenters"]:
        contact = root.find(f".//link[@name='{wheel['id']}_wheel_link']/collision/surface/friction/ode")
        assert contact.findtext("fdir1") == "0 0 1"
        assert float(contact.findtext("mu")) == (.03 if wheel["id"].startswith("front_") else .1)
        assert float(contact.findtext("mu2")) == 1.8
        assert float(contact.findtext("slip1")) == .0001
    for invalid in (-1, float("nan"), True):
        config["metadata"]["simulationSkidContact"]["lateralSlipMpsPerN"] = invalid
        with pytest.raises(ValueError, match="slip must"):
            generate_simulation_sdf(config, WORKSPACE_SRC, tmp_path)
    config["metadata"]["simulationSkidContact"]["lateralSlipMpsPerN"] = .0001
    config["metadata"]["simulationSkidContact"]["lateralFriction"] = -1
    with pytest.raises(ValueError, match="positive"):
        generate_simulation_sdf(config, WORKSPACE_SRC, tmp_path)


def test_multiple_mesh_objects_per_wheel_keep_valid_local_indices(tmp_path):
    source = tmp_path / "vehicle_visual.obj"
    lines = []
    index = 1
    for name in ("body", "FLwheel", "FRwheel", "RLwheel", "RRwheel"):
        for part in range(2):
            lines.extend([f"o {name}__{part}\n", "v 0 0 0\n", "v 1 0 0\n", "v 0 1 0\n",
                          f"f {index} {index + 1} {index + 2}\n"])
            index += 3
    source.write_text("".join(lines))
    output = tmp_path / "split"
    output.mkdir()
    _split_integrated_visual_obj(source, output)
    for wheel in _load()["geometry"]["wheelCenters"]:
        text = (output / f"{wheel['id']}_wheel_visual.obj").read_text()
        assert "f 1 2 3" in text
        assert "f 4 5 6" in text


def test_unity_empty_measurement_frame_is_treated_as_absent(tmp_path):
    config = _load()
    imu = next(sensor for sensor in config["sensors"] if sensor["id"] == "imu")
    imu["measurementFrameId"] = ""
    imu["measurementFramePose"] = {
        "xyz": [0.0, 0.0, 0.0],
        "quaternion": [0.0, 0.0, 0.0, 1.0],
        "rpy": [0.0, 0.0, 0.0],
    }
    path = tmp_path / "unity.json"
    path.write_text(json.dumps(config), encoding="utf-8")

    loaded, warnings = load_and_validate(path)

    assert warnings == []
    assert next(sensor for sensor in loaded["sensors"] if sensor["id"] == "imu")["frameId"] == "imu_link"


def test_generated_bundle_is_deterministic_and_parseable(tmp_path):
    config = _load()
    first = tmp_path / "first"
    second = tmp_path / "second"
    generate_bundle(config, BASELINE, first)
    generate_bundle(copy.deepcopy(config), BASELINE, second)

    first_files = sorted(path.relative_to(first) for path in first.rglob("*") if path.is_file())
    second_files = sorted(path.relative_to(second) for path in second.rglob("*") if path.is_file())
    assert first_files == second_files
    for relative in first_files:
        assert (first / relative).read_bytes() == (second / relative).read_bytes()

    yaml.safe_load((first / "config" / "sensor_mounts.yaml").read_text())
    yaml.safe_load((first / "config" / "nav2_geometry.yaml").read_text())
    assert (first / "urdf" / "ackermann_current.urdf.xacro").read_text().startswith(
        "<?xml version=\"1.0\"?>"
    )
    assert (first / "models" / "ackermann_current.sdf").read_text().startswith(
        '<sdf version="1.7">'
    )


def test_generated_runtime_bundle_uses_tuned_templates(tmp_path):
    config = _load()
    output = tmp_path / "runtime"

    manifest = generate_bundle(config, BASELINE, output, WORKSPACE_SRC)

    assert manifest["runtimeTemplatesApplied"] is True
    nav = yaml.safe_load(
        (output / "config/physical/nav2_params_ackermann_fastlio_mapped.yaml").read_text()
    )
    follow_path = nav["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow_path["plugin"] == "nav2_mppi_controller::MPPIController"
    assert follow_path["vx_max"] == pytest.approx(0.3)
    assert follow_path["AckermannConstraints"]["min_turning_r"] == pytest.approx(
        config["geometry"]["minTurningRadiusM"]
    )
    fastlivo = yaml.safe_load(
        (output / "config/physical/agribot_c16_astra.yaml").read_text()
    )
    assert fastlivo["/**"]["ros__parameters"]["vio"]["img_point_cov"] == 200
    assert fastlivo["/**"]["ros__parameters"]["extrin_calib"]["Pcl"] == pytest.approx(
        [0.000000122800367, -0.079002376748972, -0.095500094196799], abs=2e-6
    )
    sdf = ET.parse(output / "models/ackermann_current.sdf").getroot()
    assert sdf.find(".//sensor[@name='n300pro_imu']") is not None
    assert float(
        sdf.find(".//plugin[@name='ackermann_drive']/wheelbase").text
    ) == pytest.approx(config["geometry"]["wheelbaseM"])

    simulated_nav = yaml.safe_load(
        (output / "config/simulation/nav2_params_ackermann_fastlio_static.yaml").read_text()
    )
    assert simulated_nav["planner_server"]["ros__parameters"]["GridBased"][
        "max_planning_time"
    ] == pytest.approx(20.0)
    lidar_z = next(
        sensor for sensor in config["sensors"] if sensor["id"] == "lidar"
    )["pose"]["xyz"][2]
    for costmap_name in ("global_costmap", "local_costmap"):
        marking = simulated_nav[costmap_name][costmap_name]["ros__parameters"][
            "stvl_layer"
        ]["lidar_mark"]
        assert marking["min_obstacle_height"] == pytest.approx(lidar_z + 0.02)


def test_integrated_unity_visual_replaces_placeholder_visuals(tmp_path):
    config = _load()
    unity_export = tmp_path / "unity_export"
    unity_export.mkdir()
    (unity_export / "vehicle_visual.obj").write_text("# test\n", encoding="utf-8")

    urdf = ET.fromstring(generate_urdf(config))
    assert (
        urdf.find("./link[@name='base_link']/visual[@name='vehicle_body']")
        is not None
    )
    assert urdf.find("./link[@name='imu_link']/visual") is None
    wheel_mesh = urdf.find(
        "./link[@name='front_left_wheel_link']/visual/geometry/mesh"
    )
    assert wheel_mesh is not None
    assert wheel_mesh.get("filename").endswith("/front_left_wheel_visual.obj")
    assert urdf.find("./link[@name='front_left_wheel_link']/collision") is not None

    sdf = ET.fromstring(generate_simulation_sdf(config, WORKSPACE_SRC, unity_export))
    base = sdf.find("./model/link[@name='base_link']")
    assert [visual.get("name") for visual in base.findall("visual")] == [
        "vehicle_body_visual"
    ]
    wheel_uri = sdf.find(
        ".//link[@name='front_left_wheel_link']/visual/geometry/mesh/uri"
    )
    assert wheel_uri is not None
    assert wheel_uri.text.endswith("/front_left_wheel_visual.obj")
    assert sdf.find(".//link[@name='front_left_wheel_link']/collision") is not None


def test_unity_obj_is_split_into_body_and_joint_local_wheels(tmp_path):
    source = tmp_path / "unity" / "vehicle_visual.obj"
    source.parent.mkdir()
    lines = ["# test\n", "mtllib vehicle_visual.mtl\n"]
    objects = [
        ("body", (0.0, 0.0, 0.0)),
        ("FLwheel", (1.0, 2.0, 3.0)),
        ("FRwheel", (1.0, -2.0, 3.0)),
        ("RLwheel", (-1.0, 2.0, 3.0)),
        ("RRwheel", (-1.0, -2.0, 3.0)),
    ]
    vertex_index = 1
    for name, center in objects:
        lines.append(f"o {name}\n")
        for offset in ((-0.1, -0.2, -0.3), (0.1, -0.2, 0.3), (0.1, 0.2, -0.3)):
            lines.append(
                "v "
                + " ".join(str(value + delta) for value, delta in zip(center, offset))
                + "\n"
            )
        lines.extend(("vt 0 0\n", "vt 1 0\n", "vt 1 1\n"))
        lines.extend(("vn 0 1 0\n",) * 3)
        lines.append("usemtl test_material\n")
        lines.append(
            f"f {vertex_index}/{vertex_index}/{vertex_index} "
            f"{vertex_index + 1}/{vertex_index + 1}/{vertex_index + 1} "
            f"{vertex_index + 2}/{vertex_index + 2}/{vertex_index + 2}\n"
        )
        vertex_index += 3
    source.write_text("".join(lines), encoding="utf-8")

    output = tmp_path / "models"
    _split_integrated_visual_obj(source, output)

    body = (output / "vehicle_visual.obj").read_text(encoding="utf-8")
    assert body.count("\nf ") == 1
    for wheel_id in ("front_left", "front_right", "rear_left", "rear_right"):
        wheel = (output / f"{wheel_id}_wheel_visual.obj").read_text(
            encoding="utf-8"
        )
        assert wheel.count("\nf ") == 1
        assert "f 1/1/1 2/2/2 3/3/3" in wheel
        vertices = [
            [float(value) for value in line.split()[1:4]]
            for line in wheel.splitlines()
            if line.startswith("v ")
        ]
        for axis in range(3):
            minimum = min(vertex[axis] for vertex in vertices)
            maximum = max(vertex[axis] for vertex in vertices)
            assert minimum + maximum == pytest.approx(0.0)


def test_obj_visual_lod_reduces_faces_and_preserves_material(tmp_path):
    mesh = tmp_path / "dense.obj"
    lines = ["mtllib vehicle_visual.mtl\n", "o panel\n", "usemtl blue\n"]
    grid_size = 6
    for y in range(grid_size):
        for x in range(grid_size):
            lines.append(f"v {x * 0.004} {y * 0.004} 0\n")
    lines.extend("vn 0 0 1\n" for _ in range(grid_size * grid_size))
    input_faces = 0
    for y in range(grid_size - 1):
        for x in range(grid_size - 1):
            first = y * grid_size + x + 1
            right = first + 1
            upper = first + grid_size
            upper_right = upper + 1
            lines.append(f"f {first}//{first} {right}//{right} {upper}//{upper}\n")
            lines.append(
                f"f {right}//{right} {upper_right}//{upper_right} {upper}//{upper}\n"
            )
            input_faces += 2
    mesh.write_text("".join(lines), encoding="utf-8")

    stats = _simplify_obj_vertex_clusters(mesh, grid_size_m=0.01)
    simplified = mesh.read_text(encoding="utf-8")

    assert 0 < stats["outputFaces"] < input_faces
    assert stats["outputVertices"] < grid_size * grid_size
    assert "mtllib vehicle_visual.mtl" in simplified
    assert "usemtl blue" in simplified
    assert simplified.count("\nf ") == stats["outputFaces"]


def test_small_scale_visuals_can_keep_millimetre_detail_without_changing_default_lod(tmp_path):
    config = _load()
    config["vehicleType"] = "tracked"
    for wheel in config["geometry"]["wheelCenters"]:
        wheel["steering"] = False
    (tmp_path/"vehicle_visual.obj").write_text("v 0 0 0\nv .005 0 0\nv 0 .005 0\nf 1 2 3\n")
    config["metadata"]["simulationBodyLodGridM"] = .002
    manifest = generate_bundle(config, tmp_path/"vehicle_config.json", tmp_path/"fine")
    assert manifest["visualLod"]["bodyGridM"] == .002
    assert manifest["visualLod"]["body"]["outputFaces"] == 1
    for invalid in (True, 0., .1, float("nan")):
        config["metadata"]["simulationBodyLodGridM"] = invalid
        with pytest.raises(ValueError, match="simulationBodyLodGridM"):
            generate_bundle(config, tmp_path/"vehicle_config.json", tmp_path/"invalid")


def test_current_ackermann_runtime_verifier_covers_expected_surface():
    results = verify_current(_load(), WORKSPACE_SRC)
    assert len(results) >= 40
    names = {result.name for result in results}
    assert {
        "chassis.wheelbase_m",
        "nav2.min_turning_radius",
        "fastlivo.extrinsic_T",
        "simulation.wheelbase",
        "simulation_sensor.lidar.pose",
    }.issubset(names)


def _write_sensor_cad(directory):
    for sensor_id in ("imu", "lidar", "right_camera", "rtk_master"):
        name = f"sensor_{sensor_id}_visual"
        (directory / f"{name}.obj").write_text(
            f"mtllib {name}.mtl\nv 0 0 0\nv 0.01 0 0\nv 0 0.01 0\nusemtl blue\nf 1 2 3\n",
            encoding="utf-8",
        )
        (directory / f"{name}.mtl").write_text("newmtl blue\nKd 0 0 1\n", encoding="utf-8")


def test_sensor_cad_is_visible_in_urdf_and_sdf_without_duplicate_placeholders(tmp_path):
    config = _load()
    _write_sensor_cad(tmp_path)
    source = tmp_path / "vehicle_config.json"
    source.write_text(json.dumps(config), encoding="utf-8")
    output = tmp_path / "bundle"
    generate_bundle(config, source, output)
    urdf = ET.parse(output / "urdf" / "ackermann_current.urdf.xacro")
    for sensor in ("imu", "lidar", "right_camera", "rtk_master"):
        item = next(item for item in config["sensors"] if item["id"] == sensor)
        visual = urdf.find(f"./link[@name='{item['frameId']}']/visual/geometry/mesh")
        assert visual.get("filename").endswith(f"/sensor_{sensor}_visual.obj")
        assert (output / "models" / f"sensor_{sensor}_visual.mtl").is_file()
    assert urdf.find("./link[@name='rtk_secondary_link']/visual") is None
    sdf = ET.parse(output / "models" / "ackermann_current.sdf")
    assert len(sdf.findall(".//visual/plugin[@filename='libagribot_simulation_sensor_visual.so']")) == 4


def test_runtime_sensor_cad_visual_poses_match_mounts(tmp_path):
    config = _load()
    _write_sensor_cad(tmp_path)
    (tmp_path / "vehicle_visual.obj").write_text("# body\n", encoding="utf-8")
    sdf = ET.fromstring(generate_simulation_sdf(config, WORKSPACE_SRC, tmp_path))
    base = sdf.find("./model/link[@name='base_link']")
    assert len(base.findall("visual")) == 5
    for sensor in ("imu", "lidar", "right_camera", "rtk_master"):
        item = next(item for item in config["sensors"] if item["id"] == sensor)
        values = [float(value) for value in base.find(f"visual[@name='{sensor}_cad_visual']/pose").text.split()]
        assert values[:3] == pytest.approx(item["pose"]["xyz"])


def test_sensor_cad_requires_material_sidecar(tmp_path):
    (tmp_path / "sensor_lidar_visual.obj").write_text("# incomplete\n", encoding="utf-8")
    with pytest.raises(ValueError, match="Incomplete sensor CAD"):
        generate_simulation_sdf(_load(), WORKSPACE_SRC, tmp_path)
