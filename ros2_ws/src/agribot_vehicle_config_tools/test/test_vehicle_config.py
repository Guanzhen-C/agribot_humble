import copy
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from agribot_vehicle_config_tools.generator import (
    _split_integrated_visual_obj,
    generate_bundle,
    generate_simulation_sdf,
    generate_urdf,
)
from agribot_vehicle_config_tools.model import (
    ConfigError,
    fastlivo_extrinsics,
    footprint,
    load_and_validate,
)
from agribot_vehicle_config_tools.verifier import verify_current


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
    assert extrinsics["Pcl"] == pytest.approx([0.0, -0.079, -0.0955], abs=2e-6)


def test_inconsistent_turning_radius_is_rejected(tmp_path):
    config = _load()
    config["geometry"]["minTurningRadiusM"] = 2.0
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps(config), encoding="utf-8")
    with pytest.raises(ConfigError, match="minTurningRadiusM"):
        load_and_validate(path)


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
        [0.0, -0.079, -0.0955], abs=2e-6
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
