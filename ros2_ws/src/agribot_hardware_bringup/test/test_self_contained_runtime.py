import json
import re
import struct
import xml.etree.ElementTree as element_tree
from pathlib import Path

import yaml

PACKAGE_ROOT = Path(__file__).parents[1]
FORBIDDEN_PROJECT_PACKAGES = {
    "agribot_ackermann_mppi",
    "agribot_autonomy",
    "agribot_rl_nav",
    "scout_base",
    "scout_navigation",
}


def runtime_text_files():
    yield PACKAGE_ROOT / "CMakeLists.txt"
    yield PACKAGE_ROOT / "package.xml"
    yield from (PACKAGE_ROOT / "launch").rglob("*.py")
    yield from (PACKAGE_ROOT / "differential" / "launch").rglob("*.py")
    yield from (PACKAGE_ROOT / "ackermann" / "launch").rglob("*.py")


def test_runtime_has_no_reference_to_removed_project_packages():
    for path in runtime_text_files():
        source = path.read_text()
        for package in FORBIDDEN_PROJECT_PACKAGES:
            assert package not in source, f"{path} still references {package}"


def test_all_agribot_nodes_are_provided_by_this_package():
    node_pattern = re.compile(r'package\s*=\s*"(agribot_[^"]+)"')
    for path in (PACKAGE_ROOT / "launch").rglob("*.py"):
        for package in node_pattern.findall(path.read_text()):
            assert package == "agribot_hardware_bringup"


def test_localization_sources_are_built_without_sibling_source_paths():
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text()
    assert "localization/navsat/src/rtk_eskf_localization_node.cpp" in cmake
    assert "localization/navsat/src/rtk_gi_engine.cpp" in cmake
    assert "localization/navsat/third_party/kf_gins/kf-gins/insmech.cpp" in cmake
    assert "../KF-GINS" not in cmake
    assert "/home/" not in cmake


def test_vehicle_launch_files_are_only_installed_at_top_level():
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text()
    assert "differential/launch\n  DESTINATION" not in cmake
    assert "ackermann/launch\n  DESTINATION" not in cmake
    assert "install(DIRECTORY differential/launch/" in cmake
    assert "install(DIRECTORY ackermann/launch/" in cmake


def test_migrated_runtime_resources_exist_and_parse():
    expected = (
        "localization/navsat/scripts/navsat_pose_bridge.py",
        "localization/fastlio/scripts/fastlio_odom_bridge.py",
        "ackermann/config/nav2_params_ackermann_navsat_static.yaml",
        "ackermann/config/nav2_params_ackermann_fastlio_mapped.yaml",
        "ackermann/config/nav2_params_ackermann_fastlio_local.yaml",
        "ackermann/launch/ackermann_mppi_fastlio_mapped.launch.py",
        "ackermann/launch/ackermann_mppi_fastlio_3d_mapping.launch.py",
        "config/pcd_initial_localization.yaml",
        "config/pcd_mapping.yaml",
        "localization/navsat/include/agribot_hardware_bringup/navsat_frame_conversions.hpp",
        "localization/pcd/src/pcd_initial_localizer.cpp",
        "localization/pcd/src/pcd_map_builder.cpp",
        "meshes/ackermann_vehicle_body.glb",
        "meshes/ackermann_front_left_wheel.glb",
        "meshes/ackermann_front_right_wheel.glb",
        "meshes/ackermann_rear_left_wheel.glb",
        "meshes/ackermann_rear_right_wheel.glb",
        "meshes/hipnuc_n300pro.glb",
        "meshes/lslidar_c16_v4.glb",
        "rviz/navigation_local.rviz",
        "rviz/pcd_mapping.rviz",
        "urdf/ackermann_vehicle.urdf",
        "scripts/pcd_to_nav2_map.py",
        "scripts/start_wheeltec_car_gui.sh",
        "scripts/wheeltec_car_gui.py",
        "desktop/wheeltec-car-gui.desktop",
    )
    for relative_path in expected:
        assert (PACKAGE_ROOT / relative_path).is_file()

    obsolete = (
        "ackermann/config/nav2_params_ackermann_fastlio_static.yaml",
        "ackermann/launch/ackermann_mppi_fastlio.launch.py",
        "ackermann/launch/ackermann_mppi_fastlio_localization.launch.py",
        "ackermann/launch/ackermann_mppi_fastlio_mapping.launch.py",
        "config/pointcloud_to_laserscan_mapping.yaml",
        "config/slam_toolbox_mapping_c16.yaml",
        "config/slam_toolbox_localization_c16.yaml",
        "config/slam_toolbox_c16.yaml",
        "config/pcd_localization.yaml",
        "localization/pcd/src/pcd_global_localizer.cpp",
        "config/mrpt_pf_localization.yaml",
        "config/mrpt_relocalization_pipeline.yaml",
        "localization/pcd/src/pointcloud_tf_adapter.cpp",
    )
    for relative_path in obsolete:
        assert not (PACKAGE_ROOT / relative_path).exists()

    for path in (PACKAGE_ROOT / "ackermann" / "behavior_trees").glob("*.xml"):
        element_tree.parse(path)


def test_navsat_wrapper_converts_kf_gins_state_to_rear_axle_base_link():
    source = (
        PACKAGE_ROOT
        / "localization/navsat/src/rtk_eskf_localization_node.cpp"
    ).read_text()
    assert "navsat_frames::fluToFrd(antenna_lever_flu_m_)" in source
    assert "imuMapPositionToBaseMapPosition" in source
    assert "imuMapVelocityToBaseFluVelocity" in source
    assert "imuPoseCovarianceToBaseMapPoseCovariance" in source
    assert "independentImuTwistCovarianceToBaseFlu" in source
    assert "baseMapPositionToSensorMapPosition" in source
    assert "addEligiblePoseMeasurement" in source
    assert "if (tryInitializeEngine())" in source
    assert "resetFusionAfterImuDiscontinuity" in source
    assert "candidate.time + time_tolerance_sec" in source
    assert "has_parameter(name)" in source
    assert "last_used_rtk_heading_time_" in source
    assert "navsat_frames::shouldUseRtkHeading" in source
    assert "handleRtkHeadingWithCovariance" in source
    assert "latest_rtk_heading_std_rad_" in source
    assert "use_pose_yaw_measurement_ && sample.has_yaw" in source
    assert "Ignoring /initialpose" in source


def test_ackermann_behavior_trees_never_request_backup():
    for path in (PACKAGE_ROOT / "ackermann" / "behavior_trees").glob("*.xml"):
        tree = element_tree.parse(path)
        assert not tree.findall(".//BackUp")


def test_rviz_goal_tool_sends_nav2_action_directly():
    for name in ("navigation.rviz", "navigation_local.rviz"):
        config = (PACKAGE_ROOT / "rviz" / name).read_text()
        assert "Class: nav2_rviz_plugins/GoalTool" in config
        assert "Class: nav2_rviz_plugins/Navigation 2" in config
        assert "Class: rviz_default_plugins/SetGoal" not in config


def test_navigation_rviz_does_not_render_3d_clouds_or_legacy_scan():
    config = (PACKAGE_ROOT / "rviz" / "navigation.rviz").read_text()
    assert "Class: rviz_default_plugins/LaserScan" not in config
    assert "/scan_mapping" not in config
    assert re.search(
        r"Class: rviz_default_plugins/PointCloud2"
        r"(?:(?!Class:).)*Enabled: false"
        r"(?:(?!Class:).)*Name: C16 Obstacle Input",
        config,
        re.DOTALL,
    )


def test_physical_vehicle_rviz_profiles_display_vehicle_and_sensor_axes():
    for name in ("navigation.rviz", "navigation_local.rviz", "pcd_mapping.rviz"):
        config = (PACKAGE_ROOT / "rviz" / name).read_text()
        assert "Class: rviz_default_plugins/RobotModel" in config
        assert "Name: Physical Ackermann Vehicle" in config
        assert "Value: /robot_description" in config
        assert "Name: N300 Pro IMU Axes" in config
        assert "Reference Frame: imu_link" in config
        assert "Name: C16 Lidar Axes" in config
        assert "Reference Frame: lidar_link" in config

    mapped_config = (PACKAGE_ROOT / "rviz" / "navigation.rviz").read_text()
    assert "Class: rviz_default_plugins/PoseWithCovariance" in mapped_config
    assert "Name: Initial Localized Pose" in mapped_config
    assert "Value: /localization_pose" in mapped_config
    assert "Class: rviz_default_plugins/PoseArray" not in mapped_config
    assert "/particlecloud" not in mapped_config

    for name in ("navigation_local.rviz", "pcd_mapping.rviz"):
        config = (PACKAGE_ROOT / "rviz" / name).read_text()
        assert "Class: rviz_default_plugins/PoseWithCovariance" not in config

    sensor_config = (PACKAGE_ROOT / "rviz" / "sensors.rviz").read_text()
    assert "Name: N300 Pro IMU Axes" in sensor_config
    assert "Name: C16 Lidar Axes" in sensor_config


def test_ackermann_vehicle_urdf_uses_rear_axle_centered_mesh():
    urdf_path = PACKAGE_ROOT / "urdf" / "ackermann_vehicle.urdf"
    tree = element_tree.parse(urdf_path)
    root = tree.getroot()
    assert root.find("./link[@name='base_link']") is not None
    body_visual = root.find("./link[@name='base_link']/visual[@name='step_vehicle_body']")
    mesh = body_visual.find("geometry/mesh")
    assert mesh is not None
    assert mesh.attrib["filename"] == (
        "package://agribot_hardware_bringup/meshes/ackermann_vehicle_body.glb"
    )
    assert "scale" not in mesh.attrib

    origin = body_visual.find("origin")
    assert origin is not None
    assert origin.attrib["xyz"] == "0.2794632 0.1918975 -0.7587717"
    assert origin.attrib["rpy"] == "0 0 1.5707963267948966"

    # A URDF material would override the colors embedded in the STEP-derived GLB.
    assert body_visual.find("material") is None


def test_ackermann_vehicle_urdf_has_steering_and_rolling_wheel_joints():
    root = element_tree.parse(
        PACKAGE_ROOT / "urdf" / "ackermann_vehicle.urdf"
    ).getroot()

    for side in ("left", "right"):
        steering = root.find(f"./joint[@name='front_{side}_steering_joint']")
        assert steering is not None
        assert steering.attrib["type"] == "revolute"
        assert steering.find("axis").attrib["xyz"] == "0 0 1"
        assert steering.find("parent").attrib["link"] == "base_link"
        assert steering.find("child").attrib["link"] == f"front_{side}_steering_link"

        wheel = root.find(f"./joint[@name='front_{side}_wheel_joint']")
        assert wheel is not None
        assert wheel.attrib["type"] == "continuous"
        assert wheel.find("axis").attrib["xyz"] == "0 1 0"
        assert wheel.find("parent").attrib["link"] == f"front_{side}_steering_link"

    for side in ("left", "right"):
        wheel = root.find(f"./joint[@name='rear_{side}_wheel_joint']")
        assert wheel is not None
        assert wheel.attrib["type"] == "continuous"
        assert wheel.find("axis").attrib["xyz"] == "0 1 0"
        assert wheel.find("parent").attrib["link"] == "base_link"

    assert not (PACKAGE_ROOT / "meshes" / "ackermann_vehicle.glb").exists()


def test_ackermann_vehicle_urdf_places_calibrated_sensor_models():
    urdf_path = PACKAGE_ROOT / "urdf" / "ackermann_vehicle.urdf"
    root = element_tree.parse(urdf_path).getroot()
    mounts = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "sensor_mounts.yaml").read_text()
    )
    visuals = {
        visual.attrib.get("name"): visual
        for visual in root.findall("./link[@name='base_link']/visual")
    }

    imu = visuals["hipnuc_n300pro"]
    assert imu.find("geometry/mesh").attrib["filename"] == (
        "package://agribot_hardware_bringup/meshes/hipnuc_n300pro.glb"
    )
    assert [float(value) for value in imu.find("origin").attrib["xyz"].split()] == (
        mounts["imu"]["xyz"]
    )
    assert [float(value) for value in imu.find("origin").attrib["rpy"].split()] == (
        mounts["imu"]["rpy"]
    )

    lidar = visuals["lslidar_c16_v4"]
    assert lidar.find("geometry/mesh").attrib["filename"] == (
        "package://agribot_hardware_bringup/meshes/lslidar_c16_v4.glb"
    )
    lidar_visual_xyz = [
        float(value) for value in lidar.find("origin").attrib["xyz"].split()
    ]
    assert lidar_visual_xyz == [
        mounts["lidar"]["xyz"][0],
        mounts["lidar"]["xyz"][1],
        mounts["lidar"]["xyz"][2] - 0.04855,
    ]
    assert lidar.find("origin").attrib["rpy"] == "0 0 3.141592653589793"


def test_ackermann_vehicle_mesh_retains_step_detail_and_materials():
    mesh_names = (
        "ackermann_vehicle_body.glb",
        "ackermann_front_left_wheel.glb",
        "ackermann_front_right_wheel.glb",
        "ackermann_rear_left_wheel.glb",
        "ackermann_rear_right_wheel.glb",
    )
    total_triangle_count = 0
    body_document = None

    for mesh_name in mesh_names:
        mesh_path = PACKAGE_ROOT / "meshes" / mesh_name
        with mesh_path.open("rb") as stream:
            magic, version, total_length = struct.unpack("<4sII", stream.read(12))
            json_length, json_type = struct.unpack("<II", stream.read(8))
            document = json.loads(stream.read(json_length))

        assert magic == b"glTF"
        assert version == 2
        assert total_length == mesh_path.stat().st_size
        assert json_type == 0x4E4F534A
        assert len(document["meshes"]) <= 40

        for mesh in document["meshes"]:
            for primitive in mesh["primitives"]:
                assert primitive.get("mode", 4) == 4
                index_accessor = document["accessors"][primitive["indices"]]
                total_triangle_count += index_accessor["count"] // 3

        if mesh_name == "ackermann_vehicle_body.glb":
            body_document = document

    materials = body_document["materials"]
    colors = {
        tuple(
            material.get("pbrMetallicRoughness", {}).get(
                "baseColorFactor", [1.0, 1.0, 1.0, 1.0]
            )
        )
        for material in materials
    }
    assert len(materials) >= 30
    assert len(colors) >= 30
    assert 600_000 <= total_triangle_count <= 660_000


def test_sensor_meshes_match_n300pro_and_c16_scale():
    def inspect_glb(relative_path):
        mesh_path = PACKAGE_ROOT / relative_path
        with mesh_path.open("rb") as stream:
            magic, version, total_length = struct.unpack("<4sII", stream.read(12))
            json_length, json_type = struct.unpack("<II", stream.read(8))
            document = json.loads(stream.read(json_length))
        assert magic == b"glTF"
        assert version == 2
        assert total_length == mesh_path.stat().st_size
        assert json_type == 0x4E4F534A

        lower = [float("inf")] * 3
        upper = [float("-inf")] * 3
        triangle_count = 0
        for mesh in document["meshes"]:
            for primitive in mesh["primitives"]:
                position = document["accessors"][primitive["attributes"]["POSITION"]]
                lower = [min(old, new) for old, new in zip(lower, position["min"])]
                upper = [max(old, new) for old, new in zip(upper, position["max"])]
                indices = document["accessors"][primitive["indices"]]
                triangle_count += indices["count"] // 3
        return document, [high - low for low, high in zip(lower, upper)], triangle_count

    n300, n300_size, n300_triangles = inspect_glb("meshes/hipnuc_n300pro.glb")
    assert len(n300["materials"]) >= 5
    assert 0.0265 <= n300_size[0] <= 0.028
    assert 0.024 <= n300_size[1] <= 0.025
    assert 0.0145 <= n300_size[2] <= 0.016
    assert n300_triangles < 2_000

    c16, c16_size, c16_triangles = inspect_glb("meshes/lslidar_c16_v4.glb")
    assert len(c16["materials"]) >= 1
    assert 0.101 <= c16_size[0] <= 0.103
    assert 0.116 <= c16_size[1] <= 0.118
    assert 0.077 <= c16_size[2] <= 0.079
    assert 100_000 <= c16_triangles <= 140_000


def test_mapping_rviz_displays_voxelized_pcd_map():
    config = (PACKAGE_ROOT / "rviz" / "pcd_mapping.rviz").read_text()
    assert "Name: 3D PCD Map" in config
    assert "Value: /pcd_map" in config
    assert "Durability Policy: Transient Local" in config


def test_simulation_orchard_map_is_not_bundled_or_defaulted():
    assert not (PACKAGE_ROOT / "maps" / "orchard_v2_map6.yaml").exists()
    assert not (PACKAGE_ROOT / "maps" / "orchard_v2_map6.pgm").exists()
    for path in runtime_text_files():
        assert "orchard_v2_map6" not in path.read_text()


def test_kf_gins_subset_keeps_its_license_and_attribution():
    third_party = PACKAGE_ROOT / "localization" / "navsat" / "third_party" / "kf_gins"
    assert (third_party / "LICENSE").is_file()
    assert "GNU GENERAL PUBLIC LICENSE" in (third_party / "LICENSE").read_text()
    assert "i2Nav-WHU/KF-GINS" in (third_party / "README.md").read_text()
