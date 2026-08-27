import json
import math
import xml.etree.ElementTree as element_tree
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
FASTLIVO_ROOT = PACKAGE_ROOT.parent / "FAST-LIVO2"
URDF_ROOT = element_tree.parse(
    PACKAGE_ROOT / "urdf" / "ackermann_vehicle.urdf"
).getroot()


def joint_origin_x(name):
    joint = URDF_ROOT.find(f"./joint[@name='{name}']")
    return float(joint.find("origin").attrib["xyz"].split()[0])


LEFT_WHEELBASE = joint_origin_x("front_left_steering_joint") - joint_origin_x(
    "rear_left_wheel_joint"
)
RIGHT_WHEELBASE = joint_origin_x("front_right_steering_joint") - joint_origin_x(
    "rear_right_wheel_joint"
)
MODEL_WHEELBASE = (LEFT_WHEELBASE + RIGHT_WHEELBASE) * 0.5
MODEL_MAX_STEERING = float(
    URDF_ROOT.find("./joint[@name='front_left_steering_joint']/limit").attrib[
        "upper"
    ]
)
MODEL_MIN_TURNING_RADIUS = MODEL_WHEELBASE / math.tan(MODEL_MAX_STEERING)
SAFETY_FOOTPRINT = [
    [0.754818, 0.485974],
    [0.754818, -0.485974],
    [-0.227500, -0.485974],
    [-0.227500, 0.485974],
]


def load_config(name, vehicle=None):
    directory = PACKAGE_ROOT / vehicle if vehicle else PACKAGE_ROOT
    with (directory / "config" / name).open() as stream:
        return yaml.safe_load(stream)


def rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def transpose(matrix):
    return [list(row) for row in zip(*matrix)]


def matmul(left, right):
    return [
        [sum(left[row][k] * right[k][column] for k in range(3)) for column in range(3)]
        for row in range(3)
    ]


def matvec(matrix, vector):
    return [sum(row[index] * vector[index] for index in range(3)) for row in matrix]


def flatten(matrix):
    return [value for row in matrix for value in row]


def test_lidar_imu_extrinsics_are_consistent_across_physical_runtime_configs():
    mounts = load_config("sensor_mounts.yaml")
    fastlio = load_config("fast_lio_c16.yaml")["/**"]["ros__parameters"]
    bridge = load_config("fastlio_bridge.yaml")["fastlio_odom_bridge"][
        "ros__parameters"
    ]
    eskf = load_config("kf_gins_n300pro.yaml")["rtk_eskf_localization"][
        "ros__parameters"
    ]

    base_from_imu = rpy_matrix(mounts["imu"]["rpy"])
    base_from_lidar = rpy_matrix(mounts["lidar"]["rpy"])
    imu_from_lidar = matmul(transpose(base_from_imu), base_from_lidar)
    base_delta = [
        mounts["lidar"]["xyz"][index] - mounts["imu"]["xyz"][index]
        for index in range(3)
    ]

    assert fastlio["mapping"]["extrinsic_R"] == pytest.approx(
        flatten(imu_from_lidar), abs=1.0e-8
    )
    assert fastlio["mapping"]["extrinsic_T"] == pytest.approx(
        matvec(transpose(base_from_imu), base_delta), abs=1.0e-8
    )
    assert bridge["base_to_body_xyz"] == pytest.approx(mounts["imu"]["xyz"])
    assert bridge["base_to_body_rpy"] == pytest.approx(mounts["imu"]["rpy"])
    assert eskf["base_to_imu_rpy_rad"] == pytest.approx(mounts["imu"]["rpy"])


def test_fastlivo_camera_extrinsics_match_physical_mounts():
    mounts = load_config("sensor_mounts.yaml")
    fastlivo = yaml.safe_load(
        (FASTLIVO_ROOT / "config" / "agribot_c16_astra.yaml").read_text()
    )["/**"]["ros__parameters"]["extrin_calib"]

    base_from_lidar = rpy_matrix(mounts["lidar"]["rpy"])
    base_from_camera = rpy_matrix(mounts["camera"]["rpy"])
    camera_from_lidar = matmul(transpose(base_from_camera), base_from_lidar)
    lidar_minus_camera = [
        lidar - camera
        for lidar, camera in zip(
            mounts["lidar"]["xyz"], mounts["camera"]["xyz"]
        )
    ]

    assert mounts["camera"]["xyz"][0] - mounts["lidar"]["xyz"][0] == (
        pytest.approx(0.0955)
    )
    assert mounts["camera"]["xyz"][1] == pytest.approx(0.0)
    assert mounts["camera"]["xyz"][2] - mounts["imu"]["xyz"][2] == (
        pytest.approx(0.011)
    )
    assert fastlivo["Rcl"] == pytest.approx(
        flatten(camera_from_lidar), abs=1.0e-8
    )
    assert fastlivo["Pcl"] == pytest.approx(
        matvec(transpose(base_from_camera), lidar_minus_camera), abs=1.0e-8
    )


def test_rtk_mount_and_eskf_lever_arm_use_the_same_calibration():
    mounts = load_config("sensor_mounts.yaml")
    eskf = load_config("kf_gins_n300pro.yaml")["rtk_eskf_localization"][
        "ros__parameters"
    ]
    rtk = load_config("rtk_nmea.yaml")["rtk_nmea"]["ros__parameters"]

    imu_xyz = mounts["imu"]["xyz"]
    rtk_xyz = mounts["rtk"]["xyz"]
    expected_lever_arm = [
        rtk_coordinate - imu_coordinate
        for rtk_coordinate, imu_coordinate in zip(rtk_xyz, imu_xyz)
    ]

    assert rtk_xyz == pytest.approx([0.1425, 0.2952585, 0.78476])
    assert mounts["rtk"]["rpy"] == [0.0, 0.0, 0.0]
    assert rtk_xyz[1] > 0.0
    assert eskf["base_to_imu_m"] == pytest.approx(imu_xyz)
    assert eskf["antlever_m"] == pytest.approx(expected_lever_arm)
    assert eskf["antlever_m"] == pytest.approx([0.0, 0.2952585, 0.64176])
    assert eskf["rtk_heading_timeout_sec"] <= 0.25
    assert 0.0 < eskf["max_imu_gap_sec"] <= 0.5
    assert rtk["heading_offset_deg"] == -90.0
    assert eskf["use_rtk_heading_covariance"] is True
    assert (
        eskf["rtk_heading_covariance_topic"]
        == rtk["heading_covariance_topic"]
    )
    assert rtk["fixed_heading_std_floor_deg"] == pytest.approx(1.0)
    assert rtk["float_heading_std_floor_deg"] == pytest.approx(5.0)

    initializer = load_config("rtk_map_initializer.yaml")[
        "rtk_map_initializer"
    ]["ros__parameters"]
    assert initializer["base_to_master_antenna_m"] == pytest.approx(rtk_xyz)
    assert initializer["allow_unvalidated_georeference_yaw"] is True


def assert_c16_stvl(
    costmap,
    obstacle_range,
    min_obstacle_height=0.08,
    max_obstacle_height=1.8,
):
    assert "obstacle_layer" not in costmap
    assert "stvl_layer" in costmap["plugins"]
    layer = costmap["stvl_layer"]
    assert (
        layer["plugin"]
        == "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
    )
    assert layer["mapping_mode"] is False
    assert layer["observation_sources"] == "lidar_mark lidar_clear"

    marking = layer["lidar_mark"]
    assert marking["topic"] == "/lidar/points"
    assert marking["data_type"] == "PointCloud2"
    assert marking["marking"] is True
    assert marking["clearing"] is False
    assert marking["obstacle_range"] == obstacle_range
    assert marking["min_obstacle_height"] == min_obstacle_height
    assert marking["max_obstacle_height"] == max_obstacle_height

    clearing = layer["lidar_clear"]
    assert clearing["topic"] == "/lidar/points"
    assert clearing["data_type"] == "PointCloud2"
    assert clearing["marking"] is False
    assert clearing["clearing"] is True
    assert clearing["model_type"] == 1
    assert clearing["horizontal_fov_angle"] > 6.28


def test_differential_config_uses_state_lattice_and_mppi_diff_drive():
    config = load_config(
        "nav2_params_differential_fastlivo_mapped.yaml", "differential"
    )
    chassis = load_config("chassis_can.yaml", "differential")["/**"][
        "ros__parameters"
    ]
    calibration = load_config("vehicle_calibration.yaml", "differential")
    geometry = calibration["geometry"]
    drivetrain = calibration["drivetrain"]
    controller = config["controller_server"]["ros__parameters"]
    follow_path = controller["FollowPath"]
    planner = config["planner_server"]["ros__parameters"]["GridBased"]

    assert "amcl" not in config
    assert "/scan" not in yaml.safe_dump(config)
    assert controller["controller_plugins"] == ["FollowPath"]
    assert controller["odom_topic"] == "/fastlivo_rtk/odometry"
    assert follow_path["plugin"] == "nav2_mppi_controller::MPPIController"
    assert follow_path["motion_model"] == "DiffDrive"
    assert follow_path["vx_max"] == chassis["max_linear_velocity"]
    assert follow_path["vx_min"] == -calibration["motion"][
        "commissioning_reverse_velocity_mps"
    ]
    assert follow_path["vy_max"] == 0.0
    assert follow_path["vy_std"] == 0.0
    assert follow_path["wz_max"] == chassis["max_angular_velocity"]
    assert follow_path["enforce_path_inversion"] is True
    assert follow_path["PathAngleCritic"]["forward_preference"] is False
    assert "AckermannConstraints" not in follow_path
    assert "PreferForwardCritic" not in follow_path

    assert planner["plugin"] == "nav2_smac_planner/SmacPlannerLattice"
    assert planner["lattice_filepath"] == ""
    assert planner["reverse_penalty"] > 1.0
    assert planner["rotation_penalty"] > 1.0
    assert "motion_model_for_search" not in planner
    assert "minimum_turning_radius" not in planner

    assert chassis["track_width_m"] == geometry["track_width_m"]
    assert chassis["zqwl_bitrate"] == 250000
    assert chassis["send_rate_hz"] == 5.0
    assert chassis["require_sequential_feedback_counter"] is False
    assert chassis["command_full_scale_level"] == drivetrain[
        "command_full_scale_level"
    ]
    assert chassis["command_calibration_levels"] == drivetrain[
        "command_calibration_levels"
    ]
    assert chassis["command_calibration_wheel_speeds_mps"] == drivetrain[
        "command_calibration_wheel_speeds_mps"
    ]
    assert chassis["feedback_wheel_speed_mps_per_speed_unit"] == drivetrain[
        "feedback_wheel_speed_mps_per_speed_unit"
    ]
    assert chassis["invert_left_motor"] is drivetrain["invert_left_motor"]
    assert chassis["invert_right_motor"] is drivetrain["invert_right_motor"]

    behavior = config["behavior_server"]["ros__parameters"]
    assert "spin" in behavior["behavior_plugins"]
    assert "backup" in behavior["behavior_plugins"]
    bt_plugins = config["bt_navigator"]["ros__parameters"][
        "plugin_lib_names"
    ]
    assert "nav2_back_up_action_bt_node" in bt_plugins
    assert "nav2_back_up_cancel_bt_node" in bt_plugins


def test_differential_config_uses_c16_stvl_and_full_buffered_footprint():
    config = load_config(
        "nav2_params_differential_fastlivo_mapped.yaml", "differential"
    )
    calibration = load_config("vehicle_calibration.yaml", "differential")
    geometry = calibration["geometry"]
    expected_footprint = [
        [geometry["body_front_m"] + geometry["safety_front_m"],
         geometry["body_left_m"] + geometry["safety_left_m"]],
        [geometry["body_front_m"] + geometry["safety_front_m"],
         -(geometry["body_right_m"] + geometry["safety_right_m"])],
        [-(geometry["body_rear_m"] + geometry["safety_rear_m"]),
         -(geometry["body_right_m"] + geometry["safety_right_m"])],
        [-(geometry["body_rear_m"] + geometry["safety_rear_m"]),
         geometry["body_left_m"] + geometry["safety_left_m"]],
    ]

    for name, obstacle_range in (("global_costmap", 8.0), ("local_costmap", 4.0)):
        costmap = config[name][name]["ros__parameters"]
        assert_c16_stvl(costmap, obstacle_range, 0.10, 1.80)
        assert yaml.safe_load(costmap["footprint"]) == expected_footprint
        assert costmap["footprint_padding"] == 0.0
        assert costmap["inflation_layer"]["inflation_radius"] == 2.0
        assert costmap["inflation_layer"]["cost_scaling_factor"] == 1.0


def test_differential_lattice_matches_costmap_and_supports_in_place_rotation():
    config = load_config(
        "nav2_params_differential_fastlivo_mapped.yaml", "differential"
    )
    primitive_path = (
        PACKAGE_ROOT
        / "differential"
        / "config"
        / "motion_primitives"
        / "diff_5cm.json"
    )
    primitive_set = json.loads(primitive_path.read_text())
    metadata = primitive_set["lattice_metadata"]
    primitives = primitive_set["primitives"]

    assert metadata["motion_model"] == "diff"
    assert metadata["grid_resolution"] == config["global_costmap"][
        "global_costmap"
    ]["ros__parameters"]["resolution"]
    assert metadata["number_of_trajectories"] == len(primitives)
    assert metadata["num_of_headings"] >= 16
    assert any(
        primitive["trajectory_length"] == 0.0
        and primitive["poses"][0][:2] == [0.0, 0.0]
        and primitive["poses"][-1][:2] == [0.0, 0.0]
        and primitive["poses"][0][2] != primitive["poses"][-1][2]
        for primitive in primitives
    )


def test_differential_rtk_float_heading_remains_a_coarse_initialization_source():
    initializer = load_config("rtk_map_initializer.yaml", "differential")[
        "rtk_map_initializer"
    ]["ros__parameters"]
    assert initializer["allowed_heading_solutions"] == [
        "L1_FLOAT",
        "L1_INT",
        "NARROW_INT",
    ]
    assert initializer["required_fix_quality"] == 4
    assert initializer["maximum_heading_std_deg"] == pytest.approx(5.0)


def test_differential_localization_extrinsics_share_one_measured_source():
    mounts = load_config("sensor_mounts.yaml", "differential")
    fastlio = load_config("fast_lio_c16.yaml", "differential")["/**"][
        "ros__parameters"
    ]
    fastlivo = load_config("fastlivo_c16_camera.yaml", "differential")["/**"][
        "ros__parameters"
    ]
    fastlio_bridge = load_config("fastlio_bridge.yaml", "differential")[
        "fastlio_odom_bridge"
    ]["ros__parameters"]
    fastlivo_bridge = load_config("fastlivo_bridge.yaml", "differential")[
        "fastlio_odom_bridge"
    ]["ros__parameters"]
    fusion = load_config("fastlivo_rtk_fusion.yaml", "differential")[
        "fastlivo_rtk_fusion"
    ]["ros__parameters"]
    initializer = load_config("rtk_map_initializer.yaml", "differential")[
        "rtk_map_initializer"
    ]["ros__parameters"]
    localizer = load_config("pcd_initial_localization.yaml", "differential")[
        "pcd_initial_localizer"
    ]["ros__parameters"]
    calibration = load_config("vehicle_calibration.yaml", "differential")

    base_from_imu = rpy_matrix(mounts["imu"]["rpy"])
    base_from_lidar = rpy_matrix(mounts["lidar"]["rpy"])
    base_from_camera = rpy_matrix(mounts["camera"]["rpy"])
    imu_from_lidar = matmul(transpose(base_from_imu), base_from_lidar)
    camera_from_lidar = matmul(transpose(base_from_camera), base_from_lidar)
    lidar_minus_imu = [
        lidar - imu
        for lidar, imu in zip(mounts["lidar"]["xyz"], mounts["imu"]["xyz"])
    ]
    lidar_minus_camera = [
        lidar - camera
        for lidar, camera in zip(
            mounts["lidar"]["xyz"], mounts["camera"]["xyz"]
        )
    ]

    assert mounts["calibration"]["complete"] is True
    assert calibration["calibration_complete"] is True
    assert calibration["calibration_source"] == (
        "provisional_manual_measurements_accepted_2026_08_27"
    )
    assert calibration["base_link_reference"] == "tracked_contact_patch_center"
    assert mounts["imu"]["xyz"] == pytest.approx([0.0, 0.0, 0.64])
    assert mounts["imu"]["rpy"] == pytest.approx([0.0, 0.0, 0.0])
    assert mounts["lidar"]["xyz"] == pytest.approx([0.47, 0.0, 0.91])
    assert mounts["lidar"]["rpy"] == pytest.approx(
        [0.0, 0.0, math.pi / 2.0], abs=1.0e-8
    )
    assert mounts["camera"]["xyz"] == pytest.approx([0.4735, 0.0, 0.96])
    assert mounts["rtk"]["xyz"] == pytest.approx([-0.48, 0.35, 0.748])
    assert mounts["rtk"]["secondary_xyz"] == pytest.approx(
        [-0.48, -0.35, 0.748]
    )
    assert mounts["rtk"]["baseline_m"] == pytest.approx(0.70)
    assert fastlivo["time_offset"]["img_time_offset"] == -0.0033
    assert fastlio["mapping"]["extrinsic_R"] == pytest.approx(
        flatten(imu_from_lidar), abs=1.0e-8
    )
    assert fastlio["mapping"]["extrinsic_T"] == pytest.approx(
        matvec(transpose(base_from_imu), lidar_minus_imu), abs=1.0e-8
    )
    assert fastlivo["extrin_calib"]["extrinsic_R"] == pytest.approx(
        flatten(imu_from_lidar), abs=1.0e-8
    )
    assert fastlivo["extrin_calib"]["extrinsic_T"] == pytest.approx(
        matvec(transpose(base_from_imu), lidar_minus_imu), abs=1.0e-8
    )
    assert fastlivo["extrin_calib"]["Rcl"] == pytest.approx(
        flatten(camera_from_lidar), abs=1.0e-8
    )
    assert fastlivo["extrin_calib"]["Pcl"] == pytest.approx(
        matvec(transpose(base_from_camera), lidar_minus_camera), abs=1.0e-8
    )
    for bridge in (fastlio_bridge, fastlivo_bridge):
        assert bridge["base_to_body_xyz"] == pytest.approx(mounts["imu"]["xyz"])
        assert bridge["base_to_body_rpy"] == pytest.approx(mounts["imu"]["rpy"])
    assert fusion["base_to_antenna_xyz"] == pytest.approx(mounts["rtk"]["xyz"])
    assert fusion["base_from_imu_rpy"] == pytest.approx(mounts["imu"]["rpy"])
    assert initializer["base_to_master_antenna_m"] == pytest.approx(
        mounts["rtk"]["xyz"]
    )
    assert localizer["base_to_body_xyz"] == pytest.approx(mounts["lidar"]["xyz"])
    assert localizer["base_to_body_rpy"] == pytest.approx(mounts["lidar"]["rpy"])
    assert localizer["maximum_inlier_rmse"] == pytest.approx(0.30)


def test_ackermann_configs_use_mppi_and_ackermann_motion_model():
    cases = (
        (
            "nav2_params_ackermann_navsat_static.yaml",
            "/odometry/filtered_navsat",
            0.3,
            10.0,
        ),
        (
            "nav2_params_ackermann_fastlio_mapped.yaml",
            "/fastlio/odometry",
            0.3,
            10.0,
        ),
        (
            "nav2_params_ackermann_fastlio_local.yaml",
            "/fastlio/odometry",
            0.3,
            10.0,
        ),
    )
    for name, odom_topic, max_velocity, lookup_table_size in cases:
        config = load_config(name, "ackermann")
        controller = config["controller_server"]["ros__parameters"]
        follow_path = controller["FollowPath"]
        planner = config["planner_server"]["ros__parameters"]["GridBased"]

        assert controller["controller_plugins"] == ["FollowPath"]
        assert controller["odom_topic"] == odom_topic
        assert follow_path["plugin"] == "nav2_mppi_controller::MPPIController"
        assert follow_path["motion_model"] == "Ackermann"
        assert follow_path["vx_max"] == max_velocity
        assert follow_path["vx_min"] == 0.0
        assert follow_path["vy_max"] == 0.0
        assert follow_path["enforce_path_inversion"] is False
        assert follow_path["AckermannConstraints"]["min_turning_r"] == pytest.approx(
            MODEL_MIN_TURNING_RADIUS, abs=1e-6
        )
        assert follow_path["wz_max"] == pytest.approx(
            max_velocity / MODEL_MIN_TURNING_RADIUS, abs=1e-6
        )
        assert planner["plugin"] == "nav2_smac_planner/SmacPlannerHybrid"
        assert planner["motion_model_for_search"] == "DUBIN"
        assert "reverse_penalty" not in planner
        assert planner["minimum_turning_radius"] == pytest.approx(
            MODEL_MIN_TURNING_RADIUS, abs=1e-6
        )
        assert planner["lookup_table_size"] == lookup_table_size

        behavior_server = config["behavior_server"]["ros__parameters"]
        assert "backup" not in behavior_server["behavior_plugins"]
        assert "backup" not in behavior_server
        assert all(
            "back_up" not in plugin
            for plugin in config["bt_navigator"]["ros__parameters"][
                "plugin_lib_names"
            ]
        )


def test_ackermann_configs_use_c16_stvl_for_obstacles():
    for name in (
        "nav2_params_ackermann_navsat_static.yaml",
        "nav2_params_ackermann_fastlio_mapped.yaml",
        "nav2_params_ackermann_fastlio_local.yaml",
    ):
        config = load_config(name, "ackermann")
        global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
        local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]

        assert_c16_stvl(global_costmap, 8.0, 0.233, 0.8725)
        assert_c16_stvl(local_costmap, 4.0, 0.233, 0.8725)


def test_ackermann_navsat_navigation_matches_fastlio_mapped_except_odom():
    navsat = load_config("nav2_params_ackermann_navsat_static.yaml", "ackermann")
    fastlio = load_config("nav2_params_ackermann_fastlio_mapped.yaml", "ackermann")

    for node_name in ("bt_navigator", "controller_server"):
        navsat[node_name]["ros__parameters"]["odom_topic"] = fastlio[node_name][
            "ros__parameters"
        ]["odom_topic"]

    assert navsat == fastlio


def test_c16_driver_does_not_publish_legacy_scan():
    config = load_config("c16.yaml")
    lidar = config["lslidar_driver_node"]["ros__parameters"]

    assert lidar["pointcloud_topic"] == "/lidar/points"
    assert lidar["publish_scan"] is False
    assert "scan_num" not in lidar


def test_3d_mapping_and_mapped_navigation_use_optional_fpfh_localization():
    mapping = load_config("pcd_mapping.yaml")["pcd_map_builder"]["ros__parameters"]
    localizer = load_config("pcd_initial_localization.yaml")[
        "pcd_initial_localizer"
    ]["ros__parameters"]

    assert mapping["cloud_topic"] == "/cloud_registered"
    assert mapping["cloud_frame"] == "camera_init"
    assert mapping["odom_topic"] == "/fastlio/odometry"
    assert mapping["map_frame"] == "map"
    assert mapping["odom_frame"] == "odom"
    assert mapping["voxel_size"] == 0.10
    assert mapping["min_observations"] >= 2
    assert mapping["rear_exclusion_enabled"] is True
    assert mapping["rear_exclusion_min_x"] == -4.0
    assert mapping["rear_exclusion_max_x"] == -0.1275
    assert mapping["rear_exclusion_half_width"] == 0.60
    assert mapping["occupancy_resolution"] == 0.05
    assert mapping["occupancy_min_z"] == 0.233
    assert mapping["occupancy_max_z"] == 0.8725

    assert localizer["cloud_topic"] == "/cloud_registered_body"
    assert localizer["cloud_frame"] == "body"
    assert localizer["odom_topic"] == "/fastlio/odometry"
    assert localizer["initial_pose_topic"] == "/initialpose"
    assert localizer["ready_topic"] == "/localization/ready"
    assert localizer["external_ready_topic"] == ""
    assert localizer["external_ready_timeout_sec"] == pytest.approx(0.5)
    assert localizer["map_frame"] == "map"
    assert localizer["odom_frame"] == "odom"
    assert localizer["enable_fpfh"] is False
    assert localizer["automatic_global_localization"] is False
    assert localizer["base_to_body_xyz"] == [0.1425, 0.0, 0.143]
    assert localizer["initial_scan_count"] == 5
    assert localizer["initial_search_radius"] == 8.0
    assert localizer["local_submap_radius"] == 8.0
    assert localizer["feature_voxel_size"] == 0.35
    assert localizer["normal_radius"] > localizer["feature_voxel_size"]
    assert localizer["feature_radius"] > localizer["normal_radius"]
    assert localizer["fpfh_max_iterations"] == 12000
    assert localizer["coarse_ndt_resolution"] > localizer["fine_ndt_resolution"]
    assert localizer["gicp_max_correspondence_distance"] == 0.50
    assert localizer["maximum_inlier_rmse"] == 0.20
    assert "runtime_matching_rate_hz" not in localizer
    assert "runtime_correction_alpha" not in localizer
    assert "runtime_failure_limit" not in localizer
    assert "maximum_translation_refinement" not in localizer
    assert "maximum_yaw_refinement" not in localizer
    assert "minimum_overlap" not in localizer
    assert "maximum_tilt" not in localizer
    assert "maximum_base_height" not in localizer

    localizer_source = (
        PACKAGE_ROOT
        / "localization"
        / "pcd"
        / "src"
        / "pcd_initial_localizer.cpp"
    ).read_text()
    assert "pcl::FPFHEstimation" in localizer_source
    assert "pcl::SampleConsensusPrerejective" in localizer_source
    assert 'declare_parameter<bool>("enable_fpfh", false)' in localizer_source
    assert 'declare_parameter<bool>("automatic_global_localization", false)' in (
        localizer_source
    )
    assert "automatic_global_localization requires enable_fpfh" in localizer_source
    assert "if (enable_fpfh_)" in localizer_source
    assert "pcl::NormalDistributionsTransform" in localizer_source
    assert "pcl::GeneralizedIterativeClosestPoint" in localizer_source
    assert "cloud_subscription_.reset()" not in localizer_source
    assert "initial_pose_subscription_.reset()" not in localizer_source
    assert "matching_timer_->cancel()" not in localizer_source
    assert "interpolateTransform" not in localizer_source
    assert "Runtime registration" not in localizer_source
    assert "runtime map correction" not in localizer_source
    assert "if (matching_ || localized_)" in localizer_source
    assert "initial localization accepted; map-to-odom correction fixed" in localizer_source
    assert "result.inlier_rmse > maximum_inlier_rmse_" in localizer_source
    assert "result moved too far from the RViz position prior" not in localizer_source
    assert "result disagrees with the RViz heading prior" not in localizer_source
    assert "scan-to-map overlap is below the acceptance limit" not in localizer_source
    assert "result has implausible roll or pitch" not in localizer_source
    assert "result has implausible rear-axle height" not in localizer_source


def test_ackermann_configs_use_buffered_rear_axle_footprint():
    for name in (
        "nav2_params_ackermann_navsat_static.yaml",
        "nav2_params_ackermann_fastlio_mapped.yaml",
        "nav2_params_ackermann_fastlio_local.yaml",
    ):
        config = load_config(name, "ackermann")
        for costmap_name in ("global_costmap", "local_costmap"):
            costmap = config[costmap_name][costmap_name]["ros__parameters"]
            assert yaml.safe_load(costmap["footprint"]) == SAFETY_FOOTPRINT
            assert costmap["footprint_padding"] == 0.0


def test_ackermann_fastlio_local_config_uses_rolling_obstacle_costmaps():
    config = load_config("nav2_params_ackermann_fastlio_local.yaml", "ackermann")

    for costmap_name in ("global_costmap", "local_costmap"):
        costmap = config[costmap_name][costmap_name]["ros__parameters"]
        assert costmap["global_frame"] == "odom"
        assert costmap["rolling_window"] is True
        assert "static_layer" not in costmap["plugins"]
        assert costmap["plugins"] == ["stvl_layer", "inflation_layer"]
        assert_c16_stvl(
            costmap,
            8.0 if costmap_name == "global_costmap" else 4.0,
            0.233,
            0.8725,
        )

    global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
    assert global_costmap["width"] == 20
    assert global_costmap["height"] == 20
    assert global_costmap["track_unknown_space"] is False
    assert config["bt_navigator"]["ros__parameters"]["global_frame"] == "odom"
    assert config["behavior_server"]["ros__parameters"]["global_frame"] == "odom"


def test_ackermann_fastlio_local_uses_kinematically_feasible_global_planner():
    config = load_config("nav2_params_ackermann_fastlio_local.yaml", "ackermann")
    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    controller = config["controller_server"]["ros__parameters"]["FollowPath"]

    assert planner["plugin"] == "nav2_smac_planner/SmacPlannerHybrid"
    assert planner["motion_model_for_search"] == "DUBIN"
    assert planner["minimum_turning_radius"] == pytest.approx(
        MODEL_MIN_TURNING_RADIUS, abs=1e-6
    )
    assert planner["minimum_turning_radius"] == controller["AckermannConstraints"][
        "min_turning_r"
    ]
    assert planner["angle_quantization_bins"] == 72
    # Humble's downsampler keeps its initial origin when a rolling costmap moves.
    assert planner["downsample_costmap"] is False
    assert planner["downsampling_factor"] == 1
    assert planner["analytic_expansion_max_length"] >= (
        4.0 * planner["minimum_turning_radius"]
    )
    assert controller["vx_min"] == 0.0
    assert controller["enforce_path_inversion"] is False
    assert controller["PathAlignCritic"]["use_path_orientations"] is True
    assert controller["PathAlignCritic"]["offset_from_furthest"] == 6
    assert controller["PathFollowCritic"]["offset_from_furthest"] == 6
    assert controller["PathAngleCritic"]["forward_preference"] is True
    assert controller["PathAngleCritic"]["cost_weight"] == 4.0
    assert controller["PathAngleCritic"]["offset_from_furthest"] == 8
    assert controller["PreferForwardCritic"]["enabled"] is True


def test_ackermann_fastlio_mapped_uses_real_vehicle_limits_and_static_map():
    config = load_config("nav2_params_ackermann_fastlio_mapped.yaml", "ackermann")
    controller = config["controller_server"]["ros__parameters"]["FollowPath"]
    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
    local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]

    assert config["bt_navigator"]["ros__parameters"]["global_frame"] == "map"
    assert controller["vx_max"] == 0.30
    assert controller["vx_min"] == 0.0
    assert controller["batch_size"] == 1200
    assert controller["PathAlignCritic"]["offset_from_furthest"] == 6
    assert controller["PathFollowCritic"]["offset_from_furthest"] == 6
    assert controller["PathAngleCritic"]["cost_weight"] == 4.0
    assert controller["PathAngleCritic"]["offset_from_furthest"] == 8
    assert controller["AckermannConstraints"]["min_turning_r"] == pytest.approx(
        MODEL_MIN_TURNING_RADIUS, abs=1e-6
    )
    assert planner["minimum_turning_radius"] == pytest.approx(
        MODEL_MIN_TURNING_RADIUS, abs=1e-6
    )
    assert planner["lookup_table_size"] == 10.0
    assert planner["downsample_costmap"] is True
    assert planner["downsampling_factor"] == 2
    for costmap in (global_costmap, local_costmap):
        assert costmap["global_frame"] == "map"
        assert costmap["plugins"] == [
            "static_layer",
            "stvl_layer",
            "inflation_layer",
            "semantic_proximity_layer",
        ]
        assert costmap["static_layer"]["subscribe_to_updates"] is True
        assert costmap["inflation_layer"]["inflation_radius"] == 2.0
        assert costmap["inflation_layer"]["cost_scaling_factor"] == 1.0
        semantic_layer = costmap["semantic_proximity_layer"]
        for key, value in {
            "plugin": "agribot_hardware_bringup::SemanticProximityLayer",
            "enabled": True,
            "source_topic": "/semantic_navigation/proximity_costmap",
            "maximum_cost": 240,
        }.items():
            assert semantic_layer[key] == value
    assert global_costmap["semantic_proximity_layer"]["obstacle_costmap_topic"] == (
        "obstacle_costmap"
    )
    assert global_costmap["semantic_proximity_layer"][
        "obstacle_costmap_publish_frequency"
    ] == 1.0
    assert "obstacle_costmap_topic" not in local_costmap["semantic_proximity_layer"]


def test_ackermann_fastlio_local_config_limits_steering_corrections():
    config = load_config("nav2_params_ackermann_fastlio_local.yaml", "ackermann")
    follow_path = config["controller_server"]["ros__parameters"]["FollowPath"]

    assert follow_path["wz_std"] <= 0.10
    assert follow_path["wz_max"] <= 0.30
    assert follow_path["az_max"] <= 0.35
    assert follow_path["PathAlignCritic"]["cost_weight"] <= 3.5


def test_ackermann_fastlio_local_balances_long_plans_and_rdk_load():
    config = load_config("nav2_params_ackermann_fastlio_local.yaml", "ackermann")
    controller = config["controller_server"]["ros__parameters"]["FollowPath"]
    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
    local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]

    assert planner["lookup_table_size"] == 10.0
    assert global_costmap["resolution"] == 0.10
    assert local_costmap["resolution"] == 0.05
    assert controller["batch_size"] == 1200
    assert global_costmap["update_frequency"] == 2.0
    assert local_costmap["update_frequency"] == 5.0
    assert global_costmap["stvl_layer"]["voxel_size"] == 0.10
    assert local_costmap["stvl_layer"]["voxel_size"] == 0.10
    assert global_costmap["stvl_layer"]["voxel_decay"] == 5.0


def test_ackermann_serial_config_limits_steering_rate():
    config = load_config("chassis_serial.yaml", "ackermann")["/**"]["ros__parameters"]

    assert config["send_rate_hz"] == 20.0
    assert config["max_steering_rate_rad_s"] == 0.60
    assert config["require_localization_ready"] is False
    assert config["localization_ready_topic"] == "/localization/ready"
    assert config["localization_ready_timeout_sec"] == 2.5


def test_ackermann_chassis_and_visualization_match_urdf_kinematics():
    expected_max_angular = 0.80 / MODEL_MIN_TURNING_RADIUS
    for name in ("chassis_can.yaml", "chassis_serial.yaml"):
        config = load_config(name, "ackermann")["/**"]["ros__parameters"]
        assert config["wheelbase_m"] == pytest.approx(MODEL_WHEELBASE, abs=1e-7)
        assert config["max_steering_angle_rad"] == MODEL_MAX_STEERING
        assert config["max_angular_velocity"] == pytest.approx(
            expected_max_angular, abs=1e-6
        )

    visualization = load_config("joint_state_publisher.yaml", "ackermann")["/**"][
        "ros__parameters"
    ]
    assert visualization["wheelbase_m"] == pytest.approx(
        MODEL_WHEELBASE, abs=1e-7
    )
    assert visualization["max_steering_angle_rad"] == MODEL_MAX_STEERING
    assert visualization["wheel_radius_m"] == 0.1275


def test_ackermann_can_supports_localization_readiness_gate():
    config = load_config("chassis_can.yaml", "ackermann")["/**"]["ros__parameters"]

    assert config["require_localization_ready"] is False
    assert config["localization_ready_topic"] == "/localization/ready"
    assert config["localization_ready_timeout_sec"] == 2.5
    assert config["recover_zqwl_startup"] is True
    assert config["startup_feedback_timeout_sec"] == 1.0


def test_ackermann_fastlio_local_uses_requested_inflation_radius():
    config = load_config("nav2_params_ackermann_fastlio_local.yaml", "ackermann")

    for costmap_name in ("global_costmap", "local_costmap"):
        costmap = config[costmap_name][costmap_name]["ros__parameters"]
        assert costmap["inflation_layer"]["inflation_radius"] == 2.0
        assert costmap["inflation_layer"]["cost_scaling_factor"] == 1.0
