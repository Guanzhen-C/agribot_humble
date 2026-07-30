from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


def load_config(name, vehicle=None):
    directory = PACKAGE_ROOT / vehicle if vehicle else PACKAGE_ROOT
    with (directory / "config" / name).open() as stream:
        return yaml.safe_load(stream)


def assert_c16_stvl(costmap, obstacle_range):
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
    assert marking["min_obstacle_height"] == 0.08
    assert marking["max_obstacle_height"] == 1.8

    clearing = layer["lidar_clear"]
    assert clearing["topic"] == "/lidar/points"
    assert clearing["data_type"] == "PointCloud2"
    assert clearing["marking"] is False
    assert clearing["clearing"] is True
    assert clearing["model_type"] == 1
    assert clearing["horizontal_fov_angle"] > 6.28


def test_differential_configs_use_dwb_and_matching_limits():
    for name in ("nav2_dwb_navsat.yaml", "nav2_dwb_fastlio.yaml"):
        config = load_config(name, "differential")
        controller = config["controller_server"]["ros__parameters"]
        follow_path = controller["FollowPath"]

        assert controller["controller_plugins"] == ["FollowPath"]
        assert follow_path["plugin"] == "dwb_core::DWBLocalPlanner"
        assert follow_path["max_vel_x"] == 0.8
        assert follow_path["max_vel_theta"] == 1.4
        assert "RotateToGoal" in follow_path["critics"]


def test_differential_configs_use_c16_stvl_for_obstacles():
    for name in ("nav2_dwb_navsat.yaml", "nav2_dwb_fastlio.yaml"):
        config = load_config(name, "differential")
        global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
        local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]

        assert_c16_stvl(global_costmap, 8.0)
        assert_c16_stvl(local_costmap, 4.0)


def test_ackermann_configs_use_mppi_and_ackermann_motion_model():
    cases = (
        (
            "nav2_params_ackermann_navsat_static.yaml",
            "/odometry/filtered_navsat",
            0.8,
            5.0,
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
        assert follow_path["vy_max"] == 0.0
        assert follow_path["AckermannConstraints"]["min_turning_r"] == 1.30
        assert planner["plugin"] == "nav2_smac_planner/SmacPlannerHybrid"
        assert planner["motion_model_for_search"] == "REEDS_SHEPP"
        assert planner["minimum_turning_radius"] == 1.30
        assert planner["lookup_table_size"] == lookup_table_size


def test_ackermann_configs_use_c16_stvl_for_obstacles():
    for name in (
        "nav2_params_ackermann_navsat_static.yaml",
        "nav2_params_ackermann_fastlio_mapped.yaml",
        "nav2_params_ackermann_fastlio_local.yaml",
    ):
        config = load_config(name, "ackermann")
        global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
        local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]

        assert_c16_stvl(global_costmap, 8.0)
        assert_c16_stvl(local_costmap, 4.0)


def test_c16_driver_does_not_publish_legacy_scan():
    config = load_config("c16.yaml")
    lidar = config["lslidar_driver_node"]["ros__parameters"]

    assert lidar["pointcloud_topic"] == "/lidar/points"
    assert lidar["publish_scan"] is False
    assert "scan_num" not in lidar


def test_mapping_projects_a_height_band_for_slam_toolbox_only():
    projection = load_config("pointcloud_to_laserscan_mapping.yaml")[
        "pointcloud_to_laserscan"
    ]["ros__parameters"]
    slam = load_config("slam_toolbox_mapping_c16.yaml")["slam_toolbox"][
        "ros__parameters"
    ]
    localization = load_config("slam_toolbox_localization_c16.yaml")[
        "slam_toolbox"
    ]["ros__parameters"]

    assert projection["target_frame"] == "base_link"
    assert projection["min_height"] == 0.20
    assert projection["max_height"] == 1.20
    assert projection["range_min"] == 0.30
    assert projection["range_max"] == 20.0
    assert slam["scan_topic"] == "/scan_mapping"
    assert slam["mode"] == "mapping"
    assert slam["odom_frame"] == "odom"
    assert slam["map_frame"] == "map"
    assert slam["base_frame"] == "base_link"
    assert slam["scan_queue_size"] == 10
    assert slam["max_laser_range"] == 20.0
    assert slam["do_loop_closing"] is True
    assert slam["loop_match_maximum_variance_coarse"] == 2.0
    assert slam["loop_match_minimum_response_coarse"] == 0.45
    assert slam["loop_match_minimum_response_fine"] == 0.55
    assert localization["scan_topic"] == "/scan_mapping"
    assert localization["mode"] == "localization"
    assert localization["map_file_name"] == ""
    assert localization["map_start_pose"] == [0.0, 0.0, 0.0]
    assert localization["scan_queue_size"] == 10
    assert localization["max_laser_range"] == 20.0
    assert localization["loop_match_maximum_variance_coarse"] == 2.0
    assert localization["loop_match_minimum_response_coarse"] == 0.45
    assert localization["loop_match_minimum_response_fine"] == 0.55


def test_ackermann_configs_use_measured_rear_axle_footprint():
    expected = [
        [0.66, 0.33],
        [0.66, -0.33],
        [-0.12, -0.33],
        [-0.12, 0.33],
    ]
    for name in (
        "nav2_params_ackermann_navsat_static.yaml",
        "nav2_params_ackermann_fastlio_mapped.yaml",
        "nav2_params_ackermann_fastlio_local.yaml",
    ):
        config = load_config(name, "ackermann")
        for costmap_name in ("global_costmap", "local_costmap"):
            costmap = config[costmap_name][costmap_name]["ros__parameters"]
            assert yaml.safe_load(costmap["footprint"]) == expected


def test_ackermann_fastlio_local_config_uses_rolling_obstacle_costmaps():
    config = load_config("nav2_params_ackermann_fastlio_local.yaml", "ackermann")

    for costmap_name in ("global_costmap", "local_costmap"):
        costmap = config[costmap_name][costmap_name]["ros__parameters"]
        assert costmap["global_frame"] == "odom"
        assert costmap["rolling_window"] is True
        assert "static_layer" not in costmap["plugins"]
        assert costmap["plugins"] == ["stvl_layer", "inflation_layer"]
        assert_c16_stvl(
            costmap, 8.0 if costmap_name == "global_costmap" else 4.0
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
    assert planner["motion_model_for_search"] == "REEDS_SHEPP"
    assert planner["minimum_turning_radius"] == 1.30
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
    assert controller["vx_min"] < 0.0
    assert controller["enforce_path_inversion"] is True
    assert controller["PathAlignCritic"]["use_path_orientations"] is True
    assert controller["PathAngleCritic"]["forward_preference"] is False
    assert controller["PreferForwardCritic"]["enabled"] is False


def test_ackermann_fastlio_mapped_uses_real_vehicle_limits_and_static_map():
    config = load_config("nav2_params_ackermann_fastlio_mapped.yaml", "ackermann")
    controller = config["controller_server"]["ros__parameters"]["FollowPath"]
    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
    local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]

    assert config["bt_navigator"]["ros__parameters"]["global_frame"] == "map"
    assert controller["vx_max"] == 0.30
    assert controller["vx_min"] < 0.0
    assert controller["batch_size"] == 1200
    assert controller["AckermannConstraints"]["min_turning_r"] == 1.30
    assert planner["minimum_turning_radius"] == 1.30
    assert planner["lookup_table_size"] == 10.0
    assert planner["downsample_costmap"] is True
    assert planner["downsampling_factor"] == 2
    for costmap in (global_costmap, local_costmap):
        assert costmap["global_frame"] == "map"
        assert costmap["plugins"] == [
            "static_layer",
            "stvl_layer",
            "inflation_layer",
        ]
        assert costmap["static_layer"]["subscribe_to_updates"] is True
        assert costmap["inflation_layer"]["inflation_radius"] == 0.2


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


def test_ackermann_fastlio_local_uses_requested_inflation_radius():
    config = load_config("nav2_params_ackermann_fastlio_local.yaml", "ackermann")

    for costmap_name in ("global_costmap", "local_costmap"):
        costmap = config[costmap_name][costmap_name]["ros__parameters"]
        assert costmap["inflation_layer"]["inflation_radius"] == 0.2
