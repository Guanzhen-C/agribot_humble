from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


def load_config(name, vehicle=None):
    directory = PACKAGE_ROOT / vehicle if vehicle else PACKAGE_ROOT
    with (directory / "config" / name).open() as stream:
        return yaml.safe_load(stream)


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


def test_differential_configs_use_horizontal_scan_for_obstacles():
    for name in ("nav2_dwb_navsat.yaml", "nav2_dwb_fastlio.yaml"):
        config = load_config(name, "differential")
        local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]
        obstacle_layer = local_costmap["obstacle_layer"]

        assert obstacle_layer["observation_sources"] == "scan"
        assert obstacle_layer["scan"]["topic"] == "/scan"
        assert obstacle_layer["scan"]["data_type"] == "LaserScan"


def test_ackermann_configs_use_mppi_and_ackermann_motion_model():
    cases = (
        (
            "nav2_params_ackermann_navsat_static.yaml",
            "/odometry/filtered_navsat",
            0.8,
        ),
        ("nav2_params_ackermann_fastlio_static.yaml", "/fastlio/odometry", 0.8),
        ("nav2_params_ackermann_fastlio_local.yaml", "/fastlio/odometry", 0.3),
    )
    for name, odom_topic, max_velocity in cases:
        config = load_config(name, "ackermann")
        controller = config["controller_server"]["ros__parameters"]
        follow_path = controller["FollowPath"]

        assert controller["controller_plugins"] == ["FollowPath"]
        assert controller["odom_topic"] == odom_topic
        assert follow_path["plugin"] == "nav2_mppi_controller::MPPIController"
        assert follow_path["motion_model"] == "Ackermann"
        assert follow_path["vx_max"] == max_velocity
        assert follow_path["vy_max"] == 0.0
        assert follow_path["AckermannConstraints"]["min_turning_r"] == 1.30


def test_ackermann_configs_use_horizontal_scan_for_obstacles():
    for name in (
        "nav2_params_ackermann_navsat_static.yaml",
        "nav2_params_ackermann_fastlio_static.yaml",
        "nav2_params_ackermann_fastlio_local.yaml",
    ):
        config = load_config(name, "ackermann")
        local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]
        obstacle_layer = local_costmap["obstacle_layer"]

        assert obstacle_layer["observation_sources"] == "scan"
        assert obstacle_layer["scan"]["topic"] == "/scan"
        assert obstacle_layer["scan"]["data_type"] == "LaserScan"


def test_ackermann_configs_use_measured_rear_axle_footprint():
    expected = [
        [0.66, 0.33],
        [0.66, -0.33],
        [-0.12, -0.33],
        [-0.12, 0.33],
    ]
    for name in (
        "nav2_params_ackermann_navsat_static.yaml",
        "nav2_params_ackermann_fastlio_static.yaml",
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
        assert costmap["plugins"] == ["obstacle_layer", "inflation_layer"]
        assert costmap["obstacle_layer"]["scan"]["topic"] == "/scan"

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


def test_ackermann_fastlio_local_config_limits_steering_corrections():
    config = load_config("nav2_params_ackermann_fastlio_local.yaml", "ackermann")
    follow_path = config["controller_server"]["ros__parameters"]["FollowPath"]

    assert follow_path["wz_std"] <= 0.10
    assert follow_path["wz_max"] <= 0.30
    assert follow_path["az_max"] <= 0.35
    assert follow_path["PathAlignCritic"]["cost_weight"] <= 3.5


def test_ackermann_serial_config_limits_steering_rate():
    config = load_config("chassis_serial.yaml", "ackermann")["/**"]["ros__parameters"]

    assert config["send_rate_hz"] == 20.0
    assert config["max_steering_rate_rad_s"] == 0.60


def test_ackermann_fastlio_local_uses_requested_inflation_radius():
    config = load_config("nav2_params_ackermann_fastlio_local.yaml", "ackermann")

    for costmap_name in ("global_costmap", "local_costmap"):
        costmap = config[costmap_name][costmap_name]["ros__parameters"]
        assert costmap["inflation_layer"]["inflation_radius"] == 0.4
