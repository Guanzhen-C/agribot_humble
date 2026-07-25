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


def test_collision_monitor_parameters_match_launched_node_name():
    config = load_config("collision_monitor.yaml")
    assert "vehicle_collision_monitor" in config
    sources = config["vehicle_collision_monitor"]["ros__parameters"]
    assert sources["observation_sources"] == ["scan"]
    assert sources["scan"]["topic"] == "/scan"
