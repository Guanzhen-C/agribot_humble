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
        ("nav2_params_ackermann_navsat_static.yaml", "/odometry/filtered_navsat"),
        ("nav2_params_ackermann_fastlio_static.yaml", "/fastlio/odometry"),
    )
    for name, odom_topic in cases:
        config = load_config(name, "ackermann")
        controller = config["controller_server"]["ros__parameters"]
        follow_path = controller["FollowPath"]

        assert controller["controller_plugins"] == ["FollowPath"]
        assert controller["odom_topic"] == odom_topic
        assert follow_path["plugin"] == "nav2_mppi_controller::MPPIController"
        assert follow_path["motion_model"] == "Ackermann"
        assert follow_path["vx_max"] == 0.8
        assert follow_path["vy_max"] == 0.0
        assert follow_path["AckermannConstraints"]["min_turning_r"] == 0.75


def test_ackermann_configs_use_horizontal_scan_for_obstacles():
    for name in (
        "nav2_params_ackermann_navsat_static.yaml",
        "nav2_params_ackermann_fastlio_static.yaml",
    ):
        config = load_config(name, "ackermann")
        local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]
        obstacle_layer = local_costmap["obstacle_layer"]

        assert obstacle_layer["observation_sources"] == "scan"
        assert obstacle_layer["scan"]["topic"] == "/scan"
        assert obstacle_layer["scan"]["data_type"] == "LaserScan"


def test_collision_monitor_parameters_match_launched_node_name():
    config = load_config("collision_monitor.yaml")
    assert "vehicle_collision_monitor" in config
    sources = config["vehicle_collision_monitor"]["ros__parameters"]
    assert sources["observation_sources"] == ["scan"]
    assert sources["scan"]["topic"] == "/scan"
