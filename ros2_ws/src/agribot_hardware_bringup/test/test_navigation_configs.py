from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


def load_config(name):
    with (PACKAGE_ROOT / "config" / name).open() as stream:
        return yaml.safe_load(stream)


def test_differential_configs_use_dwb_and_matching_limits():
    for name in ("nav2_dwb_navsat.yaml", "nav2_dwb_fastlio.yaml"):
        config = load_config(name)
        controller = config["controller_server"]["ros__parameters"]
        follow_path = controller["FollowPath"]

        assert controller["controller_plugins"] == ["FollowPath"]
        assert follow_path["plugin"] == "dwb_core::DWBLocalPlanner"
        assert follow_path["max_vel_x"] == 0.8
        assert follow_path["max_vel_theta"] == 1.4
        assert "RotateToGoal" in follow_path["critics"]


def test_differential_configs_use_horizontal_scan_for_obstacles():
    for name in ("nav2_dwb_navsat.yaml", "nav2_dwb_fastlio.yaml"):
        config = load_config(name)
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
