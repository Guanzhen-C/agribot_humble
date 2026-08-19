import importlib.util
import json
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
ACKERMANN_CONFIG = PACKAGE_ROOT / "ackermann" / "config"
BRIDGE_SCRIPT = PACKAGE_ROOT / "scripts" / "planner_validation_bridge.py"


def load_yaml(path):
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_planner_validation_matches_physical_ackermann_planner():
    planner_only = load_yaml(
        ACKERMANN_CONFIG / "nav2_params_ackermann_planner_only.yaml"
    )
    mapped = load_yaml(
        ACKERMANN_CONFIG / "nav2_params_ackermann_fastlio_mapped.yaml"
    )
    planner = planner_only["planner_server"]["ros__parameters"]["GridBased"]
    mapped_planner = mapped["planner_server"]["ros__parameters"]["GridBased"]

    for parameter in (
        "plugin",
        "motion_model_for_search",
        "minimum_turning_radius",
        "angle_quantization_bins",
        "analytic_expansion_max_length",
        "lookup_table_size",
        "downsample_costmap",
        "downsampling_factor",
    ):
        assert planner[parameter] == mapped_planner[parameter]


def test_planner_validation_uses_only_static_2d_map_layers():
    planner_only = load_yaml(
        ACKERMANN_CONFIG / "nav2_params_ackermann_planner_only.yaml"
    )
    mapped = load_yaml(
        ACKERMANN_CONFIG / "nav2_params_ackermann_fastlio_mapped.yaml"
    )
    costmap = planner_only["global_costmap"]["global_costmap"]["ros__parameters"]
    mapped_costmap = mapped["global_costmap"]["global_costmap"]["ros__parameters"]

    assert costmap["plugins"] == ["static_layer", "inflation_layer"]
    assert "stvl_layer" not in costmap
    assert costmap["footprint"] == mapped_costmap["footprint"]
    assert costmap["footprint_padding"] == 0.0
    assert costmap["footprint_padding"] == mapped_costmap["footprint_padding"]
    assert costmap["inflation_layer"]["plugin"] == (
        mapped_costmap["inflation_layer"]["plugin"]
    )
    assert costmap["inflation_layer"]["inflation_radius"] == 2.0
    assert costmap["inflation_layer"]["cost_scaling_factor"] == 1.0


def test_planner_validation_launch_has_no_motion_or_sensor_nodes():
    source = (
        PACKAGE_ROOT
        / "ackermann"
        / "launch"
        / "ackermann_smac_planner_validation.launch.py"
    ).read_text(encoding="utf-8")

    assert 'package="nav2_map_server"' in source
    assert 'package="nav2_planner"' in source
    assert 'package="pcl_ros"' in source
    assert 'executable="pcd_to_pointcloud"' in source
    assert '"/planning_test/map_3d"' in source
    assert 'DeclareLaunchArgument("show_3d_map", default_value="false")' in source
    assert 'executable="planner_validation_bridge.py"' in source
    assert '"route_plan"' in source
    assert '"route_plan": LaunchConfiguration("route_plan")' in source
    assert '"route_waypoint_mode"' in source
    assert '"path_output_file"' in source
    assert '"node_names": ["map_server", "planner_server"]' in source
    for forbidden in (
        "nav2_controller",
        "bt_navigator",
        "fast_lio",
        "lslidar_driver",
        "hipnuc_imu",
        "ackermann_chassis",
    ):
        assert forbidden not in source


def test_planner_validation_bridge_uses_multiple_waypoints_and_explicit_start():
    source = (
        PACKAGE_ROOT / "scripts" / "planner_validation_bridge.py"
    ).read_text(encoding="utf-8")

    assert "ComputePathThroughPoses" in source
    assert "request.goals" in source
    assert "request.use_start = True" in source
    assert '"/initialpose"' in source
    assert '"/goal_pose"' in source
    assert '"/planning_test/path"' in source
    assert '"/planning_test/waypoints"' in source
    assert "self.waypoints.clear()" in source
    assert "TransformBroadcaster" in source
    assert "load_semantic_route_plan" in source
    assert 'route_poses = semantic_route["astar_poses"]' in source
    assert "for stop in route_poses[1:]" in source
    assert "write_path_output" in source
    assert "path_avoidance_intersections" in source
    assert "State.PRIMARY_STATE_ACTIVE" in source
    assert '"/planner_server/get_state"' in source


def test_planner_validation_bridge_loads_only_ordered_preview_route(tmp_path):
    spec = importlib.util.spec_from_file_location(
        "planner_validation_bridge", BRIDGE_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    route = {
        "schema_version": 3,
        "route_id": "route_test",
        "frame_id": "map",
        "execution_policy": {
            "preview_only": True,
            "execution_authorized": False,
            "requires_nav2_path_planning": True,
            "requires_nav2_keepout_enforcement": True,
        },
        "request": {"start": "place_000", "via": [], "goal": "place_001"},
        "resolved_stops": [
            {
                "selector": "place_000",
                "navigation_route_index": 0,
                "navigation_anchor_place": "place_000",
                "navigation_anchor_position": {"x": 0.0, "y": 0.0},
            },
            {
                "selector": "place_001",
                "navigation_route_index": 2,
                "navigation_anchor_place": "place_001",
                "navigation_anchor_position": {"x": 2.0, "y": 1.0},
            },
        ],
        "route": {
            "poses": [
                {
                    "place_id": "place_000",
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "yaw": 0.0,
                },
                {
                    "place_id": "place_middle",
                    "position": {"x": 1.0, "y": 0.5, "z": 0.0},
                    "yaw": 0.25,
                },
                {
                    "place_id": "place_001",
                    "position": {"x": 2.0, "y": 1.0, "z": 0.0},
                    "yaw": 0.5,
                },
            ]
        },
        "avoidance_constraints": {
            "radius_m": 1.5,
            "nodes": [
                {
                    "selector": "place_009",
                    "position": {"x": 4.0, "y": 5.0, "z": 0.0},
                }
            ],
        },
    }
    route_path = tmp_path / "route.json"
    route_path.write_text(json.dumps(route), encoding="utf-8")

    loaded = module.load_semantic_route_plan(str(route_path), "map")

    assert loaded["route_id"] == "route_test"
    assert [item["selector"] for item in loaded["astar_poses"]] == [
        "place_000",
        "place_middle",
        "place_001",
    ]
    assert [item["selector"] for item in loaded["requested_stops"]] == [
        "place_000",
        "place_001",
    ]
    assert loaded["avoidance_zones"][0]["radius_m"] == 1.5

    route["request"]["goal"] = "place_999"
    route_path.write_text(json.dumps(route), encoding="utf-8")
    with pytest.raises(RuntimeError, match="do not match request order"):
        module.load_semantic_route_plan(str(route_path), "map")


def test_planner_validation_rviz_uses_pose_and_goal_topics():
    rviz = load_yaml(PACKAGE_ROOT / "rviz" / "planner_validation.rviz")
    tools = rviz["Visualization Manager"]["Tools"]
    classes = [tool["Class"] for tool in tools]

    assert "rviz_default_plugins/SetInitialPose" in classes
    assert "rviz_default_plugins/SetGoal" in classes
    document = (PACKAGE_ROOT / "rviz" / "planner_validation.rviz").read_text(
        encoding="utf-8"
    )
    assert "Value: /initialpose" in document
    assert "Value: /goal_pose" in document
    assert "Value: /planning_test/path" in document
    assert "Value: /planning_test/waypoints" in document
    assert "rviz_default_plugins/PoseArray" in document
    assert "rviz_default_plugins/PointCloud2" in document
    assert "Value: /planning_test/map_3d" in document


def test_native_nav_through_poses_validation_uses_real_bt_and_mock_controller():
    source = (
        PACKAGE_ROOT
        / "ackermann"
        / "launch"
        / "ackermann_nav_through_poses_validation.launch.py"
    ).read_text(encoding="utf-8")

    assert 'package="nav2_bt_navigator"' in source
    assert 'package="pcl_ros"' in source
    assert 'executable="pcd_to_pointcloud"' in source
    assert '"/planning_test/map_3d"' in source
    assert 'DeclareLaunchArgument("show_3d_map", default_value="false")' in source
    assert 'executable="planner_validation_follow_path.py"' in source
    assert "navigate_through_poses_w_replanning_ackermann.xml" in source
    assert 'name="lifecycle_manager_navigation"' in source
    for forbidden in (
        'package="nav2_controller"',
        "ackermann_chassis_can_node",
        "ackermann_chassis_serial_node",
        "lslidar_driver",
        "hipnuc_imu",
    ):
        assert forbidden not in source


def test_native_nav_through_poses_rviz_uses_official_panel_and_goal_tool():
    document = (
        PACKAGE_ROOT / "rviz" / "nav_through_poses_validation.rviz"
    ).read_text(encoding="utf-8")

    assert "Class: nav2_rviz_plugins/Navigation 2" in document
    assert "Class: nav2_rviz_plugins/GoalTool" in document
    assert "Class: rviz_default_plugins/PointCloud2" in document
    assert "Value: /planning_test/map_3d" in document
    assert "Value: /waypoints" in document
    assert "Value: /planning_test/path" in document


def test_dry_run_follow_path_publishes_one_received_path_without_cmd_vel():
    source = (
        PACKAGE_ROOT / "scripts" / "planner_validation_follow_path.py"
    ).read_text(encoding="utf-8")

    assert "ActionServer" in source
    assert "FollowPath" in source
    assert '"/follow_path"' in source
    assert '"/planning_test/path"' in source
    assert "ClearEntireCostmap" in source
    assert "Wait" in source
    assert "cmd_vel" not in source
