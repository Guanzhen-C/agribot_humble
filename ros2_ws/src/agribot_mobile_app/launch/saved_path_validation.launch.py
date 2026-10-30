"""Isolated kinematic validation using the production Nav2 differential settings."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    if os.environ.get("ROS_DOMAIN_ID") != "81" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("Validation requires ROS_DOMAIN_ID=81 ROS_LOCALHOST_ONLY=1")
    hardware = Path(get_package_share_directory("agribot_hardware_bringup")) / "differential"
    mobile = Path(get_package_share_directory("agribot_mobile_app"))
    params = RewrittenYaml(
        source_file=str(hardware / "config/nav2_params_differential_fastlivo_mapped.yaml"),
        root_key="", param_rewrites={"lattice_filepath": str(hardware / "config/motion_primitives/diff_5cm.json")},
        convert_types=True)
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="obstacle"),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", parameters=[params]),
        Node(package="nav2_controller", executable="controller_server", name="controller_server", parameters=[params],
             remappings=[("cmd_vel", "/saved_test/cmd_vel")]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager",
             parameters=[{"autostart": True, "node_names": ["planner_server", "controller_server"]}]),
        ExecuteProcess(cmd=["python3", str(mobile / "tools/test_saved_path_navigation.py")],
                       additional_env={"SAVED_PATH_TEST_SCENARIO": LaunchConfiguration("scenario")}, output="screen"),
    ])
