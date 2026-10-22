import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scene_manifest", default_value=os.path.expanduser(
            "~/.local/share/agribot/k2_automation_gaussian_20261022/scene.json")),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("run_waypoints", default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("agribot_vehicle_description"), "launch",
            "configured_ackermann_sim.launch.py")), launch_arguments={
                "vehicle_id": "ackermann_current", "scene_manifest": LaunchConfiguration("scene_manifest"),
                "gui": LaunchConfiguration("gui"), "rviz": LaunchConfiguration("rviz"),
                "run_waypoints": LaunchConfiguration("run_waypoints"),
                "localization_mode": "fastlivo_rtk", "planner_mode": "smac_hybrid",
                "controller_mode": "mppi", "vision_mode": "off",
            }.items()),
    ])
