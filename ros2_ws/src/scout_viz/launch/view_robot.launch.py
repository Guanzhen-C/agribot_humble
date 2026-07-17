import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess


def generate_launch_description():
    viz_share = get_package_share_directory("scout_viz")
    clean_rviz_launcher = os.path.join(viz_share, "scripts", "run_rviz2_clean_env.sh")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(viz_share, "rviz", "robot.rviz"),
            ),
            ExecuteProcess(
                cmd=[clean_rviz_launcher, LaunchConfiguration("rviz_config")],
                output="screen",
            ),
        ]
    )
