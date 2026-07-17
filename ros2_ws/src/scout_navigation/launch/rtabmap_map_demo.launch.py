import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    scout_control_share = get_package_share_directory("scout_control")
    scout_gazebo_share = get_package_share_directory("scout_gazebo")
    scout_navigation_share = get_package_share_directory("scout_navigation")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_xvfb", default_value="true"),
            DeclareLaunchArgument("xvfb_display", default_value=":100"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("enable_keyboard_teleop", default_value="false"),
            DeclareLaunchArgument("x", default_value="2.0"),
            DeclareLaunchArgument("y", default_value="36.0"),
            DeclareLaunchArgument("z", default_value="0.146336"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "database_path",
                default_value=os.path.join(
                    scout_navigation_share, "maps", "rtabmap_d435i_orchard1.db"
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(scout_gazebo_share, "launch", "scout_orchard_world.launch.py")
                ),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "headless": LaunchConfiguration("headless"),
                    "use_xvfb": LaunchConfiguration("use_xvfb"),
                    "xvfb_display": LaunchConfiguration("xvfb_display"),
                    "rviz": "false",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "publish_simulated_odom": "false",
                    "publish_joint_states": "true",
                    "x": LaunchConfiguration("x"),
                    "y": LaunchConfiguration("y"),
                    "z": LaunchConfiguration("z"),
                    "yaw": LaunchConfiguration("yaw"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(scout_control_share, "launch", "control.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "enable_ekf": "false",
                    "enable_twist_mux": "true",
                    "cmd_vel_out_topic": "/cmd_vel",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        scout_control_share, "launch", "teleop_keyboard.launch.py"
                    )
                ),
                condition=IfCondition(LaunchConfiguration("enable_keyboard_teleop")),
                launch_arguments={"cmd_vel_topic": "/joy_teleop/cmd_vel"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        scout_navigation_share, "launch", "rtabmap_map.launch.py"
                    )
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "start_robot": "false",
                    "launch_rviz": LaunchConfiguration("rviz"),
                    "database_path": LaunchConfiguration("database_path"),
                    "approx_sync": "true",
                    "imu_topic": "/imu/data",
                    "wait_imu_to_init": "false",
                }.items(),
            ),
        ]
    )
