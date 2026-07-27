import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    autonomy_share = get_package_share_directory("agribot_autonomy")
    fast_livo_share = get_package_share_directory("fast_livo")

    parameter_blackboard = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="parameter_blackboard",
        output="screen",
        parameters=[LaunchConfiguration("camera_params_file")],
    )

    fast_livo2 = Node(
        package="fast_livo",
        executable="fastlivo_mapping",
        name="laserMapping",
        output="screen",
        parameters=[
            LaunchConfiguration("fastlivo2_config_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "fastlivo2_config_file",
                default_value=os.path.join(
                    autonomy_share, "config", "fast_livo2_sim.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "camera_params_file",
                default_value=os.path.join(
                    autonomy_share, "config", "fast_livo2_camera_sim.yaml"
                ),
            ),
            DeclareLaunchArgument("fastlivo2_visualize", default_value="false"),
            DeclareLaunchArgument(
                "fastlivo2_output_odom_topic", default_value="/fastlivo/odometry"
            ),
            parameter_blackboard,
            Node(
                package="agribot_autonomy",
                executable="imu_frame_bridge.py",
                name="fastlivo2_imu_frame_bridge",
                output="screen",
            ),
            TimerAction(period=1.0, actions=[fast_livo2]),
            TimerAction(
                period=1.5,
                actions=[
                    Node(
                        package="agribot_autonomy",
                        executable="fastlio_odom_bridge.py",
                        name="fastlivo2_odom_bridge",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": LaunchConfiguration("use_sim_time"),
                                "input_odom_topic": "/aft_mapped_to_init",
                                "output_odom_topic": LaunchConfiguration(
                                    "fastlivo2_output_odom_topic"
                                ),
                                "input_odom_frame": "camera_init",
                                "input_body_frame": "aft_mapped",
                                "output_odom_frame": "odom",
                                "output_base_frame": "base_link",
                                "is_simulation": True,
                                "stamp_with_current_time": True,
                                "publish_tf": True,
                            }
                        ],
                    )
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    os.path.join(fast_livo_share, "rviz_cfg", "fast_livo2.rviz"),
                ],
                condition=IfCondition(LaunchConfiguration("fastlivo2_visualize")),
                output="screen",
            ),
        ]
    )
