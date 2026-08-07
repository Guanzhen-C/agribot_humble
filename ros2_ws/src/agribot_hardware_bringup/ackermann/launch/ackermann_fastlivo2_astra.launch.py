import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")

    openni2_library_path = [
        LaunchConfiguration("openni2_runtime"),
        ":",
        EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
    ]

    camera = Node(
        package="openni2_camera",
        executable="openni2_camera_driver",
        namespace="camera",
        name="openni2_camera",
        output="screen",
        parameters=[
            {
                "color_mode": "VGA_30Hz",
                # Depth remains lazy and is not consumed by FAST-LIVO2.
                "depth_mode": "QVGA_30Hz",
                "color_depth_synchronization": False,
                "depth_registration": False,
                "use_device_time": False,
                "enable_reconnect": True,
            }
        ],
        additional_env={"LD_LIBRARY_PATH": openni2_library_path},
        condition=IfCondition(LaunchConfiguration("start_camera")),
    )

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
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_sensors", default_value="true"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_cloud_rate", default_value="2.0"),
            DeclareLaunchArgument("rviz_image_rate", default_value="10.0"),
            DeclareLaunchArgument("rviz_path_rate", default_value="1.0"),
            DeclareLaunchArgument(
                "openni2_runtime",
                default_value="/opt/orbbec/openni2",
                description="Directory containing libOpenNI2 and the Astra plugin",
            ),
            DeclareLaunchArgument(
                "fastlivo2_config_file",
                default_value=os.path.join(
                    hardware_share, "config", "fast_livo2_c16_astra.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "camera_params_file",
                default_value=os.path.join(
                    hardware_share,
                    "config",
                    "fast_livo2_camera_astra_640.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=os.path.join(
                    hardware_share,
                    "rviz",
                    "fast_livo2_astra.rviz",
                ),
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                hardware_share,
                                "launch",
                                "sensors.launch.py",
                            )
                        ),
                        launch_arguments={
                            "start_lidar": "true",
                            "start_imu": "true",
                            "start_rtk": "false",
                            "rviz": "false",
                        }.items(),
                    )
                ],
                condition=IfCondition(LaunchConfiguration("start_sensors")),
            ),
            camera,
            parameter_blackboard,
            TimerAction(period=3.0, actions=[fast_livo2]),
            TimerAction(
                period=3.5,
                actions=[
                    Node(
                        package="agribot_hardware_bringup",
                        executable="fastlio_odom_bridge.py",
                        name="fastlivo2_odom_bridge",
                        output="screen",
                        parameters=[
                            {
                                "input_odom_topic": "/aft_mapped_to_init",
                                "output_odom_topic": "/fastlivo/odometry",
                                "input_odom_frame": "camera_init",
                                "input_body_frame": "aft_mapped",
                                "output_odom_frame": "odom",
                                "output_base_frame": "base_link",
                                "publish_tf": True,
                                "stamp_with_current_time": False,
                                "is_simulation": False,
                                "base_to_body_xyz": [0.1425, 0.0, 0.143],
                                "base_to_body_rpy": [
                                    0.000572424,
                                    -0.009139547,
                                    -0.000002616,
                                ],
                            }
                        ],
                    ),
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="odom_to_fastlivo2_world",
                        arguments=[
                            "--frame-id",
                            "odom",
                            "--child-frame-id",
                            "camera_init",
                        ],
                    ),
                ],
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="fastlivo2_cloud_throttle",
                arguments=[
                    "messages",
                    "/cloud_registered",
                    LaunchConfiguration("rviz_cloud_rate"),
                    "/cloud_registered_rviz",
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="fastlivo2_image_throttle",
                arguments=[
                    "messages",
                    "/rgb_img",
                    LaunchConfiguration("rviz_image_rate"),
                    "/rgb_img_rviz",
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="fastlivo2_path_throttle",
                arguments=[
                    "messages",
                    "/path",
                    LaunchConfiguration("rviz_path_rate"),
                    "/path_rviz",
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    LaunchConfiguration("rviz_config_file"),
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
