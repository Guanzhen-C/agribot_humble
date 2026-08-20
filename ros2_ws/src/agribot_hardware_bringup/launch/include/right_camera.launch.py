import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validate_driver(context):
    driver = LaunchConfiguration("camera_driver").perform(context)
    if driver not in ("hikrobot_mvs", "usb_cam"):
        raise RuntimeError("camera_driver必须是hikrobot_mvs或usb_cam")
    return []


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    hikrobot_share = get_package_share_directory("hikrobot_mvs_ros2")
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_driver", default_value="hikrobot_mvs"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "hikrobot_camera_config",
                default_value=os.path.join(
                    hikrobot_share, "config", "mv_cu013_a0uc.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "hikrobot_camera_serial", default_value="DB0447659"
            ),
            DeclareLaunchArgument("hikrobot_trigger_enable", default_value="false"),
            DeclareLaunchArgument(
                "usb_camera_config",
                default_value=os.path.join(
                    hardware_share, "config", "right_camera.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "right_camera_device", default_value="/dev/agribot_right_camera"
            ),
            OpaqueFunction(function=_validate_driver),
            Node(
                package="hikrobot_mvs_ros2",
                executable="hikrobot_mvs_camera_node",
                name="agribot_right_camera",
                output="screen",
                parameters=[
                    LaunchConfiguration("hikrobot_camera_config"),
                    {
                        "serial_number": LaunchConfiguration(
                            "hikrobot_camera_serial"
                        ),
                        "trigger_enable": LaunchConfiguration(
                            "hikrobot_trigger_enable"
                        ),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
                condition=LaunchConfigurationEquals(
                    "camera_driver", "hikrobot_mvs"
                ),
            ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="agribot_right_camera",
                output="screen",
                parameters=[
                    LaunchConfiguration("usb_camera_config"),
                    {
                        "video_device": LaunchConfiguration(
                            "right_camera_device"
                        ),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
                remappings=[
                    ("image_raw", "/camera/rgb/image_raw"),
                    ("camera_info", "/camera/rgb/camera_info"),
                ],
                condition=LaunchConfigurationEquals("camera_driver", "usb_cam"),
            ),
        ]
    )
