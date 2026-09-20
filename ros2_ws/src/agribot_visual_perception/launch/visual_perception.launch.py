from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="canny"),
            DeclareLaunchArgument(
                "input_topic", default_value="/camera/rgb/image_raw"
            ),
            DeclareLaunchArgument(
                "output_topic", default_value="/vision/annotated_image"
            ),
            DeclareLaunchArgument("max_rate_hz", default_value="10.0"),
            Node(
                package="agribot_visual_perception",
                executable="visual_perception_node",
                name="visual_perception",
                output="screen",
                parameters=[
                    {
                        "mode": LaunchConfiguration("mode"),
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "max_rate_hz": LaunchConfiguration("max_rate_hz"),
                    }
                ],
            ),
        ]
    )
