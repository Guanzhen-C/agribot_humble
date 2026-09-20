from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="object_detection"),
            DeclareLaunchArgument(
                "input_topic", default_value="/camera/rgb/image_raw"
            ),
            DeclareLaunchArgument(
                "output_topic", default_value="/vision/annotated_image"
            ),
            DeclareLaunchArgument("result_topic", default_value="/vision/results"),
            DeclareLaunchArgument("model_dir", default_value=""),
            DeclareLaunchArgument("model_path", default_value=""),
            DeclareLaunchArgument("device", default_value=""),
            DeclareLaunchArgument("confidence", default_value="0.35"),
            DeclareLaunchArgument("image_size", default_value="640"),
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
                        "result_topic": LaunchConfiguration("result_topic"),
                        "model_dir": LaunchConfiguration("model_dir"),
                        "model_path": LaunchConfiguration("model_path"),
                        "device": LaunchConfiguration("device"),
                        "confidence": LaunchConfiguration("confidence"),
                        "image_size": LaunchConfiguration("image_size"),
                        "max_rate_hz": LaunchConfiguration("max_rate_hz"),
                    }
                ],
            ),
        ]
    )
