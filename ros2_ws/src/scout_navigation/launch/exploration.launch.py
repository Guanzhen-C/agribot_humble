from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "scout_navigation: frontier exploration is intentionally inactive in "
                    "the ROS 2 Humble baseline. The upstream ROS 1 launch kept the "
                    "exploration nodes commented out, so this entrypoint currently acts "
                    "as a compatibility placeholder."
                )
            )
        ]
    )
