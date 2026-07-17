import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_scout_actions(context):
    desc_share = get_package_share_directory("scout_description")
    xacro_exec = os.path.join(get_package_prefix("xacro"), "bin", "xacro")
    description_file = os.path.join(desc_share, "urdf", "scout.urdf.xacro")
    empty_urdf = os.path.join(desc_share, "urdf", "empty.urdf")

    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)
    laser_3d_enabled = LaunchConfiguration("laser_3d_enabled").perform(context)
    laser_3d_xyz = LaunchConfiguration("laser_3d_xyz").perform(context)
    laser_3d_rpy = LaunchConfiguration("laser_3d_rpy").perform(context)
    laser_3d_topic = LaunchConfiguration("laser_3d_topic").perform(context)
    laser_3d_update_rate = LaunchConfiguration("laser_3d_update_rate").perform(context)
    laser_3d_horizontal_samples = LaunchConfiguration("laser_3d_horizontal_samples").perform(
        context
    )
    laser_3d_vertical_samples = LaunchConfiguration("laser_3d_vertical_samples").perform(context)
    laser_3d_min_range = LaunchConfiguration("laser_3d_min_range").perform(context)
    laser_3d_max_range = LaunchConfiguration("laser_3d_max_range").perform(context)
    publish_odom_tf = LaunchConfiguration("publish_odom_tf").perform(context)

    spawn_urdf = os.path.join(tempfile.gettempdir(), "scout_spawn.generated.urdf")
    xacro_cmd = [
        xacro_exec,
        description_file,
        f"robot_namespace:={robot_namespace}",
        f"urdf_extras:={empty_urdf}",
        f"laser_3d_enabled:={laser_3d_enabled}",
        f"laser_3d_xyz:={laser_3d_xyz}",
        f"laser_3d_rpy:={laser_3d_rpy}",
        f"laser_3d_topic:={laser_3d_topic}",
        f"laser_3d_update_rate:={laser_3d_update_rate}",
        f"laser_3d_horizontal_samples:={laser_3d_horizontal_samples}",
        f"laser_3d_vertical_samples:={laser_3d_vertical_samples}",
        f"laser_3d_min_range:={laser_3d_min_range}",
        f"laser_3d_max_range:={laser_3d_max_range}",
        f"publish_odom_tf:={publish_odom_tf}",
    ]
    urdf_xml = subprocess.check_output(xacro_cmd, text=True)
    with open(spawn_urdf, "w", encoding="utf-8") as handle:
        handle.write(urdf_xml)

    return [
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity",
                "scout",
                "-file",
                spawn_urdf,
                "-package_to_model",
                "-robot_namespace",
                robot_namespace,
                "-x",
                x,
                "-y",
                y,
                "-z",
                z,
                "-Y",
                yaw,
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    desc_share = get_package_share_directory("scout_description")
    default_model_file = os.path.join(desc_share, "urdf", "scout.sdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="/"),
            DeclareLaunchArgument("model_file", default_value=default_model_file),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("publish_joint_states", default_value="true"),
            DeclareLaunchArgument("laser_3d_enabled", default_value="false"),
            DeclareLaunchArgument("laser_3d_xyz", default_value="0 0 0"),
            DeclareLaunchArgument("laser_3d_rpy", default_value="0 0 0"),
            DeclareLaunchArgument("laser_3d_topic", default_value="points"),
            DeclareLaunchArgument("laser_3d_update_rate", default_value="5"),
            DeclareLaunchArgument("laser_3d_horizontal_samples", default_value="360"),
            DeclareLaunchArgument("laser_3d_vertical_samples", default_value="16"),
            DeclareLaunchArgument("laser_3d_min_range", default_value="0.3"),
            DeclareLaunchArgument("laser_3d_max_range", default_value="25.0"),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(desc_share, "launch", "description.launch.py")
                ),
                launch_arguments={
                    "robot_namespace": LaunchConfiguration("robot_namespace"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "publish_robot_state": "true",
                    "laser_3d_enabled": LaunchConfiguration("laser_3d_enabled"),
                    "laser_3d_xyz": LaunchConfiguration("laser_3d_xyz"),
                    "laser_3d_rpy": LaunchConfiguration("laser_3d_rpy"),
                    "laser_3d_topic": LaunchConfiguration("laser_3d_topic"),
                    "laser_3d_update_rate": LaunchConfiguration("laser_3d_update_rate"),
                    "laser_3d_horizontal_samples": LaunchConfiguration(
                        "laser_3d_horizontal_samples"
                    ),
                    "laser_3d_vertical_samples": LaunchConfiguration(
                        "laser_3d_vertical_samples"
                    ),
                    "laser_3d_min_range": LaunchConfiguration("laser_3d_min_range"),
                    "laser_3d_max_range": LaunchConfiguration("laser_3d_max_range"),
                    "publish_odom_tf": LaunchConfiguration("publish_odom_tf"),
                }.items(),
            ),
            OpaqueFunction(function=_spawn_scout_actions),
            Node(
                package="scout_gazebo",
                executable="simulated_joint_state_publisher.py",
                name="simulated_joint_state_publisher",
                output="screen",
                parameters=[
                    {
                        "odom_topic": "/odom",
                        "joint_state_topic": "/joint_states",
                        "wheel_radius": 0.16459,
                        "track_width": 0.58306,
                        "publish_hz": 50.0,
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("publish_joint_states")),
            ),
        ]
    )
