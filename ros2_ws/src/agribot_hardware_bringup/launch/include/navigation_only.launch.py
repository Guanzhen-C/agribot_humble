import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    nav2_bt_navigator_share = get_package_share_directory("nav2_bt_navigator")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        root_key="",
        param_rewrites={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "default_nav_to_pose_bt_xml": LaunchConfiguration(
                "default_nav_to_pose_bt_xml"
            ),
            "default_nav_through_poses_bt_xml": LaunchConfiguration(
                "default_nav_through_poses_bt_xml"
            ),
            "odom_topic": LaunchConfiguration("odom_topic"),
        },
        convert_types=True,
    )
    remappings = [
        ("cmd_vel", "/nav2/cmd_vel"),
        ("map", LaunchConfiguration("map_topic")),
        ("/map", LaunchConfiguration("map_topic")),
    ]

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    hardware_share, "config", "nav2_dwb_navsat.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "default_nav_to_pose_bt_xml",
                default_value=os.path.join(
                    nav2_bt_navigator_share,
                    "behavior_trees",
                    "navigate_to_pose_w_replanning_and_recovery.xml",
                ),
            ),
            DeclareLaunchArgument(
                "default_nav_through_poses_bt_xml",
                default_value=os.path.join(
                    nav2_bt_navigator_share,
                    "behavior_trees",
                    "navigate_through_poses_w_replanning_and_recovery.xml",
                ),
            ),
            GroupAction(
                actions=[
                    Node(
                        package="nav2_controller",
                        executable="controller_server",
                        name="controller_server",
                        output="screen",
                        remappings=remappings,
                        parameters=[configured_params],
                    ),
                    Node(
                        package="nav2_planner",
                        executable="planner_server",
                        name="planner_server",
                        output="screen",
                        remappings=remappings,
                        parameters=[configured_params],
                    ),
                    Node(
                        package="nav2_behaviors",
                        executable="behavior_server",
                        name="behavior_server",
                        output="screen",
                        remappings=remappings,
                        parameters=[configured_params],
                    ),
                    Node(
                        package="nav2_bt_navigator",
                        executable="bt_navigator",
                        name="bt_navigator",
                        output="screen",
                        remappings=remappings,
                        parameters=[configured_params],
                    ),
                    Node(
                        package="nav2_waypoint_follower",
                        executable="waypoint_follower",
                        name="waypoint_follower",
                        output="screen",
                        remappings=remappings,
                        parameters=[configured_params],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_navigation",
                        output="screen",
                        parameters=[
                            {"use_sim_time": use_sim_time, "autostart": autostart},
                            {
                                "node_names": [
                                    "controller_server",
                                    "planner_server",
                                    "behavior_server",
                                    "bt_navigator",
                                    "waypoint_follower",
                                ]
                            },
                        ],
                    ),
                ]
            ),
        ]
    )
