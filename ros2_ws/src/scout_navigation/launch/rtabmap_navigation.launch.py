import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scout_navigation_share = get_package_share_directory("scout_navigation")
    nav2_bt_navigator_share = get_package_share_directory("nav2_bt_navigator")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "database_path",
                default_value=os.path.join(
                    scout_navigation_share, "maps", "rtabmap_d435i_orchard1.db"
                ),
            ),
            DeclareLaunchArgument("rtabmap_args", default_value=""),
            DeclareLaunchArgument(
                "grid_args",
                default_value=(
                    "--Grid/MaxGroundHeight 0.2 "
                    "--Grid/MaxObstacleHeight 2 "
                    "--Grid/NormalsSegmentation false "
                    "--Optimizer/GravitySigma 0.3 "
                    "--Rtabmap/DetectionRate 5"
                ),
            ),
            DeclareLaunchArgument("rgb_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument(
                "depth_topic", default_value="/camera/depth/image_rect_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/color/camera_info"
            ),
            DeclareLaunchArgument(
                "depth_camera_info_topic",
                default_value="/camera/depth/camera_info",
            ),
            DeclareLaunchArgument("imu_topic", default_value="/rtabmap/imu"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("approx_sync", default_value="false"),
            DeclareLaunchArgument("wait_imu_to_init", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("start_robot", default_value="false"),
            DeclareLaunchArgument("port_name", default_value="can1"),
            DeclareLaunchArgument("agilex_joystick", default_value="false"),
            DeclareLaunchArgument("simulated_robot", default_value="false"),
            DeclareLaunchArgument("robot_namespace", default_value="/"),
            DeclareLaunchArgument("odom_topic_name", default_value="odom"),
            DeclareLaunchArgument("pub_tf", default_value="false"),
            DeclareLaunchArgument("base_enabled", default_value="true"),
            DeclareLaunchArgument("control_enabled", default_value="true"),
            DeclareLaunchArgument("move_nav2", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("launch_rtabmapviz", default_value="false"),
            DeclareLaunchArgument("nav2_map_topic", default_value="/rtabmap/grid_prob_map"),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=os.path.join(
                    scout_navigation_share, "config", "nav2_params.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "default_nav_to_pose_bt_xml",
                default_value=os.path.join(
                    nav2_bt_navigator_share,
                    "behavior_trees",
                    "navigate_w_replanning_time.xml",
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        scout_navigation_share, "launch", "include", "robot_bringup.launch.py"
                    )
                ),
                condition=IfCondition(LaunchConfiguration("start_robot")),
                launch_arguments={
                    "port_name": LaunchConfiguration("port_name"),
                    "agilex_joystick": LaunchConfiguration("agilex_joystick"),
                    "simulated_robot": LaunchConfiguration("simulated_robot"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "robot_namespace": LaunchConfiguration("robot_namespace"),
                    "odom_topic_name": LaunchConfiguration("odom_topic_name"),
                    "pub_tf": LaunchConfiguration("pub_tf"),
                    "base_enabled": LaunchConfiguration("base_enabled"),
                    "control_enabled": LaunchConfiguration("control_enabled"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("rtabmap_launch"), "/launch/rtabmap.launch.py"]
                ),
                launch_arguments={
                    "rtabmap_args": [
                        LaunchConfiguration("rtabmap_args"),
                        " ",
                        LaunchConfiguration("grid_args"),
                    ],
                    "rgb_topic": LaunchConfiguration("rgb_topic"),
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "depth_camera_info_topic": LaunchConfiguration(
                        "depth_camera_info_topic"
                    ),
                    "approx_sync": LaunchConfiguration("approx_sync"),
                    "wait_imu_to_init": LaunchConfiguration("wait_imu_to_init"),
                    "imu_topic": LaunchConfiguration("imu_topic"),
                    "frame_id": LaunchConfiguration("frame_id"),
                    "odom_topic": LaunchConfiguration("odom_topic"),
                    "visual_odometry": "false",
                    "database_path": LaunchConfiguration("database_path"),
                    "localization": "true",
                    "rviz": "false",
                    "rtabmapviz": LaunchConfiguration("launch_rtabmapviz"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    os.path.join(scout_navigation_share, "rviz", "rtabmap.rviz"),
                ],
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        scout_navigation_share,
                        "launch",
                        "include",
                        "navigation_only.launch.py",
                    )
                ),
                condition=IfCondition(LaunchConfiguration("move_nav2")),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "autostart": LaunchConfiguration("autostart"),
                    "map_topic": LaunchConfiguration("nav2_map_topic"),
                    "params_file": LaunchConfiguration("nav2_params_file"),
                    "default_nav_to_pose_bt_xml": LaunchConfiguration(
                        "default_nav_to_pose_bt_xml"
                    ),
                }.items(),
            ),
        ]
    )
