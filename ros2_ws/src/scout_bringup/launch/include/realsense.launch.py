from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_no", default_value="''"),
            DeclareLaunchArgument("usb_port_id", default_value="''"),
            DeclareLaunchArgument("device_type", default_value="''"),
            DeclareLaunchArgument("json_file_path", default_value="''"),
            DeclareLaunchArgument(
                "camera",
                default_value=EnvironmentVariable(
                    "SCOUT_REALSENSE_TOPIC", default_value="camera"
                ),
            ),
            DeclareLaunchArgument("output", default_value="screen"),
            DeclareLaunchArgument("depth_profile", default_value="640,480,30"),
            DeclareLaunchArgument("color_profile", default_value="640,480,30"),
            DeclareLaunchArgument("infra_profile", default_value="640,480,30"),
            DeclareLaunchArgument("fisheye_profile", default_value="640,480,30"),
            DeclareLaunchArgument("gyro_fps", default_value="400"),
            DeclareLaunchArgument("accel_fps", default_value="250"),
            DeclareLaunchArgument("enable_depth", default_value="true"),
            DeclareLaunchArgument("enable_color", default_value="true"),
            DeclareLaunchArgument("enable_infra1", default_value="true"),
            DeclareLaunchArgument("enable_infra2", default_value="true"),
            DeclareLaunchArgument("enable_gyro", default_value="true"),
            DeclareLaunchArgument("enable_accel", default_value="true"),
            DeclareLaunchArgument("enable_sync", default_value="false"),
            DeclareLaunchArgument("align_depth", default_value="true"),
            DeclareLaunchArgument("pointcloud_enable", default_value="true"),
            DeclareLaunchArgument("clip_distance", default_value="-2.0"),
            DeclareLaunchArgument("linear_accel_cov", default_value="0.01"),
            DeclareLaunchArgument("initial_reset", default_value="false"),
            DeclareLaunchArgument("reconnect_timeout", default_value="6.0"),
            DeclareLaunchArgument("wait_for_device_timeout", default_value="-1.0"),
            DeclareLaunchArgument("unite_imu_method", default_value="2"),
            DeclareLaunchArgument("topic_odom_in", default_value="odom_in"),
            DeclareLaunchArgument("calib_odom_file", default_value="''"),
            DeclareLaunchArgument("tf_publish_rate", default_value="0.0"),
            DeclareLaunchArgument("target_frame", default_value="base_link"),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value=[LaunchConfiguration("camera"), "/depth/color/points"],
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value=[LaunchConfiguration("camera"), "/scan"],
            ),
            DeclareLaunchArgument(
                "imu_raw_topic",
                default_value=[LaunchConfiguration("camera"), "/imu"],
            ),
            DeclareLaunchArgument("imu_filtered_topic", default_value="/rtabmap/imu"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("realsense2_camera"), "/launch/rs_launch.py"]
                ),
                launch_arguments={
                    "camera_name": LaunchConfiguration("camera"),
                    "serial_no": LaunchConfiguration("serial_no"),
                    "usb_port_id": LaunchConfiguration("usb_port_id"),
                    "device_type": LaunchConfiguration("device_type"),
                    "json_file_path": LaunchConfiguration("json_file_path"),
                    "output": LaunchConfiguration("output"),
                    "depth_module.profile": LaunchConfiguration("depth_profile"),
                    "rgb_camera.profile": LaunchConfiguration("color_profile"),
                    "tracking_module.profile": LaunchConfiguration("fisheye_profile"),
                    "enable_depth": LaunchConfiguration("enable_depth"),
                    "enable_color": LaunchConfiguration("enable_color"),
                    "enable_infra1": LaunchConfiguration("enable_infra1"),
                    "enable_infra2": LaunchConfiguration("enable_infra2"),
                    "enable_gyro": LaunchConfiguration("enable_gyro"),
                    "enable_accel": LaunchConfiguration("enable_accel"),
                    "gyro_fps": LaunchConfiguration("gyro_fps"),
                    "accel_fps": LaunchConfiguration("accel_fps"),
                    "pointcloud.enable": LaunchConfiguration("pointcloud_enable"),
                    "enable_sync": LaunchConfiguration("enable_sync"),
                    "align_depth.enable": LaunchConfiguration("align_depth"),
                    "clip_distance": LaunchConfiguration("clip_distance"),
                    "linear_accel_cov": LaunchConfiguration("linear_accel_cov"),
                    "initial_reset": LaunchConfiguration("initial_reset"),
                    "reconnect_timeout": LaunchConfiguration("reconnect_timeout"),
                    "wait_for_device_timeout": LaunchConfiguration("wait_for_device_timeout"),
                    "unite_imu_method": LaunchConfiguration("unite_imu_method"),
                    "topic_odom_in": LaunchConfiguration("topic_odom_in"),
                    "calib_odom_file": LaunchConfiguration("calib_odom_file"),
                    "tf_publish_rate": LaunchConfiguration("tf_publish_rate"),
                }.items(),
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="realsense_to_laserscan",
                output="screen",
                remappings=[
                    ("cloud_in", LaunchConfiguration("pointcloud_topic")),
                    ("scan", LaunchConfiguration("scan_topic")),
                ],
                parameters=[
                    {
                        "target_frame": LaunchConfiguration("target_frame"),
                        "transform_tolerance": 1.0,
                        "min_height": 0.05,
                        "max_height": 1.0,
                        "angle_min": -0.7592182246175333,
                        "angle_max": 0.7592182246175333,
                        "angle_increment": 0.005,
                        "scan_time": 0.3333,
                        "range_min": 0.105,
                        "range_max": 8.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                        "concurrency_level": 1,
                    }
                ],
            ),
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter",
                output="screen",
                remappings=[
                    ("imu/data_raw", LaunchConfiguration("imu_raw_topic")),
                    ("imu/data", LaunchConfiguration("imu_filtered_topic")),
                ],
                parameters=[
                    {
                        "use_mag": False,
                        "publish_tf": False,
                        "world_frame": "enu",
                    }
                ],
            ),
        ]
    )
