import os
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _write_orchard_geometry_sdf(gazebo_share):
    mesh_dir = os.path.join(gazebo_share, "meshes")
    orchard_model_file = os.path.join(
        tempfile.gettempdir(), "scout_orchard_geometry.generated.sdf"
    )
    orchard_model_xml = f"""<?xml version="1.0"?>
<sdf version="1.7">
  <model name="orchard_geometry">
    <static>true</static>
    <link name="orchard_world_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
      <visual name="ground">
        <geometry>
          <mesh>
            <uri>file://{mesh_dir}/orchard_world.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="trunks">
        <geometry>
          <mesh>
            <uri>file://{mesh_dir}/orchard_trunks.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="leaves">
        <geometry>
          <mesh>
            <uri>file://{mesh_dir}/orchard_leaves.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="ground_collision">
        <geometry>
          <mesh>
            <uri>file://{mesh_dir}/orchard_world.dae</uri>
          </mesh>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>100.0</mu>
              <mu2>50.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <collision name="trunks_collision">
        <geometry>
          <mesh>
            <uri>file://{mesh_dir}/orchard_trunks.dae</uri>
          </mesh>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""
    with open(orchard_model_file, "w", encoding="utf-8") as orchard_file:
        orchard_file.write(orchard_model_xml)
    return orchard_model_file


def generate_launch_description():
    gazebo_share = get_package_share_directory("scout_gazebo")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    scout_base_share = get_package_share_directory("scout_base")
    viz_share = get_package_share_directory("scout_viz")
    orchard_model_file = _write_orchard_geometry_sdf(gazebo_share)
    have_xvfb = shutil.which("Xvfb") is not None
    default_use_xvfb = "true" if not os.environ.get("DISPLAY") else "false"
    system_model_paths = [
        model_path
        for model_path in (
            "/usr/share/gazebo-11/models",
            "/usr/share/gazebo/models",
            os.path.expanduser("~/.gazebo/models"),
            os.path.dirname(gazebo_share),
        )
        if os.path.isdir(model_path)
    ]
    gazebo_model_path = os.pathsep.join(
        system_model_paths
        + ([os.environ["GAZEBO_MODEL_PATH"]] if os.environ.get("GAZEBO_MODEL_PATH") else [])
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_name",
                default_value=os.path.join(gazebo_share, "worlds", "orchard_barriers.world"),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_xvfb", default_value=default_use_xvfb),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("spawn_orchard_geometry", default_value="false"),
            DeclareLaunchArgument("publish_simulated_odom", default_value="false"),
            DeclareLaunchArgument("publish_ground_truth", default_value="false"),
            DeclareLaunchArgument("publish_joint_states", default_value="true"),
            DeclareLaunchArgument("x", default_value="2.0"),
            DeclareLaunchArgument("y", default_value="36.0"),
            DeclareLaunchArgument("z", default_value="0.146336"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("laser_enabled", default_value="false"),
            DeclareLaunchArgument("laser_3d_enabled", default_value="false"),
            DeclareLaunchArgument("laser_3d_xyz", default_value="0 0 0"),
            DeclareLaunchArgument("laser_3d_rpy", default_value="0 0 0"),
            DeclareLaunchArgument("laser_3d_topic", default_value="points"),
            DeclareLaunchArgument("laser_3d_update_rate", default_value="10"),
            DeclareLaunchArgument("laser_3d_horizontal_samples", default_value="720"),
            DeclareLaunchArgument("laser_3d_vertical_samples", default_value="16"),
            DeclareLaunchArgument("laser_3d_min_range", default_value="0.3"),
            DeclareLaunchArgument("laser_3d_max_range", default_value="25.0"),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            DeclareLaunchArgument("xvfb_display", default_value=":100"),
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path),
            *(
                [
                    ExecuteProcess(
                        cmd=[
                            "Xvfb",
                            LaunchConfiguration("xvfb_display"),
                            "-screen",
                            "0",
                            "1280x1024x24",
                        ],
                        output="screen",
                        condition=IfCondition(LaunchConfiguration("use_xvfb")),
                    ),
                    SetEnvironmentVariable(
                        "DISPLAY",
                        LaunchConfiguration("xvfb_display"),
                        condition=IfCondition(LaunchConfiguration("use_xvfb")),
                    ),
                    SetEnvironmentVariable(
                        "LIBGL_ALWAYS_SOFTWARE",
                        "1",
                        condition=IfCondition(LaunchConfiguration("use_xvfb")),
                    ),
                    SetEnvironmentVariable(
                        "MESA_GL_VERSION_OVERRIDE",
                        "3.3",
                        condition=IfCondition(LaunchConfiguration("use_xvfb")),
                    ),
                ]
                if have_xvfb
                else []
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
                ),
                launch_arguments={
                    "world": LaunchConfiguration("world_name"),
                    "gui": LaunchConfiguration("gui"),
                    "headless": LaunchConfiguration("headless"),
                    "verbose": "false",
                }.items(),
                condition=UnlessCondition(LaunchConfiguration("use_xvfb")),
            ),
            TimerAction(
                period=1.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
                        ),
                        launch_arguments={
                            "world": LaunchConfiguration("world_name"),
                            "gui": LaunchConfiguration("gui"),
                            "headless": LaunchConfiguration("headless"),
                            "verbose": "false",
                        }.items(),
                    )
                ],
                condition=IfCondition(LaunchConfiguration("use_xvfb")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_share, "launch", "spawn_scout.launch.py")
                ),
                launch_arguments={
                    "x": LaunchConfiguration("x"),
                    "y": LaunchConfiguration("y"),
                    "z": LaunchConfiguration("z"),
                    "yaw": LaunchConfiguration("yaw"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "laser_enabled": LaunchConfiguration("laser_enabled"),
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
                    "publish_joint_states": LaunchConfiguration("publish_joint_states"),
                }.items(),
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity",
                    "orchard_geometry",
                    "-file",
                    orchard_model_file,
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("spawn_orchard_geometry")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(scout_base_share, "launch", "base.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("publish_simulated_odom")),
                launch_arguments={
                    "simulated_robot": "true",
                    "odom_topic_name": "odom",
                    "odom_frame": "odom",
                    "base_frame": "base_link",
                    "pub_tf": "true",
                }.items(),
            ),
            Node(
                package="scout_gazebo",
                executable="odom_to_ground_truth.py",
                name="odom_to_ground_truth",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/odom",
                        "output_topic": "/base_pose_ground_truth",
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("publish_ground_truth")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(viz_share, "rviz", "robot.rviz")],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
