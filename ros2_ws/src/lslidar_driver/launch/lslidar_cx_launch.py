import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    driver_config = os.path.join(get_package_share_directory('lslidar_driver'),'config','lslidar_cx.yaml')
    rviz_config = os.path.join(get_package_share_directory('lslidar_driver'),'rviz','lslidar_cx.rviz')
    launch_rviz = LaunchConfiguration('rviz')

    p = subprocess.Popen("echo $ROS_DISTRO", stdout=subprocess.PIPE, shell=True)
    driver_node = ""
    rviz_node = ""
    ros_version = p.communicate()[0]

    if ros_version == b'dashing\n' or ros_version == b'eloquent\n':
        driver_node = Node(package='lslidar_driver',
                           executable='lslidar_driver_node',
                           name='lslidar_driver_node',
                           namespace='cx', # 与对应yaml文件中命名空间一致
                           output='screen',
                           parameters=[driver_config],
                           )
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace='cx',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(launch_rviz))
    else:
        driver_node = Node(package='lslidar_driver',
                           executable='lslidar_driver_node',
                           name='lslidar_driver_node',
                           namespace='cx', # 与对应yaml文件中命名空间一致
                           parameters=[driver_config],
                           output='screen'
                           )
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(launch_rviz)
        )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        driver_node,
        rviz_node
    ])
