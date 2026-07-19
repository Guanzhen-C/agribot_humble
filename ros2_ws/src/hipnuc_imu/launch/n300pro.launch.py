# Copyright 2026 cgz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("hipnuc_imu")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file", default_value=os.path.join(share, "config", "n300pro.yaml")
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C2C082600-if00",
            ),
            Node(
                package="hipnuc_imu",
                executable="hipnuc_imu_node",
                name="hipnuc_imu",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"serial_port": LaunchConfiguration("serial_port")},
                ],
            ),
        ]
    )
