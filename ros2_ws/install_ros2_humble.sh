#!/usr/bin/env bash
set -euo pipefail

readonly WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo apt-get update
sudo apt-get install -y \
  ros-humble-desktop \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-mppi-controller \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-tf-transformations \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-argcomplete

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

set +u
source /opt/ros/humble/setup.bash
set -u
rosdep install \
  --from-paths "${WORKSPACE_DIR}/src" \
  --ignore-src \
  --rosdistro humble \
  -r -y

echo "ROS 2 Humble dependencies are installed."
echo "Build with ${WORKSPACE_DIR}/build_ros2_humble.sh."
echo "Then source ${WORKSPACE_DIR}/setup_ros2_humble.sh before running the workspace."
