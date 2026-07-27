# Vikit: Vision-Kit for Robotics

Vikit is a versatile collection of C++ tools and utilities designed for computer vision and robotics projects. This version has been modernized for the ROS2 ecosystem (specifically tested on Jazzy and Humble) with a focus on high-performance parameter handling and cross-node compatibility.

---

## 🔒 Disclaimer & Acknowledgments

### Usage Policy
This software is provided for **educational and research purposes only**. It shall **not be used for any commercial purposes**.

### Acknowledgments
We extend our deepest gratitude to the original developers and contributors of the projects that served as the foundation for this kit:
*   [uzh-rpg/rpg_vikit](https://github.com/uzh-rpg/rpg_vikit)
*   [xuankuzcr/rpg_vikit](https://github.com/xuankuzcr/rpg_vikit)
*   [uavfly/vikit](https://github.com/uavfly/vikit)

---

## 🚀 Key Improvements in ROS2

### Optimized Parameter Fetching Architecture
One of the most significant challenges in migrating from ROS1 to ROS2 is the removal of the global Parameter Server. In ROS2, parameters are local to each node. Vikit addresses this through a multi-tiered fetching strategy in `params_helper.hpp`:

1.  **Native Node Access**: Direct, high-speed access to parameters owned by the current node handle.
2.  **`SyncParametersClient` (High Performance)**: For cross-node parameter access (e.g., retrieving camera intrinsics from a central `parameter_blackboard`). This utilizes optimized ROS2 Service calls to achieve microsecond-level latency, avoiding the overhead of CLI tools.
3.  **Command-Line Fallback**: A robust fallback mechanism using `popen` to interface with the ROS2 CLI (`ros2 param get`), ensuring parameter retrieval even in complex edge cases where service clients might be restricted.

This architecture ensures that vision components can load dozens of camera parameters nearly instantaneously, a critical requirement for real-time SLAM and VIO systems.

---

## 🛠 Installation Guide

### Prerequisites: Sophus
Vikit relies on Sophus for Lie groups. It is recommended to use version `1.22.10`.

```bash
git clone https://github.com/strasdat/Sophus.git -b 1.22.10
cd Sophus && mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

### Building `vikit_common`
`vikit_common` is a pure CMake package and can be installed globally.

```bash
cd vikit_common
mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

### Building `vikit_ros`
`vikit_ros` is integrated into the ROS2 workspace and should be built using `colcon`.

```bash
# Move vikit_ros to your workspace src directory
cd ~/ros2_ws
colcon build --symlink-install --packages-select vikit_ros
```

---

## 📅 Maintenance Info
*   **Last Update**: December 2025
*   **Target Systems**: Ubuntu 22.04 (Humble) / 24.04 (Jazzy)
*   **Compiler**: C++17 compliant (GCC 9+)
