# Lite6 模型来源

- 仓库：https://github.com/xArm-Developer/xarm_ros2
- 分支：humble
- 固定提交：62936f7ea1846a85f7350de2c4c18f39e6d19715
- 网格：`xarm_description/meshes/lite6/visual/*.stl`，原样复制。
- 夹爪：`xarm_description/meshes/gripper/lite/visual/{shell,finger1,finger2}.stl`，原样复制；shell 在本包中命名为 gripper_shell。
- 夹爪安装和行程：`xarm_description/urdf/gripper/uflite_gripper.urdf.xacro`，在 link6 末端固定，两个手指分别沿 ±Y 方向移动 8.9 mm。
- 关节安装变换：`xarm_description/config/kinematics/default/lite6_default_kinematics.yaml`。
- 关节限位：`xarm_description/urdf/lite6/lite6.urdf.xacro`，非 limited 模式。
- 许可证：BSD-3-Clause，见同目录 LICENSE.txt。

这里只复用官方几何和运动链。车载安装点为名义仿真值；低质量、无重力、无碰撞的展示关节不代表真实机械臂动力学、载荷、抓取或安全验证。未包含硬件驱动、SDK 或 MoveIt。
