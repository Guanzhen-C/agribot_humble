# 车载机械臂安装和运动展示

本包是独立仿真组件，不放入真机 hardware 包，不连接 CAN 或真实机械臂。

Unity 装备背包中的“六轴机械臂”（Hand）可安装或卸载，仅阿克曼车 Car0 和履带车 Car3 提供该功能。沿用作者新版背包、安装位置和模型底座基准，以厂家分件替换静态展示网格用于动作演示。导出的 `vehicle_config.json` 保存安装点和选项；ROS 导出器生成相同的 URDF/SDF 六关节运动链和末端夹爪。机械臂不重复合入车体网格。默认未安装，原有车型、算法和路线不变。

使用现有车辆直接演示，不需要重新导出 Unity 模型：

```bash
source /opt/ros/humble/setup.bash
source ~/agribot_ws/ros2_ws/install/setup.bash
ros2 launch agribot_vehicle_description configured_ackermann_sim.launch.py \
  vehicle_id:=ackermann_current manipulator:=true arm_demo:=true \
  run_waypoints:=false gui:=false rviz:=true
# 履带车将 vehicle_id 改为 unity_car3
```

`manipulator` 默认 `configured`，遵循导出配置；显式 `false` 不安装。`arm_demo` 默认 `configured`，可显式 `false` 只显示不运动。收到车辆运动指令时，演示节点回到零位；这不是用于真实车辆的机械臂收拢联锁。

```bash
# 停止动作并回零
ros2 service call /manipulator/demo_enabled std_srvs/srv/SetBool '{data: false}'
# 再次演示
ros2 service call /manipulator/demo_enabled std_srvs/srv/SetBool '{data: true}'
```

关节指令为 `/manipulator/joint_trajectory`，包含六个旋转关节及两个夹爪直线关节；Gazebo 关节反馈合并到 `/joint_states`，RViz RobotModel 据此运动。使用 Gazebo 原有关节位姿轨迹插件，属于运动学展示，不验证力控、果实接触、采摘或载荷。真机配置仍为原来已验证的底盘算法，不导出机械臂执行指令。
