# 差速车 FAST-LIVO2/RTK MPPI 仿真验证

该包只承载差速车仿真验证，不包含真机驱动。定位链与真机保持一致：

- Gazebo C16、IMU、RGB 相机和模拟固定解 RTK；
- FAST-LIVO2 局部里程计；
- `agribot_hardware_bringup` 中的固定滞后因子图融合；
- Smac State Lattice 差速运动基元；
- Nav2 MPPI DiffDrive 控制器。

Nav2 参数不读取真机配置。`nav2_params_scout_orchard_sim.yaml` 按 Scout
仿真模型、0.05 m 果园地图和 Jetson 上的仿真实时率单独配置；差速运动基元是
Nav2 Humble 提供的 0.05 m 分辨率、0.5 m 转弯半径官方样例副本。

交互验证：

```bash
source /opt/ros/humble/setup.bash
source ~/agribot_ws/ros2_ws/install/setup.bash
ros2 launch agribot_differential_fastlivo_mppi \
  differential_mppi_fastlivo_rtk_sim.launch.py
```

无界面自动闭环验收：

```bash
ros2 launch agribot_differential_fastlivo_mppi \
  differential_mppi_fastlivo_rtk_sim.launch.py \
  gui:=false rviz:=false auto_run:=true
cat /tmp/differential_fastlivo_rtk_sim_report.json
```

自动验收会检查定位融合就绪、State Lattice 全局路径、MPPI 线速度和角速度、
车辆实际移动、终点误差以及融合定位相对 Gazebo 真值的 RMSE。
默认从 `(2.0, 35.0)` 行驶至 `(35.0, 35.5)`，在果园场景中完成约 33 m
的长距离闭环。可通过 `test_goal_x`、`test_goal_y` 和 `test_goal_yaw`
覆盖自动测试终点。
