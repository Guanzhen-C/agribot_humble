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

## 导入3DGS场景中的真机保存路径回归

新增独立入口 `scanned_saved_route_validation.launch.py`，不改变上面的默认仿真。
它使用 Unity 导出的 `unity_car3` 履带车，读取真机保存的完整104个锚点及密集
路径，不重新规划全局路线。跟踪控制器和局部Smac绕行复用 `SavedPathRunner`。
本入口读取真机差速导航YAML的副本，替换仿真时钟、里程计/点云话题及
运动基元路径，并叠加 `scanned_saved_route_nav2_overlay.yaml` 的仿真控制器参数。
终点位置容差按Car3前后轴距一半设置；导航轮廓按实际Unity车体外缘加5厘米
仿真余量重新生成，不照搬另一台车的对称矩形。可用 `--simulation-clearance`
修改本次离线仿真的余量，真实CAD/物理碰撞体和原始真机文件不修改。
其他默认入口仍使用各自的仿真参数。

### 准备

需要已导入的LCC2场景目录、带地理参考的当前导航地图，以及
`地图基名_saved_paths.json` 中的 `manual104_saved` 路线。必须保留原场景的
`source/*.lcc2` 地理坐标元数据，不能凭场景名称猜测坐标方向。

```bash
source /opt/ros/humble/setup.bash
source ~/agribot_ws/ros2_ws/install/setup.bash

ros2 run agribot_differential_fastlivo_mppi prepare_scanned_route_validation.py \
  /path/to/imported/scene.json /path/to/current_map_base \
  ~/agribot_ws/ros2_ws/src/agribot_hardware_bringup/differential/config/nav2_params_differential_fastlivo_mapped.yaml \
  /path/to/new_validation_bundle \
  --nav2-overlay "$(ros2 pkg prefix agribot_differential_fastlivo_mppi)/share/agribot_differential_fastlivo_mppi/config/scanned_saved_route_rpp_overlay.yaml"

ros2 run agribot_differential_fastlivo_mppi add_scanned_ground_support.py \
  /path/to/new_validation_bundle

ros2 run agribot_differential_fastlivo_mppi prepare_scanned_contact_proxy.py \
  /path/to/new_validation_bundle --height-limit 0.95
```

上述命令选择项目已有RPP控制器用于完整密集路线跟踪，保留碰撞检测和局部
Smac恢复。省略 `--nav2-overlay` 则使用独立的MPPI对照参数，两者结果不能混称。
准备器校验地图与原保存路线的指纹，将扫描UTM局部坐标对齐到地图ENU坐标，
输出旋转和残差。原高斯、三角网格、地图和104点文件不覆盖。地面支撑器以
密集PCD中已验证可通行区域的低位地形补网格缺面，保留抬高的道路和坡度。
派生物理地面使用ODE高度场，不使用三角网格边缘作为履带接触面，避免内部
三角形边造成异常侧向冲击。原连续STL仍保留供地形查询和检查；高度图采用
Gazebo 11实际支持的8位编码，量化误差上界记录在准备报告中。
自由像素使用地图YAML的 `free_thresh` 和 `negate`，不使用固定白色灰度门限，
因此GIMP清洗产生的近白色像素不会被误判。未知低位地面的策略同步读取
规划器 `allow_unknown`；较高结构仍保留，未知值本身不会额外制造硬障碍。
上述Car3验证命令使用0.95米自由区通行包络，覆盖该车约0.88米的实体车体高度
并留出小余量。准备器默认仍为更保守的0.45米；不要将0.95米用于未确认可通行
的区域或其他车型。接触代理把完全位于手工确认的自由栅格内、低于所选包络的扫描面
交给连续地面处理，避免草丛和重建凸起顶住底盘。映射障碍物及其10厘米边界、
不允许通行的未知区和更高结构仍保留原接触三角面；完整原网格继续用于雷达射线，原高斯
外观不变。跨自由区/障碍区或跨高度门限的面先自适应细分到约10厘米，
避免因大三角形的包围框碰到远处障碍，就保留整张自由区伪碰撞面。
这是基于人工清洗地图的可通行硬地近似，不是可变形土壤、涉水、
作物力学或真实越障能力保证，不能拿它删除未确认可通行的障碍物。
本次地面与履带接触刚度均为100,000N/m，阻尼1,000Ns/m，物理步长1毫秒。
不采用曾导致IMU/估计异常的全局0.1m/s接触修正限速，仍使用ODE默认100m/s。
传感器射线仅查询完整原扫描网格，派生高度场和硬接触代理不重复参与射线计算；
履带插件自身的机器人、履带和左右侧碰撞标志保留不变。

### 运行及验收

```bash
export ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11393
export GAZEBO_IP=127.0.0.1
ros2 launch agribot_differential_fastlivo_mppi scanned_saved_route_validation.launch.py \
  bundle:=/path/to/new_validation_bundle
```

完整使用原始高斯外观。此独立入口为Car3增加按真实导轮位置构造的连续履带
接触体，复用[Gazebo官方SimpleTrackedVehiclePlugin](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/plugins/SimpleTrackedVehiclePlugin.cc)
的ODE接触/摩擦驱动。ROS适配器只传递速度和进行两侧履带限幅，不设置车体
位姿或车体速度状态；同时限制指令加减速为0.5m/s²、角加减速为0.8rad/s²，
制动及切换旋转指令也经过同一限制。原CAD、传感器安装和场景障碍物保留。车体的单整盒
碰撞代理按实际CAD三角面修正为上车体及两个下侧梁，保留真实低位结构，
不再把车头/腹部的实际空隙填成实体。
原四个轮子的碰撞代理关闭，但转动和履带外观动画保留。普通默认场景不启用
这个可选接触体。模拟GNSS采用WGS84椭球，送入
实际KF-GINS节点；导航读取估计位姿和真实Nav2局部代价地图。独立P3D真值话题
`/validation/physics_truth` 仅用于生成模拟GNSS和验收，不直接反馈导航、发布TF
或设置车辆位置。没有全程测试时限，停车绕行仍按生产代码执行。

RViz显示当前真机二维地图、原完整路径、104点编号及测得的物理轨迹。
输出目录中的 `live_status.json` 为进度，`actual_trajectory.csv` 为物理/估计
轨迹和控制量，`result.json` 为最终结果。只有跟踪成功、104点按路径顺序在
0.75米范围内经过、终点距离不超过0.5米、实际停止且源文件指纹未变化才判通过。
这不等同于真实GNSS精度、FAST-LIVO2精度或所有场景可通行的实车验收。

全程完成后，在相同ROS域中可恢复完整结果显示，不重新启动导航：

```bash
ros2 run agribot_differential_fastlivo_mppi display_scanned_route_result.py \
  /path/to/new_validation_bundle
```

该只读节点要求报告为完整通过，仅发布原计划、全部实际轨迹及104点编号，
不发布控制命令或TF，不创建导航动作。

停止时向本次 `ros2 launch` 发Ctrl+C；不要使用全局 `pkill`，以免停止其他
worktree的仿真。不要在同一ROS域中同时运行真机导航和本测试。

故障复现可加 `diagnostic_start_anchor:=7 diagnostic_end_anchor:=12`，从原第7
点的物理地面重新出生，只跟踪该小段，结果写入 `diagnostics/`。此模式始终
标记 `diagnostic_only`，不能被计作104点全程通过。每次停车绕行会保存当时
的局部栅格和未来1米的轮廓检查结果，供区分实际阻塞和控制器停滞。
仅测试物理驱动时再加 `run_route:=false`，此时不会自动发送导航任务。
