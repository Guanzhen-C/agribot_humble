# 农机、机具、场景与算法兼容规范

## 范围

本规范针对当前 Unity 配置平台及 ROS 2 仿真，不更改两种真机的运行参数。
机器可读清单：[`compatibility.json`](../src/agribot_farm_sim/config/compatibility.json)。
清单按实际安装、卸载、导出链路整理，不把资源目录里尚未接入的模型算作可用功能。
本轮实际测试结论见 [2026-10-19 验证结果](farm_compatibility_results_20261019.md)。

验收分为：挂载兼容、参数兼容、短程闭环、全路线完成。四者不能相互替代。
某辆车短程通过不代表另一辆车、另一场景、另一算法或同车的全路线通过；旧参数证据也不能直接作为当前通过。
本轮先形成规则与证据，不增加大模型界面或自动执行行为。手动实验选项仍保留；不能把可选项等同于已具备导航推荐资格。

### 正常运行与回归路线分离

2026-10-20起，Unity“启动仿真”和配置型ROS启动入口默认关闭预设路线执行，由用户在RViz官方Navigation 2面板中选取经停位姿并手动开始。Gazebo负责仿真，不负责该Nav2点选界面。旧路线生成逻辑只服务于显式 `run_waypoints:=true` 的自动回归测试；下文“规定点”“全路线”均指该类测试协议，不表示普通启动会自动行驶。切换为手选点不修复或掩盖已有规划/跟踪失败，也不等于所有组合通过。

### 当前仿真采用主机轮廓

用户随后要求农机具不参与规划和碰撞：配置型仿真启动时，临时Nav2参数仅取主机车体尺寸加原缓冲；临时SDF/URDF删除农具及其子链接的碰撞几何，保留显示、安装、惯性、主机轮胎与车体接触，并继续滤除农具自身点云。它不改规范导出或真机参数。带铰接拖车的接地轮也不再提供接触，因此不能把该模式当作拖车支撑/载荷或实际机具碰撞验证。

下文完整机具轮廓的静态安装审查和历史运行结果仍保留作为原协议证据，不代表当前主机轮廓模式通过。当前新增大农机仿真另使用 `nominal_ackermann_navigation.yaml`，只调整规划/控制软代价、前视及搜索网格，保留物理转向半径、限速和主机硬碰撞；原Car0调优及真机配置不变。

## 固定挂载关系

“八种后挂”指凿式犁、圆盘耙、六行开沟机、悬挂式中耕机、林地粉碎机、四行铣土机、大蒜播种机和灌水覆膜机。新增组合的安装、导出与场景运行分别验收，不把配置可选项当作全程已通过。

| 主机 | 底盘 | 允许机具，均可选择裸车 | 当前能力边界 |
| --- | --- | --- | --- |
| Car0 阿克曼小车 | 阿克曼 | 六轴机械臂与夹爪 | 动作演示，不验证采摘或接触力控 |
| Car1 大型驾驶室拖拉机 | 阿克曼 | 八种后挂 | 名义米制尺寸，固定运输 |
| Car2 宽轮驾驶室拖拉机 | 阿克曼 | 八种后挂 | 名义米制尺寸，固定运输 |
| Car3 履带小车 | 差速履带 | 六轴机械臂与夹爪 | 与 Car0 相同的演示边界 |
| Car4 通用轮式拖拉机 | 阿克曼 | 八种后挂 | 名义米制尺寸，固定运输 |
| Car5 紧凑型开式拖拉机 | 阿克曼 | 八种后挂 | 名义米制尺寸，固定运输 |
| Car6 紧凑型拖拉机 | 阿克曼 | 除粉碎机和铣土机外的六种后挂 | 两种重机具需先设计配重；不虚增质量放行 |
| Car7 带驾驶室拖拉机 | 阿克曼 | 八种后挂、双轴拖车 | 八种固定运输；拖车有被动轮和铰接 |
| Car8 骑乘式割草机 | 差速与被动脚轮 | 前置割草刀杆 | 固定运输，不模拟刀片割草 |
| Car9 收割机主机 | 名义差速履带 | 谷物割台 | 全尺寸名义仿真，不模拟真实后轮转向与收割 |

共 10 辆主机、12 种机具、51 条非空挂载关系，加上裸车，共 **61 种配置**。其中固定后挂 46 种。机具不使用 Car 命名。
当前一车一次装一个机具。装备面板没有喷洒组件；历史生成器支持不等于当前可安装运行。
跨车型设计矩阵为 [`rear_mount_matrix.json`](../src/agribot_farm_sim/config/rear_mount_matrix.json)，保留两项需要设计配重的搭配。`tractor_cultivator` 的旧同名历史导出已备份，再按当前挂点重新生成；旧结果不能作为新模型通过证据。

Unity `CompatibilityContractTests` 逐项调用实际限制函数，核对 10 辆车 × 12 种机具的允许/禁止状态，并核对 57 个农机资源预设与五组算法选项，防止清单与 UI 不一致。装备面板直接根据已安装的预设清单判断可用性，不再维护另一套按车名硬编码的后挂限制。新增机具预览按需加载，卸载后释放生成的网格；场景导出仍为本车和所选机具的完整配置。

所有固定后挂均核对主机/机具真实 CAD 挂点、离地和名义静态轴荷；新增配置还检查机具及连杆对轮胎圆柱/前轮转向保守包络的间隙。几何检查不是实际举升能力或动态稳定认证，必须继续进行精确组合的运行验收。

### 原缩比拖拉机的尺寸恢复

以下为用户授权、根据模型外形选取的名义仿真尺寸，不是厂家参数或实测值。车库不再通过缩小车辆来容纳大车；车库范围及相机取景随车调整，车辆根节点保持单位缩放。

| 车型 | 车体长 / m | 车体宽 / m | 名义主机质量 / kg |
| --- | ---: | ---: | ---: |
| Car1 | 4.80 | 2.557 | 6000 |
| Car2 | 5.20 | 2.930 | 7000 |
| Car4 | 3.60 | 2.110 | 2500 |
| Car5 | 3.40 | 1.536 | 1400 |

同时更新轮距、轴距、轮径、轮心、惯量、导航外廓和传感器安装位。传感器本体保持已有米制尺寸，不随原车模型放大。原始 Unity 导出资产保留，归一化输入和安装版本另存。旧缩比配置的运行结果不能作为新尺寸通过证据。

## 场景规则

9 个场景为：苹果园、玉米试验田、温室行间、农场运输区、番茄温室、草莓种植区、葡萄园、蔬菜试验田、宽幅农机地头运输区。
清单中的 `sceneGroups` 是**候选测试范围**，不是全程通过的白名单。

- 小车优先行间巡检、传感器和视觉展示。
- 紧凑拖拉机优先露天田间、运输区及足够宽的通道，装机具后必须使用组合外廓。
- 大拖拉机不能直接使用小车的急弯和窄行间路径。
- 长拖车仅考虑开阔区低速前进缓弯；当前没有铰接状态空间规划，不推荐窄行间或倒车。
- 收割机主机长约 11.33 m、割台宽约 12.73 m，不能借用缩比模型的旧通过记录。
- 带宽幅割台组合已在新增地头运输区完成全程：平整地面、6 个规定点、约 67 m 路线及 12 m 半径转弯，使用 NavSat + Smac Hybrid + RPP。不改动原有场景路线，也不将本次通过外推至窄行间、其他算法或实际收割作业。

静态检查复用正式 `footprint()`，包括缓冲、机具和拖车保守扫掠包络，检查出生点和全部原规定点。
结果分为 `POSES_CLEAR`、`UNKNOWN_SPACE_ONLY` 和 `MISSION_REVIEW_REQUIRED`，保留每个未知、碰撞或越界点，不把未知涂成自由栅格。
离散位姿无碰撞仍不代表连接路径、曲率、地形或高度可行。未知栅格单独列为复核，不冒充真实障碍。

`UNKNOWN_SPACE_ONLY` 仅表示没有检查到已知障碍或越界，而原地图含未知像素。审查还读取每个实际生成的导航配置中的 `GridBased.allow_unknown`。只有所选定位/感知/控制/规划配置明确允许未知通行时，才将此类原路线放入运行测试计划，并附 `routeWarnings`，继续由实时障碍与完整路线检查验收；不改现有规划器参数。其他配置的许可、字符串 `"true"`、已知碰撞或越界均不能借此放行。苹果园Car0/Car3裸车与带臂组合的原23点因此保留，不能因为末点的未知像素静默排除。

初步检查已发现：Car7 开沟机与番茄温室原路线存在外廓冲突；蔬菜场景部分规定点不适合大拖拉机；收割机、割台和拖车不能照跑小车的玉米行间全路线。
应另选适合的运输路线，不能缩小轮廓、删去原规定点或直接写成通过。

## 算法适配

| 模块 | 当前选项 | 限制 |
| --- | --- | --- |
| 几何感知 | STVL、Nav2 VoxelLayer | 检查高度带、自车剔除、障碍清除 |
| 定位 | FAST-LIVO2+RTK、FAST-LIO2、NavSat ESKF、KISS-ICP | 使用各自需要的相机、点云、IMU、RTK输入；模拟室内 GNSS 不代表真实室内可用 |
| 规划 | Smac Hybrid、NavFn、Theta*、Smac2D、规定点直连 | 默认 Smac Hybrid；二维规划不保证阿克曼曲率；直连不保证避障 |
| 控制 | MPPI、RPP、DWB | 根据底盘应用阿克曼或差速运动学，并继承本车外廓和速度 |
| 独立视觉 | 关闭、目标检测、实例分割、人体姿态估计 | 只输出标注图及结构化结果，不参与导航闭环 |

这是 480 个**选项组合**，不是 480 个独立算法。
阿克曼 MPPI 使用 Ackermann，RPP 禁止原地转向，DWB 使用阿克曼轨迹生成器。
差速/履带 MPPI 使用 DiffDrive、DWB 使用差速生成器，RPP 根据倒车开关选择朝向旋转模式。
差速车的二维规划保留用户指定的终点朝向，不用最后一个毫米级栅格连线推导朝向；目标位置和航向容差不放宽。阿克曼及默认 Smac 参数不变。
不同车型不可套用 Car0 的外廓、速度和转弯参数。视觉推理正常也不等于农作物识别精度通过。

名义米制车辆的 RPP 前视距离随组合车长设置；收割机另有显式的名义履带接触参数，包含横向摩擦、横向力相关滑移和轮速目标设置。它们只用于本轮仿真，不改变原小车接触参数或真机标定。接触参数不是实测轮胎/土壤模型，不能用于宣称真实牵引性能。
### 按场景、主机和机具选择算法

一般场景优先使用真机已验证的 **STVL + FAST-LIVO2/RTK + Smac Hybrid-A* + MPPI**；MPPI 运动模型继续按车辆配置区分 Ackermann 与 DiffDrive。不因为装了某个机具就统一改用 RPP。

小型、紧凑型农机先测试默认的 FAST-LIVO2/RTK，包括开阔场景，不统一切为 NavSat。大型拖拉机和收割机在 `arable_field`、`wide_headland` 中，在 GNSS 位置、IMU、初始航向与地理基准有效时，优先测试 **STVL + NavSat ESKF + Smac Hybrid + MPPI**。这是按车型分组的测试配方，不是全程通过保证；GNSS 条件不满足时不能直接采用。已有精确组合的例外单独保留；默认定位和 NavSat 的运行记录不能互相代替。

选择顺序为：确认挂载关系 → 检查组合外廓与路线空间 → 检查传感器条件 → 优先测试精确组合规则、场景规则或真机默认链路 → 有具体失败证据时仅替换必要模块 → 对精确组合重新完成原路线。更换算法不能解决机具物理上过不去或转弯空间不足的问题，不通过删点、缩小轮廓或放宽验收标准取得通过。

| 精确组合与场景 | 当前测试配方 | 依据及条件 |
| --- | --- | --- |
| 无单独场景规则的组合 | 真机默认链路 | 待该组合完成独立验证，不自动标记通过 |
| 小型、紧凑型，无精确组合覆盖 | STVL + FAST-LIVO2/RTK + Smac Hybrid + MPPI | 先验证默认链路；失败后单独测试替代定位 |
| 大型拖拉机、收割机，玉米试验田或宽地头，无精确组合覆盖 | STVL + NavSat + Smac Hybrid + MPPI | GNSS位置、IMU、初始航向和地理基准有效；各组合仍须独立验证 |
| Car4 + 圆盘耙，宽地头运输区 | STVL + NavSat + Smac Hybrid + MPPI | 默认融合定位曾严重偏离；仅换定位后完成原 6 点全路线。要求 GNSS、IMU 和初始航向有效 |
| Car7 + 六行开沟机，宽地头运输区 | STVL + NavSat + Smac Hybrid + MPPI | 默认融合定位偏离、RTK 因子被拒绝；仅换定位后完成原 6 点及转弯。须容纳 5.51 m 宽机具，GNSS/IMU/初始航向有效 |
| Car9 + 谷物割台，宽地头运输区 | STVL + NavSat + Smac Hybrid + RPP | 当前具有该全尺寸组合的全路线通过证据；不声称其他算法必然不可用。要求 GNSS 有效且空间容纳 12.73 m 外宽 |

场景级配方写在清单顶层的 `sceneAlgorithms.<场景ID>`；可用 `vehicleSceneGroups` 限定车型分组。更具体的例外写在挂载关系的同名字段，优先级更高。两者均包含 `algorithm`、`reason` 和 `conditions`，不再使用会影响所有场景的 `testAlgorithm`。例如 Car4 圆盘耙回到番茄温室，仍优先采用默认融合与 MPPI。Car4 凿式犁和 Car5 圆盘耙已补测默认 MPPI 并通过，不再为两者设置全场景 RPP 覆盖。

本轮导航审查与公共启动入口保持一致：使用主机外廓加原缓冲，农机具不参与导航碰撞与接触碰撞；地面边界仍保守检查整车组合。终点位置容差为前后轴距一半，不要求终点朝向；中间点要求不变。这不验证机具防撞或实际作业安全。

`select`、挂载短程计划和全路线计划使用同一个选择函数。输出是带使用条件的**离线测试配方**，不是全程验收证明，不是运行中的自动故障切换，也不修改 Unity 用户手动选择或真机启动命令。若条件不满足，必须停止采用该配方并单独评估，不能把模拟室内 GNSS 当作真实室内可用证据。独立视觉识别仍是可选展示，不参与运动闭环。

### 从实际证据选择推荐

`select` 增加可重复的 `--evidence-root` 参数。指定运行记录目录后，会重新计算输入指纹、检查完整原路线、定位误差和进程退出，再对精确的“场景＋农机＋农机具”选择：

1. 精确组合规则优先于场景规则，场景规则优先于真机默认；仅在已有有效全程证据的候选之间排序。没有场景覆盖时仍优先真机默认，不因另一算法的毫米级指标更小就替换。
2. 优先配方没有可用证据时，再考虑已有通过证据的真机默认及模块变更最少的替代组合。不把尚未测试的 NavSat 配方直接标成通过。
3. 同一算法在当前输入下有任一次全程失败、定位超限或异常退出，则不能靠另一次偶然通过入选。另一场景、另一机具、短程或已变更配置的历史通过均不能借用。
4. 没有合格组合时，`algorithm` 为 `null`，而不是把未验证配方当成推荐。保留 `testRecipe` 供下一轮试验使用。

状态 `VALIDATED` 仅表示所记录的该条仿真路线通过；`PENDING_VALIDATION` 表示缺少有效全程证据，`REVIEW_REQUIRED` 表示有失败而无可推荐替代，`MOUNTING_REVIEW_REQUIRED` 表示挂载/作业策略尚未获准。这里的“推荐”不是所有算法的全局最优证明，也不验证入土、收割、播种或真实牵引能力。无论选择什么算法，均保留本组合完整外廓、运动学和限速。

`report` 同步生成全部 61 种配置 × 9 个场景的 `algorithmSelections`，区分推荐与待验证状态。当前 Car5＋粉碎机的草莓区原 5 点路线，MPPI、RPP 和 DWB 都在掉头处未完成，故不能推荐；不能由番茄区直行通过推导草莓区掉头通过。

Smac Hybrid 的运动学与全外廓碰撞检查，以及 MPPI/RPP 的适用范围，参考 [Nav2 规划与控制插件说明](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/navigation_plugins/setup_navigation_plugins/)；本文实际运行使用项目已适配的 Humble 接口，不照搬新版插件参数。

## 验证命令

```bash
source /opt/ros/humble/setup.bash
source /home/cgz/agribot_ws/ros2_ws/install/setup.bash
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority

# 查询精确组合的测试算法、原因和使用条件，不启动仿真或真车
ros2 run agribot_farm_sim farm_compatibility select \
  --car Car4 --implement mounted_disc_harrow --scenario wide_headland

# 按当前实际全程记录选择；可以重复指定 --evidence-root 汇集多个批次
# 不启动仿真或真车，不覆盖 Unity 中的人工算法选项
ros2 run agribot_farm_sim farm_compatibility select \
  --car Car5 --implement forest_chain_crusher --scenario strawberry_field \
  --evidence-root "$HOME/.local/state/agribot-algorithm-validation/20261020_cross_mounts" \
  --evidence-root "$HOME/.local/state/agribot-algorithm-validation/20261020_scene_best"

# 静态审查：61 种配置，每种检查 48 个正式导航参数分支和 9 个场景
ros2 run agribot_farm_sim farm_compatibility audit \
  --output "$HOME/.local/state/agribot-algorithm-validation/compatibility_audit_$(date +%Y%m%d_%H%M%S)"

# 本轮米制验收：10 辆车各安装一个机具，串行运行各自已选定场景的全部规定点
ros2 run agribot_farm_sim farm_compatibility run \
  --plan "$(ros2 pkg prefix --share agribot_farm_sim)/config/metric_transport_acceptance.json" \
  --output "$HOME/.local/state/agribot-algorithm-validation/metric_transport_$(date +%Y%m%d_%H%M%S)"

# 挂载、两类底盘算法分支、场景覆盖，均为短程而非全程
export COMPAT_RUN=$(date +%Y%m%d_%H%M%S)
export COMPAT_PLAN="$HOME/.cache/agribot/compatibility_plan_$COMPAT_RUN.json"
ros2 run agribot_farm_sim farm_compatibility plan --suite baseline \
  --output "$COMPAT_PLAN"
ros2 run agribot_farm_sim farm_compatibility run \
  --plan "$COMPAT_PLAN" \
  --gui --rviz \
  --output "$HOME/.local/state/agribot-algorithm-validation/compatibility_runtime_$COMPAT_RUN"

# 指定配置的正式全路线验收，不能由短程替代
ros2 run agribot_vehicle_description verify_algorithm_runtime.py \
  --vehicle-id ackermann_current --scenario tomato_greenhouse \
  --case stvl_off_fastlivo_rtk_smac_hybrid_mppi \
  --manipulator false --arm-demo false \
  --scope full-route --motion-timeout 2400 \
  --output "$HOME/.local/state/agribot-algorithm-validation/compatibility_full_$(date +%Y%m%d_%H%M%S)"

# 汇总历史和当前证据，检查记录中的文件指纹是否仍与当前一致
ros2 run agribot_farm_sim farm_compatibility report \
  --evidence-root "$HOME/.local/state/agribot-algorithm-validation" \
  --output "$HOME/.local/state/agribot-algorithm-validation/compatibility_report_$(date +%Y%m%d_%H%M%S)"
```

`plan --suite mountings`、`components`、`scenes` 可拆开检查；`matrix` 生成候选场景内完整组合测试计划，数量很大，不默认启动。
`plan --suite routes --audit <audit.json> --scenario <场景ID>` 仅为静态位姿检查清晰且在候选范围内的组合生成原路线全程计划；其余组合在计划的 `excluded` 中列出，不能静默略过后称全部通过。
`plan --suite all-routes --audit <audit.json> --output <计划.json>` 遍历全部 9 个场景，不再按历史车型分组排除场景。配置检查和规定点外廓检查仍必须通过；`excluded` 中记录具体场景、冲突点和待复核状态。规定点冲突不等于整张场景无法通行，需另行核实通道、车高、转弯空间及合适路线，不能静默删点或缩小车体来放行。
运行器串行、不创建真实底盘节点，隔离 ROS Domain 与 Gazebo 端口，只清理自己的进程，清理异常就停止批次。
`run --gui --rviz` 为每项测试打开 Gazebo 和 RViz；不带选项时保持原无显示行为。全程计划还会核验返回记录的 `scope: full-route` 和 `full_route: PASS`，不能用短程结果冒充全程。显示选项不改变车辆、机具、路线、算法参数或质量限值。
新静态审查记录出生点至全部原规定点的折线距离下界；新全程计划按`max(2400, ceil(距离/限速×2.5+180))`生成`motionTimeoutS`观察预算，上限4小时，超出需显式执行复核。旧计划没有此字段时仍为2400秒。预算只延长观察时间，不改变仿真速度、路线、避障、误差限值或通过条件；卡住不能因为预算更长就算通过。
旧结果不覆盖；失败保留，重试用新目录。裸车明确关闭机械臂，避免上次导出残留；带臂组合检查 8 个关节反馈，行驶时不做动作演示。
机械臂本身为无接触力学的运动学展示，行驶通过不代表已验证机械臂全扫掠避障或真实载荷能力。
报告为 `evidence.json` 和 `report.md`，区分短程/全程与输入已变化的历史记录，不隐藏失败。
本次推荐审查另加定位质量核验：水平 RMSE 不超过 0.30 m、最大水平误差不超过 0.75 m。
这只是仿真证据筛选限值，不修改任何正式算法参数、真机标定或既有运行记录；完成路线但定位发散也不能进入推荐候选。
相同当前输入的全程复测只要仍有一次失败或质量不达标，就不能挑一次成功当作稳定通过；问题修复并重新验证前不进入推荐候选。

## 后续大模型推荐

本轮先验证，不接模型、不增加自动启动行为。
大模型只能返回清单中的主机、机具、场景和各模块 ID，服务端再次做确定性兼容与证据校验。
只有输入指纹仍一致、确实完成全程、场景 ID 有效且使用运动学规划的记录，才有仿真推荐候选资格。初始车型尺寸分组只记录为场景提示，不否决已经完成的精确组合；不能按车型名称永久排除场景。
未验证、失败、旧参数、未实现农艺过程应明确提示，不凭模型常识放行；仿真候选也不等于真机运行许可。
