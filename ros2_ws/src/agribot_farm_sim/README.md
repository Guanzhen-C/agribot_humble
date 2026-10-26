# 农机场景资源

本包为 Unity 导出车辆提供 Gazebo Classic 场景、同源二维占据地图、可选回归路线和场景级 RTK 地理基准。仅用于仿真，不改变真机安装参数。

## 统一 AI 助手

Unity Linux App 只保留右下角一个 AI 助手入口，问答和可点击方案在同一会话中。DeepSeek Flash 根据自然语言、当前界面状态、按时间排列的操作和对话、会话备忘选择方案；本地程序解析车辆配置并校验算法。点击“选择这套装配”会安装四类传感器和选定机具、选择场景及算法；只改算法的方案保留当前装配。仍需用户另行点击“启动仿真”，不会自动派发经停点或驱动车辆。

助手跨页面和场景加载保留同一会话，关闭窗口不取消后台回答。输入和选车、装卸、场景/算法切换、修理参数、启动/停止、导出结果等写入本机 `Application.persistentDataPath/assistant_sessions/*.jsonl`，不进入项目和Git。界面操作仅作为AI上下文，不显示在聊天区；聊天区只保留用户和助手对话，无对话时显示问候。请求使用最近40条消息与操作、当前状态、上一轮备忘；完整本地记录保留，模型备忘用于延续较早的偏好与问题。“新对话”清空当前聊天与备忘但保留界面选择。退出App后不自动恢复上次会话，也不会自动读出所有本地历史；日志不是远程知识库。

系统提示词位于 `config/assistant_system_prompt.txt`；结构化知识每次从 `compatibility.json`、场景和当前导出配置生成，附上 `manual_confirmations*.json` 中与当前版本一致的运行依据。完整10车型清单与推荐组合分开，当前有Car3和Car9两种履带车型。每个车—机具—场景均关联默认算法、条件和允许定位。可在统一接口已提供的组件范围推荐替代组合，不将“接口可选”冒充“全程成功”。用户反馈参与连续对话，不自动重写适配目录或确认记录。

场景分为内置默认场景和用户导入场景，不固定把场景总数说成9个。助手只有一个只读 Function Call：`list_scenes({})`，通过 DeepSeek 原生 `tools`/`tool_calls`/`tool` 消息查询本次实际目录，返回两类场景、各类数量、总数及导入场景的可推荐配置。目录读取 `~/.local/share/agribot/scanned_scenes/catalog.json`，与 App 使用同一 `AGRIBOT_SCANNED_SCENE_ROOT` 覆盖项。不接收路径参数，不开放导入、启动、运动控制或文件执行工具；每轮至多查询一次。

场景查询结果按会话版本复用：首次查询后，App上下文保存后端返回的目录版本标记，后续请求由本地重新核对该标记。未变化时将有效场景摘要直接提供给模型并禁用重复工具调用，通常只需一次模型请求；更换车辆、机具或算法不使目录失效。导入、删除、场景配置文件变化、新对话或App重启后重新查询。只复用目录知识，不跳过当前适配关系和应用前的本地版本复核，也不把模型备忘中的旧场景名当作有效缓存。响应中的`sceneCatalogReused`和`modelRequestCount`仅用于本地诊断。

成功导入后，场景名称和ID作为已发生界面事件加入上下文；最近导入场景与当前选中场景分别记录，即使后续操作使旧事件超出最近40条，仍可区分“刚导入的那个”和当前选择。新对话清除最近导入的历史指代，但不删除场景库或改动实际选择。导入场景的可点击方案应用前重新检查场景和车辆配置版本，删除或修改文件会阻止应用旧方案。未知用途可在对话中说明，不新增必填用途输入框；名称不代表场景语义、道路净空或真实GNSS质量。扫描场景不继承内置场景的全程通过记录，也不承诺任意大车可通行。

明确禁用的定位和车—场景组合由本地代码限制，模型不能越过：例如宽地头全部车型、葡萄园Car3/4/6、玉米田Car4均仅允许NavSat。新组合不继承旧通过标签；配置SHA变化仍阻止应用陈旧方案。应用时重新校验当前配置及适配关系，只解析可信本地字段，拒绝虚构组件、重复方案和算法模式中的隐式换车。格式错误至多纠正重试一次，不能因此放宽限制。

没有把相机、点云或实时ROS传感器数据送给助手；它不能判断实际定位是否就绪，也没有启动/停止控制工具。试验文件名、哈希等内部信息不展示。知识和对话以文字发往所配置的在线模型；本机日志不主动上传整库。车库预览仍从当前实际模型生成并在构建时校验指纹。

整个App每次启动均进入全新的车库状态：未选车辆、未安装装备、未选作业场景、未选算法，不恢复上次的界面操作和聊天。当前场景选择仅在本次运行内存中保留，已导入的场景库仍保存在磁盘；场景选择和算法不读写旧PlayerPrefs。装备背包从完整目录初始化，不再读写旧存档。旧存档、已导出的配置和实验数据不删除。手动选择场景并选齐五类算法组件，或应用AI完整方案后才启用启动按钮；部分选择也作为当前界面状态提供给助手，不冒充已选默认方案。同次运行切换页面不清空当前选择。“新对话”仅清除聊天、模型备忘、输入草稿和回复状态，不恢复旧会话，也不擅自装卸装备、改动当前运行方案或停止仿真。持续聊天没有固定轮数上限，但模型使用有界近期上下文与备忘，不保证任意久远细节逐字保留。

接口凭据只在工作站本机读取，不写入 Unity 项目、构建程序或 Git。使用 `DEEPSEEK_API_KEY`，或配置权限为 600 的 `~/.config/agribot/llm.json`：字段为 `api_key`、`base_url`（`https://api.deepseek.com`）和 `model`（`deepseek-flash`）。Unity 通过标准输入把文本交给 `/usr/bin/python3`，不把用户文本作为命令执行。`AGRIBOT_WORKSPACE_SRC` 可覆盖 ROS 源码目录；默认沿用当前同步脚本所在工作区。

离线核对候选不需要凭据或网络：

```bash
cd ~/agribot_ws/ros2_ws
PYTHONPATH=src/agribot_farm_sim:src/agribot_vehicle_config_tools \
  /usr/bin/python3 -m agribot_farm_sim.natural_recommendation \
  --workspace-src "$PWD/src" --catalog-only
```

网络不可用、超时或无匹配时只提示错误/补充问题，不修改当前车辆配置。关闭窗口只隐藏会话；退出App才取消请求，不影响已启动的仿真。原手动安装、算法库和启动命令保持不变。

## Car3 当前轮心配置

2026-10-21按用户要求，`prepare_compact_track_contacts.py`从当前可见的前后导向轮及履带零位姿网格推导接地几何，资源更新为`compact_tracks/ros_v4_real_centers`。前轮x约0.44894m、后轮x约-0.38806m，左y约0.30368m、右y约-0.29751m，z约-0.10694m；轴距约0.83700m，左右轮距约0.60118m。接地半径约0.11815m，包含导向轮外侧履带支撑包络，不再采用旧±0.201m轮心、0.22492m大圆柱。

原base参考、车身网格和传感器位姿不移动。ROS规范配置、SDF/URDF、驱动轮径、动画比例和Unity导出同源更新；最终到点容差随轴距变为约0.41850m，其他导航限速和算法不改。运行时前后横向摩擦统一0.36、纵向1.8；固定ODE横向slip1=0.01曾使农场横坡出现明显侧滑，现改用Gazebo官方WheelSlipPlugin，横向compliance=0.1、纵向compliance=0，按轮速和总质量估计的单轮法向载荷动态设置滑移。停车时不再保留固定滑移，规则对Car3全部场景生效，不改其他车型、场景坡度或导航参数。仍是四接地圆柱近似，坐标来自当前模型，不是实车测量或柔性履带动力学，不保证斜坡转向零位移。下文ros_v3“只改外观、不改接地轮”的叙述及原人工通过记录属于旧版本，不能作为此新几何及新接地参数的全程验收。

`SimulationTrackedWheelSlip`是对官方插件的本地策略适配，不修改其源码：左右履带实测线速度差在0.02～0.08m/s间平滑提高转向滑移，最大为0.003m/s/N，以免抑滑后无法正常原地转向。上述0.1横向compliance用于停车/直行，转向期间按该策略变化。没有直接强制修改车体速度或位姿。

Car9收割机的裸主机和带割台两种仿真配置也启用同一机制，但不照搬Car3数值：按15.1/17.6吨模型总质量及各轮实际半径估计接地载荷，转向滑移仍为1e-6m/s/N；前后横向摩擦按原有载荷平衡比例同时提高2倍（后方0.2，前方裸机约0.15301、带割台约0.05699），纵向1.8不变。这是运行时仿真覆盖，不修改规范导出的几何、质量、驱动、传感器或Nav2参数，也不把原有通过记录当作新接地模型的全路线验收。

## 可通行地面与材质

配置型仿真的全部车型共用地面接触策略，不因草地、泥地、道路等显示材质改变可通行性。启动时按已标注的导航表面拆分碰撞网格：平面地块使用0.5m厚实心支撑，顶面高度和边界不变；农场院落及苹果园的起伏地形保留原三角面，不用平板覆盖坡度。地面使用统一刚性接触和充足摩擦，实际轮胎/履带摩擦仍由车辆自身配置限制。显示纹理不变，作物、树干、墙体、温室框架及高架种植床等障碍碰撞不删除。

旧温室资源没有标注地面，兼容识别其底板、草坪和道路对象；不根据材质名或颜色猜测作物是否可通行。新场景应显式提供 `navigationSurface` 标记。生成的 `geometry.json.groundPolicy` 记录实体支撑数量及保留的地形/障碍面数。此修改不是越障、软土或农具作业模型，也不代表所有车辆在所有场景均已重新完成全程验证。

## 默认由用户选取经停点

Unity 的“启动仿真”、同步脚本和 `configured_ackermann_sim.launch.py` 现在默认 `run_waypoints=false`，不自动派发本包生成的路线，车辆等待用户下发任务。多点选取使用 **RViz 官方 Navigation 2 面板**，不是 Gazebo 场景编辑器的功能：先进入 `Waypoint / Nav Through Poses Mode`，用 `Nav2 Goal` 依次点击并拖出各点朝向，最后点击 `Start Nav Through Poses`。

下文预设路线生成与规定点验收仅用于显式开启的回归测试（`--run-waypoints true` 或 `run_waypoints:=true`）。原苹果园23点及历史路线文件保留，不作为正常启动时的自动任务；任意手选点仍须满足当前车辆/机具外廓、转弯半径及实际场地边界。

后续按用户要求，配置型仿真现只以主机尺寸加缓冲进行导航；农具保留显示和惯性，但临时SDF/URDF不保留其接触碰撞。下文完整机具外廓的审查和旧全程记录属于原协议，不认证当前模式下的机具间隙、拖车接地或真实作业。真机及规范导出文件不受此仿真覆盖影响。

## 组合兼容审查

当前 10 辆农机、12 种机具、61 种含裸车配置及算法/场景适配规则见
[兼容规范](../../docs/farm_compatibility_and_validation.md)，机器可读清单为 `config/compatibility.json`。
使用 `farm_compatibility select/audit/plan/run/report` 分别查询场景算法配方、进行参数审查、生成计划、串行闭环检查与证据汇总。
候选场景、静态检查和短程通过均不等同于全路线通过；旧参数记录不会直接进入后续推荐白名单。

完整路线验收在生产执行器报告原生成功后，只读观察新的仿真真值里程计和零控制指令，要求连续静止至少1秒。收尾阶段按里程计原始频率记录轨迹，并单独检查最终静止位置相对原始末点的误差；中间点0.75m、末点半轴距及已记录的0.03m异步采样裕量不变。普通2Hz日志可能漏掉制动结束位置，不能把其最后一个稀疏样本直接当作停车位置。这项改动只影响验收观测，不发布停止命令，不修改定位、规划或控制算法。未记录这些收尾检查的旧运行不能补写为新版验收通过。

中间点的有序轨迹匹配与最终停车位置分开计算。全部中间点继续采用0.75m范围，最终点只使用原生成功后实际静止位置与半轴距容差；不能把全轨迹极小极大匹配中任意一个并列样本误当作停车位置。旧记录不改写。只在显式提供 `--acceptance-policy` 时，推荐导出器才允许验收源码版本兼容复核：校验归档源码、当前源码、原记录与轨迹哈希，语法树必须仅新增这项离线计算并替换一处验收调用，其他运行输入仍逐个严格匹配。旧通过项须重新核验相同位置门槛，真实失败仍阻断推荐；仅因旧验收误判的失败标为待重跑，不能自动转换为通过。默认不提供该策略时仍采用严格源码指纹匹配。

静态路线审查区分已知碰撞、越界与仅含未知像素。仅当所选实际规划配置原本允许未知通行时，未知像素路线才带 `routeWarnings` 纳入运行测试，继续保留全部规定点与实时避障；未知不是自由或已通过。调参前阶段46种固定后挂均至少完成过一条全路线，但多数只是温室直行。2026-10-20新的阿克曼仿真MPPI参数已用于部分路线验证，仍须逐组合核验；旧参数或旧模型的成绩不能作为新输入的通过依据，其他可通行场景继续逐项验证。

新全程计划根据原路线距离下界和导出限速生成观察预算，旧计划仍默认2400秒。预算不改变车辆、地图或任何验收条件，详见兼容规范。

### 按车辆生成路线导引

`scenarios.route(scene, vehicle)` 与 `write_vehicle_route` 使用当前导出车辆的最小转弯半径；启动、静态审查与运行验收共用同一生成逻辑。Smac 使用当前主机外廓加原缓冲、原最小转弯半径与限速，按用户要求不把农具算入导航碰撞轮廓；不缩小主机或修改场地放行。

蔬菜田按用户要求只绕作物区一圈，不再附加用于规避提前到达判定的末尾小回环；两侧导引弯半径取4.5m与车辆最小转弯半径中的较大值。葡萄园按6m行距将转弯半径向上取整到3m的倍数，大车不强求相邻行回程：Car2中耕机使用6m半径，从y=-3m出发、y=9m回程。草莓区大车暂由Smac搜索较宽的地头转弯，不强制经过3m小圆弧上的辅助点。新路线独立验收，不将旧成绩直接转为新路线通过。果园原23点不变。

此前向外延长直行再强行回相邻行的方案已撤回，避免小行距迫使大车绕额外小圈或接近实体地面边缘。大车宽半圆只保留行间任务点和90度掉头中点，使用5点导引；两个斜向辅助点在Car7原生Smac对照中导致额外完整回环，已取消。小车原7点不变。Car2完整缓冲外廓的361个圆弧采样均在原地面内且未覆盖已知障碍像素；此处只是静态几何验证，不代替新5点Smac路径和闭环运行。

葡萄园按用户要求恢复扩大前的实体地面44×30m及5cm地图48×34m。地图外缘包含导入器2m栅格留白，不是实体地面；大车地头路线仍需核对整车扫掠外廓，不能因地图像素为空就认为地面足够。

生成的葡萄园路线文件附带 `navigationSurface`：真实地面边界、当前整车/农具缓冲轮廓和0.25m边缘余量。连续预规划下发前逐段插值检查完整轮廓，超出实体地面则拒绝派发；没有该字段的既有路线不改变行为。该检查不使用真值参与定位或规划，也不是运行中的定位失效保护。

名义米制拖拉机的前轮关节限位按同一中心转角与轮距换算为内轮最大角，与驱动插件一致；不是增大中心转角或缩小转弯半径。新SDF/URDF改变输入指纹，旧模型通过成绩不自动继承。

自动连续路径使用进度感知的终点检查适配器：经过前置路线点后，才允许 Nav2 终点检查器判断完成。当前所有配置型车辆的终点只按位置、容差为前后轴距一半，不要求终点朝向；中间点要求不变。真机配置不受本覆盖修改。路线导引可生成不等于整车扫掠空间足够，已知碰撞或越界仍留待复核，不强行执行。

该路线检查器通过可靠、瞬态本地的 `/current_goal` 接收生产路线执行器发布的原始终点，将原半轴距容差中心放在用户请求的位置，而不是 Smac 栅格路径最后一个离散位姿。[Humble 控制器](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/src/controller_server.cpp)默认检查路径末端；小型车的容差较小时，两者偏差可能导致提前停车。启用此模式后，缺少有效原始目标、坐标系不匹配或非有限位姿均不报告到达；完整路线进度门槛、原 Nav2 到点检查器和原容差保留，不修改路径、官方源码，不发送额外停止指令。普通手动目标检查器不受影响。

仿真 RPP 使用薄适配器 `PositionOnlyRegulatedPurePursuitController`，跟踪、初始路径朝向对齐、限速及碰撞检查仍调用官方实现。Humble 1.1.20 的[官方 RPP 实现](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp)用前视点距离触发最终朝向旋转；当大车半轴距容差大于前视距离时，会在路线开头持续给零线速度。适配器仅为 RPP 的这一内部朝向分支提供零触发半径，真正的 Nav2 到点检查器和路线进度检查仍使用原半轴距容差，不修改前视距离、车体尺寸、转弯半径、中间点或物理真机配置。原实现与适配器的区别有原生 C++ 回归；修改后各组合仍须重新验收，不能沿用旧指纹成绩。

小型和紧凑型农机先测试真机默认的 STVL + FAST-LIVO2/RTK + Smac Hybrid + MPPI，不因场景空旷就统一替换定位。大型拖拉机和收割机在玉米试验田 `arable_field`、开阔地头 `wide_headland` 优先测试 NavSat + Smac Hybrid + MPPI，前提是 GNSS、IMU、初始航向及地理基准有效。精确“场景 + 主机 + 机具”规则优先级更高，例如全尺寸割台仍用 RPP。默认定位失败后可单独测试 NavSat，两个算法的结果分别记录，不能互相代替验收。每条配方记录原因和条件。`select` 是离线测试配方，不会覆盖任意手写 launch 命令，也不自动授权运行或修改真机参数；具体规则与查询命令见兼容规范。

`select --evidence-root <运行记录目录>` 进一步按当前文件指纹与实际全程结果选择推荐；只在有效证据中优先当前场景配方，没有场景覆盖时仍优先真机默认。无合格证据时返回待验证或需复核，推荐算法为空。`report` 的 `algorithmSelections` 为全部 61×9 对关系记录这种区分，不把计划、直行通过或别的机具成绩当作本组合全场景通过。

2026-10-21 用户人工验证确认：Car4 在玉米试验田改用 NavSat，全部八种可挂载机具组合标记为人工确认通过。机器可读标签见 `config/manual_confirmations.json`，保留原配置 SHA、用户原话和代表性人工试验出处；这不是八次独立自动全程验收，不能改写原预设路线的失败。车型级场景配方只对此车、此场景限定 NavSat，其他小型/紧凑型场景仍优先默认 FAST-LIVO2/RTK；精确机具场景规则优先于车型规则。手写 launch 不被离线选择器强制改写。人工确认标签与下方严格自动路线推荐白名单分开保存，不增加其自动验收计数。

同日，用户确认 Car5 在玉米试验田使用默认 FAST-LIVO2/RTK＋Smac Hybrid＋MPPI 时，全部八种可挂载机具组合均可用。对应人工通过标签在 `config/manual_confirmations_car5_arable.json`，同样保留配置 SHA 与代表试验，不冒充逐机具自动全程验收；本条只确认默认算法可用，不把 Car4 的 NavSat 专用限制传播给 Car5。

用户随后确认 Car6 在同场景使用默认算法的全部六种已开放机具组合可用，记录在 `config/manual_confirmations_car6_arable.json`。不包含尚未开放挂载的重型粉碎机和铣土机，其证据范围与上述人工确认规则相同。

Car7 在玉米试验田使用 NavSat 的九种机具人工确认见 `config/manual_confirmations_car7_arable.json`。包括拖车的分组人工标签，但不改写其历史被动运动失败；拖车使用独立16m转弯半径，仍需独立动力学复核，不将凿式犁代表试验当作拖车自动全程成绩。

Car8 在玉米试验田使用默认融合算法的人工确认见 `config/manual_confirmations_car8_arable.json`；当前已开放机具仅前置割草刀杆，不包括裸车或未来新增机具。

Car9 在玉米试验田使用 NavSat、Smac Hybrid、MPPI 的人工确认见 `config/manual_confirmations_car9_arable.json`；当前全部可挂载机具为谷物收割割台。记录对应完整轮胎和履带外观修复后的人工试验，不增加自动原路线验收计数。

Car3、Car0 在当前玉米试验田、当前默认FAST-LIVO2/RTK、Smac Hybrid、MPPI算法下的全部已开放适配机具，分别由用户人工确认通过，见 `config/manual_confirmations_car3_arable.json` 和 `config/manual_confirmations_car0_arable.json`；控制模型分别为DiffDrive和Ackermann，代表试验为0016、0017。当前机具均仅六轴机械臂与夹爪。此前Car3“全部9场景”的记录已按用户纠正撤销，不再是有效标签；不扩展到裸车、其他场景、其他算法或未来新增机具，不改写历史自动路线成绩。

Car0 搭载六轴机械臂与夹爪，在农场运输区使用默认算法的人工选点试验0026由用户确认通过，见 `config/manual_confirmations_car0_farmyard.json`。仅确认该组合，不扩展到其他车或算法，也不增加自动全路线验收次数。

Car3 搭载六轴机械臂与夹爪，在同一农场运输区使用默认算法及改进后的自适应接地策略，试验0031由用户确认通过，见 `config/manual_confirmations_car3_farmyard.json`。仅确认当前轮心和接地策略下的该组合；0030侧滑失败保留，不改写为通过，也不外推其他场景或真实履带动力学。

用户随后明确Car3当前全部适配机具在本场景通过，补充事件为 `config/manual_confirmations_car3_farmyard_all_implements.json`，取代上一条的有效范围但保留其历史。目前机具仍仅机械臂与夹爪，共一个非裸车组合，不重复计数，也不自动覆盖未来新增机具。

Car1 搭载凿式犁，在农场运输区使用默认算法的试验0027随后由用户补充确认通过，见 `config/manual_confirmations_car1_farmyard.json`。仅确认这一个机具组合，不外推其他挂载；关闭时尚无结论的历史与后续人工确认分别保留。

用户进一步明确Car2全部已适配机具在农场运输区、当前默认算法下通过，并补充Car1同样适用。当前有效分组标签见 `config/manual_confirmations_car1_farmyard_all_implements.json` 和 `config/manual_confirmations_car2_farmyard.json`，各八个机具组合，代表试验分别0027、0028。Car1新事件扩展并取代上一条单凿式犁事件的有效范围，旧事件保留但不重复计数；其余机具未逐个重跑，不视为独立自动全程通过，也不外推其他场景或算法。

Car4当前八种已开放机具在农场运输区、默认FAST-LIVO2/RTK＋Smac Hybrid＋MPPI下由用户分组确认通过，见 `config/manual_confirmations_car4_farmyard.json`，代表试验0032。其余机具不再逐个重跑，不增加自动全程验收次数；此前Car4在玉米试验田的NavSat专用规则不变，仅本场景确认默认融合算法可用。

Car5同场景、默认算法下的八种已开放机具也由用户分组确认通过，见 `config/manual_confirmations_car5_farmyard.json`，代表试验0033。只记录当前范围的人工结论，不将其他机具视为逐个重跑或自动全程验收通过。

Car6在农场运输区、默认算法下的六种已开放机具由用户分组确认通过，见 `config/manual_confirmations_car6_farmyard.json`，代表试验0034。未开放的重型林地粉碎机、四行铣土机不在通过范围内；同样不增加自动全程验收计数。

Car8受控前轮阿克曼版本搭载前置割草刀杆，在农场运输区、默认融合定位/Smac Hybrid/MPPI下由用户确认通过，见 `config/manual_confirmations_car8_farmyard.json`，代表试验0037。仅覆盖当前已开放的这一机具组合，不包含裸车、其他场景或旧差速脚轮结构，也不增加自动全程验收次数。

Car9在当前农场运输区由用户判定不适用，割台组合试验0038已关闭，记录为 `USER_SCENE_NOT_APPLICABLE`，不标通过或算法失败。后续本场景人工验证不再安排Car9；其他场景记录不变。

Car0搭载机械臂在番茄温室 `tomato_greenhouse`、默认融合定位/Smac Hybrid/MPPI下的人工选点试验0039由用户确认通过，见 `config/manual_confirmations_car0_tomato_greenhouse.json`。仅覆盖该组合，与 `greenhouse` 场景分别记录，不增加自动全程验收次数。

Car1搭载凿式犁在同一番茄温室、默认算法下的人工选点试验0040由用户确认通过，见 `config/manual_confirmations_car1_tomato_greenhouse.json`。仅覆盖本次机具组合，不外推其他挂载、场景或算法，不增加自动全程验收次数。

Car2搭载凿式犁在同一番茄温室、默认算法下的人工选点试验0041由用户确认通过，见 `config/manual_confirmations_car2_tomato_greenhouse.json`。仅覆盖本次机具组合，不外推其他挂载、场景或算法，不增加自动全程验收次数。

Car3搭载机械臂在同一番茄温室、默认算法及当前真实CAD轮心/自适应接地策略下的人工选点试验0042由用户确认通过，见 `config/manual_confirmations_car3_tomato_greenhouse.json`。仅覆盖本次组合，不外推其他场景或算法，不增加自动全程验收次数。

Car4搭载凿式犁在同一番茄温室、默认算法下的人工选点试验0043由用户确认通过，见 `config/manual_confirmations_car4_tomato_greenhouse.json`。仅覆盖本次组合，不外推其他挂载或场景，玉米田的NavSat专用规则不变，不增加自动全程验收次数。

Car5搭载凿式犁在同一番茄温室、默认算法下的人工选点试验0044由用户确认通过，见 `config/manual_confirmations_car5_tomato_greenhouse.json`。仅覆盖本次组合，不外推其他挂载、场景或算法，不增加自动全程验收次数。

Car6搭载凿式犁在同一番茄温室、默认算法下的人工选点试验0045由用户确认通过，见 `config/manual_confirmations_car6_tomato_greenhouse.json`。仅覆盖本次组合，不外推其他挂载、场景或算法，不增加自动全程验收次数。

Car7搭载凿式犁在同一番茄温室、默认算法下的人工选点试验0046由用户确认通过，见 `config/manual_confirmations_car7_tomato_greenhouse.json`。仅覆盖本次组合，不外推其他挂载或场景，农场运输区不适用的记录保持不变，不增加自动全程验收次数。

Car8受控前轮阿克曼版本搭载前置割草刀杆在同一番茄温室、默认算法下的人工选点试验0047由用户确认通过，见 `config/manual_confirmations_car8_tomato_greenhouse.json`。仅覆盖本次组合，不外推裸车、其他场景或旧差速脚轮结构，不增加自动全程验收次数。

Car0搭载机械臂在草莓种植区 `strawberry_field`、默认算法下的人工选点试验0048由用户确认通过，见 `config/manual_confirmations_car0_strawberry_field.json`。仅覆盖本次组合，不外推其他挂载、场景或算法，不增加自动全程验收次数。

Car1搭载凿式犁在同一草莓种植区、默认算法下的人工选点试验0049由用户确认通过，见 `config/manual_confirmations_car1_strawberry_field.json`。仅覆盖本次组合，不外推其他挂载、场景或算法，不增加自动全程验收次数。

Car2在当前草莓种植区由用户判定不适用，凿式犁组合试验0050已关闭，记录为 `USER_SCENE_NOT_APPLICABLE`，不标通过或算法失败。后续该场景人工验证不再安排Car2；其他场景记录不变。

Car3搭载机械臂在同一草莓种植区、默认算法及当前CAD轮心/自适应接地策略下的人工选点试验0051由用户确认通过，见 `config/manual_confirmations_car3_strawberry_field.json`。仅覆盖本次组合，不外推其他挂载、场景或算法，不增加自动全程验收次数。

Car4搭载凿式犁在同一草莓种植区、默认算法下的人工选点试验0052由用户确认通过，见 `config/manual_confirmations_car4_strawberry_field.json`。仅覆盖本次组合，不外推其他挂载、场景或算法，玉米田的NavSat专用规则不变，不增加自动全程验收次数。

Car5搭载凿式犁在同一草莓种植区、默认算法下的人工选点试验0053由用户确认通过，见 `config/manual_confirmations_car5_strawberry_field.json`。仅覆盖本次组合，不外推其他挂载、场景或算法，不增加自动全程验收次数。

Car6搭载凿式犁在同一草莓种植区、默认算法下的人工选点试验0054由用户确认通过，见 `config/manual_confirmations_car6_strawberry_field.json`。仅覆盖本次组合，不外推其他挂载、场景或算法，不增加自动全程验收次数。

Car8受控前轮阿克曼版本搭载前置割草刀杆在同一草莓种植区、默认算法下的人工选点试验0055由用户确认通过，见 `config/manual_confirmations_car8_strawberry_field.json`。仅覆盖本次组合，不外推裸车、其他场景或旧差速脚轮结构，不增加自动全程验收次数。

2026-10-21 用户统一验收口径：同一车型版本、场景、算法只测一个代表性机具，明确通过后其余当前已适配机具按用户规则分组确认，不逐个重跑。规则见 `config/manual_validation_policy.json`。保留代表试验与分组确认的区别，不将其余机具称为逐项实测；未适配机具、排除场景、其他算法或未来新增机具不在范围内。单独“下一个”不视为通过。

Car0搭载机械臂在葡萄园 `vineyard`、默认算法下的人工选点试验0057由用户确认通过，见 `config/manual_confirmations_car0_vineyard.json`。按上述规则确认当前已适配机具组（当前仅机械臂），不外推其他车型、场景或算法，不增加自动全程验收次数；0056未判定的原记录保留。

Car3搭载机械臂在同一葡萄园、默认算法及当前CAD轮心/自适应接地策略下的人工选点试验0059由用户确认通过，见 `config/manual_confirmations_car3_vineyard.json`。按用户规则确认当前已适配机具组（当前仅机械臂），不外推其他场景或算法，不增加自动全程验收次数；0058重启前未判定的原记录保留。

随后用户明确要求葡萄园中的Car3与Car4仅使用NavSat定位，已在两车 `sceneAlgorithms.vineyard` 中设置 `allowedLocalizationModes: [navsat]`。仅替换定位，Smac Hybrid和各车对应的MPPI运动学保持不变；其他车型/场景不受影响。Car3上述FAST-LIVO2人工通过记录保留为历史，不再作为该场景当前推荐；NavSat补测结论见下文。

Car4改用NavSat后的葡萄园凿式犁代表试验0061由用户确认通过，见 `config/manual_confirmations_car4_vineyard.json`。按用户规则分组确认当前八种已适配机具，其余机具未逐项重跑，不增加自动全程验收次数。

Car5在葡萄园使用默认FAST-LIVO2/RTK＋Smac Hybrid＋MPPI的凿式犁代表试验0062由用户确认通过，见 `config/manual_confirmations_car5_vineyard.json`。按用户规则分组确认当前八种已适配机具，其余机具未逐项重跑，不增加自动全程验收次数。

Car3搭载机械臂的葡萄园NavSat补测试验0063由用户确认通过，当前有效记录见 `config/manual_confirmations_car3_vineyard_navsat.json`。替代旧默认融合定位的有效推荐，旧记录不删除；仅覆盖当前适配机具组、NavSat配方和当前车辆版本，不增加自动全程验收次数。

用户随后要求葡萄园中的Car6也仅使用NavSat，已在 `Car6.sceneAlgorithms.vineyard` 设置NavSat专用规则。只切换定位，Smac Hybrid、MPPI Ackermann及其他场景配置不变；旧默认融合试验0064保留为未判定。NavSat凿式犁代表试验0065已由用户确认通过，见 `config/manual_confirmations_car6_vineyard.json`，按用户规则覆盖当前六种适配机具，其余机具未逐项重跑，不增加自动全程验收次数。

Car8受控前轮阿克曼版本搭载前置割草刀杆在葡萄园、默认算法下的人工选点试验0066由用户确认通过，见 `config/manual_confirmations_car8_vineyard.json`，按用户规则覆盖当前适配机具组（仅前置割草刀杆），不外推其他场景或旧差速结构，不增加自动全程验收次数。

Car0搭载机械臂在蔬菜试验田 `vegetable_field`、默认算法下的人工选点试验0067由用户确认通过，见 `config/manual_confirmations_car0_vegetable_field.json`。按用户规则覆盖当前适配机具组（仅机械臂），不外推其他车型、场景或算法，不增加自动全程验收次数。

Car1在同一蔬菜试验田、默认算法下的凿式犁代表试验0068由用户确认通过，见 `config/manual_confirmations_car1_vegetable_field.json`。按用户规则分组确认当前八种适配机具，其余机具未逐项重跑，不外推其他场景或算法，不增加自动全程验收次数。

Car2在同一蔬菜试验田、默认算法下的凿式犁代表试验0069由用户确认通过，见 `config/manual_confirmations_car2_vegetable_field.json`。按用户规则分组确认当前八种适配机具，其余机具未逐项重跑，不改变草莓种植区不适用的记录，不增加自动全程验收次数。

Car3搭载机械臂在同一蔬菜试验田、默认算法及当前CAD轮心/自适应接地策略下的人工选点试验0070由用户确认通过，见 `config/manual_confirmations_car3_vegetable_field.json`。按用户规则覆盖当前适配机具组（仅机械臂），葡萄园NavSat专用规则不变，不增加自动全程验收次数。

Car4在同一蔬菜试验田、默认算法下的凿式犁代表试验0071由用户确认通过，见 `config/manual_confirmations_car4_vegetable_field.json`。按用户规则分组确认当前八种适配机具，其余机具未逐项重跑，不改变玉米田与葡萄园的NavSat专用规则，不增加自动全程验收次数。

Car5在同一蔬菜试验田、默认算法下的凿式犁代表试验0072由用户确认通过，见 `config/manual_confirmations_car5_vegetable_field.json`。按用户规则分组确认当前八种适配机具，其余机具未逐项重跑，不外推其他场景或算法，不增加自动全程验收次数。

Car6在同一蔬菜试验田、默认算法下的凿式犁代表试验0073由用户确认通过，见 `config/manual_confirmations_car6_vegetable_field.json`。按用户规则分组确认当前六种适配机具，其余机具未逐项重跑，不改变葡萄园NavSat专用规则，不增加自动全程验收次数。

Car7在同一蔬菜试验田、默认算法下的凿式犁代表试验0074由用户确认通过，见 `config/manual_confirmations_car7_vegetable_field.json`。按用户规则分组确认当前九种适配机具，其余机具未逐项重跑；拖车转向约束不同，保留独立动力学复核标志，不冒充独立自动推荐验收。农场运输区不适用记录保持，不增加自动全程验收次数。

Car8受控前轮阿克曼版本搭载前置割草刀杆在同一蔬菜试验田、默认算法下的人工选点试验0075由用户确认通过，见 `config/manual_confirmations_car8_vegetable_field.json`。按用户规则覆盖当前适配机具组（仅前置割草刀杆），不外推其他场景或旧差速结构，不增加自动全程验收次数。

2026-10-21 用户明确宽地头 `wide_headland` 的所有车型统一使用NavSat，不再使用FAST-LIVO2/RTK或FAST-LIO定位。全局场景规则设置 `allowedLocalizationModes: [navsat]` 并由精确机具配方继承；保留原Smac/MPPI或RPP控制选择，不改变其他场景的定位规则。Car0旧融合定位试验0076保留为未判定，切换后需重新确认。

随后用户明确宽地头仅需继续测试最后的收割机，其余车型全部通过。分组记录见 `config/manual_confirmations_wide_headland_non_harvesters.json`，覆盖Car0至Car8的当前适配机具/NavSat配方；仅Car0机械臂0077为本轮代表试验，其他车型机具未逐一重跑，不冒充独立实测，不增加自动全程验收次数。Car9不在该通过组内，需单独测试。

最后Car9联合收割机搭载谷物收割割台的宽地头试验0078已由用户确认通过，见 `config/manual_confirmations_car9_wide_headland.json`，实际配方为NavSat＋Smac Hybrid＋RPP，使用当前自适应接地策略。仅为该组合的人工选点确认，不外推其他场景、裸机或收割作业，不增加自动全程验收次数。

Car1在玉米试验田使用NavSat、Smac Hybrid、MPPI Ackermann时，全部八种当前适配机具由用户分组确认通过，见 `config/manual_confirmations_car1_arable.json`。代表试验0018为凿式犁组合，不冒充八次独立自动全程验收，不扩展到其他场景或算法。

Car2在同一玉米试验田使用NavSat、Smac Hybrid、MPPI Ackermann的全部八种当前适配机具也获用户分组确认，见 `config/manual_confirmations_car2_arable.json`，代表试验0019为凿式犁。至此Car0至Car9的玉米田已开放机具共51个组合均有明确人工分组标签，各自算法按记录区分；不包含裸车，不等于51次独立自动全程验收，也不将该结论外推到其他场景。

### 推荐功能的数据导出

以下只读导出器供后续大模型推荐使用，不调用大模型、不控制车辆、不改变参数或验收条件。它重新检查实际全路线结果、定位质量及输入指纹，只输出精确组合的有效证据；中文车型、机具、场景名称同源读取。完整途经点直接读取该次验收的已指纹记录，不重新生成；显示开关、DDS域和RMW采用实际验收设置，不把未开RViz的验收命令写成已验证带RViz运行。

```bash
source /opt/ros/humble/setup.bash
source /home/cgz/agribot_ws/ros2_ws/install/setup.bash
python3 -m agribot_farm_sim.recommendations \
  --evidence-root /path/to/serial_runs \
  --output /path/to/new_recommendation_export
```

输出 `recommendations.json` 与 `recommendations.md`。前者 `recommendations` 仅含当前可推荐条目；`allSelections` 保留全部组合的通过、待测、失败或证据详情不足状态。每条提供真实参数目录、源配置SHA、原途经点、算法、半轴距终点容差、速度、指标及实际验收命令。所有条目仅验证运输导航，不代表机具作业能力或真机授权。

追加 `--watch 15` 可随串行验收每15秒检查更新；各次导出独立保存，`<output>/latest` 指向最近结果。输入文件变化也会重新审查，不继续推荐旧版本成绩。不要把测试计划清单直接作为已验收白名单。

Car1、Car2、Car4、Car5 已按外形恢复名义米制尺寸，传感器本体尺寸不放大。最新组合和证据见
[米制尺寸恢复后的验收](../../docs/farm_compatibility_results_20261019.md)。
`config/metric_transport_acceptance.json` 为 10 辆车各带一个机具的串行全路线验收计划；下方旧版本的测试记录与程序目录保留作为历史，不取代该验收。

## 后悬挂机具连接（2026-10-20）

原 14 种固定后挂机具配置已扩展到 46 种，均使用检查过的主机挂点和连杆/销轴，不再用悬空方块假连接。六辆拖拉机共用八种后挂机具的设计矩阵，保留两项需要配重设计的例外；全部保留原 CAD 尺寸和传感器安装参数。

`hitches.py` 保存规范化 CAD 坐标中的主机挂点和机具连接位置，核对轴距以防套到另一缩放版本。开沟机连接既有结构；大蒜播种机和覆膜机没有检查到可直接使用的原三点销轴，因此另加名义三角接收架，用四个夹板位置固定到原机架梁，再连接主机的三点悬挂。接收架明确记录为新增结构，不把它描述成原厂部件。三根连接杆及新增接收架都进入碰撞和自车过滤配置；无已检查挂点的后挂机具直接报错，不回退到方块连接。

`check_hitch_anchors.py` 使用 Blender 的三角面最近距离独立复核挂接端点；对机架式工具检查四个夹板脚与实际机架梁，而非用包围盒推断接触。此处是刚性、抬升运输的名义仿真设计，不认证真实销轴规格、举升能力、结构强度、PTO或液压接口，不模拟入土作业。

新三组本地原始导出为 `~/.local/share/agribot/farm_assets/inspected_hitches_v4/{tractor_garlic,tractor_mulch,cab_groove}`，旧 ROS 导出及 Unity 资源备份在 `~/.local/state/agribot-algorithm-validation/20261020_remaining_hitches/`。预览、实际 ROS 导出、校验清单同源，未改真实车辆外参。

六辆拖拉机与八种后挂机具的 48 对候选中，46 对通过名义几何、静态载荷和轮胎/转向包络间隙检查，新增 32 份同源配置与预览。紧凑拖拉机挂重型粉碎机或铣土机需要先设计配重，未开放安装。可安装、可导出不等于完成全部场景全程，动态测试记录独立核验。

`config/rear_mount_matrix.json` 是跨车型设计矩阵；`scripts/expand_rear_mount_matrix.py` 在新目录准备额外组合和清单提案，不隐式覆盖历史导出。历史同名导出必须明确选择并备份后才能更新，完成的准备目录不可重新覆盖。新增本地资产在 `~/.local/share/agribot/farm_assets/rear_matrix_v1`。Unity 安装按钮读取资源预设清单，预览按安装加载、卸载释放；宽机具按实际组合外廓重新取景，扩大原车库而非缩小车辆。

## 收割机、播种机与拖车（2026-10-19）

沿用原可滚动车库与装备面板，新增以下本地资源：

| 入口 | 导出 ID | 资源 | 安装与动力学 |
| --- | --- | --- | --- |
| Car9 | `tracked_harvester` | 履带式联合收割机主机 | 默认不带割台；四个隐藏接触轮作差速运输仿真 |
| Car9 → 谷物收割割台 | `harvester_header` | 主机 + 可拆割台 | 独立机具 `combine_header` / `CombineHeader`，固定自动安装与卸下 |
| Car6 → 大蒜播种机 | `tractor_garlic` | 约 1.30 × 1.20 × 0.56 m | 后置、名义 60 kg、固定抬升运输 |
| Car6 → 灌水覆膜播种机 | `tractor_mulch` | 约 1.26 × 1.06 × 1.12 m | 后置、空箱名义 80 kg、固定抬升运输 |
| Car7 → 双轴农用拖车 | `cab_trailer` | 约 5.76 × 2.94 × 3.55 m | 偏航/俯仰牵引连接、四个被动车轮、空载低速运输 |

`CarN` 只用于农机主机，农机具不分配 Car 编号。花生收割机库里的两份 STEP 经 SHA-256 确认相同，实际只有收获机构，不是完整自走车辆，未伪装成完整车型。Car9 使用完整联合收割机装配，源 CAD 是小比例模型；现按显式 `nominalScale: 10` 制作全尺寸名义仿真几何，主机约长 11.33 m、宽 4.54 m，割台外宽约 12.72 m，**不是厂家实测尺寸**。原 CAD 和旧版本保留。Unity 不再缩小整车，扩大原车库空间及相机范围；ROS 中接触轮、外廓、碰撞、惯性与该尺寸保持一致，视觉网格采用 20 mm。

`prepare_harvester.py` 根据已检查的实体 448～1340 拆出割台，并单独分离聚合开放曲面中的割台表面，避免卸下后残留外壳。主机包含驾驶室、送料机构、履带、后轮和卸粮管；割台只在装备面板中可选安装。`convert_step.py --include-open-surfaces` 保留源 CAD 未属于封闭实体的开放曲面，完整来源为 317910 个三角面。衍生模型不纳入 Git。

Car9 配置为履带差速近似运输模型，不模拟真实联合收割机的后轮转向、履带变形和收割过程。主机质量名义 14500 kg、割台名义 2500 kg，仅用于仿真。雷达、相机、IMU、双天线 RTK 复用现有车库模型，传感器本身不放大。`config/tracked_harvester_sensors.json` 规定全尺寸车体上的表面安装位置，ROS、Unity 共用。前述缩比版本的番茄行间成绩属于旧尺寸，**不能视为全尺寸新配置的路线验收**；大机型不能沿用窄温室的通行假设。

Car9 的仿真外观由 `scripts/prepare_harvester_appearance.py` 单独准备：黄色漆面贴图、深色玻璃、橡胶胎面和轮毂分材质。该脚本同时提取聚合开放曲面中的完整后轮胎面，恢复轮毂与胎面的共同可视轴心；履带分成左右各 64 个段，沿闭合参考曲线循环，速度来自对应侧的实际仿真驱动轮反馈。Gazebo 使用纯显示插件，RViz 使用 `Track Animation` 标记显示；不增加接地碰撞体、质量或驱动力，不改变尺寸配置、控制器或导航参数，也不代表实现了柔性履带力学。当前外观资源为 `tracked_harvester/ros_visual_v11` 和 `mounted/harvester_header_visual_v11`，原 CAD 和之前版本保留。

Car3 的小型履带外观由 `scripts/prepare_compact_tracks.py` 适配，资源位于 `compact_tracks/ros_v3`：复用 Car9 的滚轮 CAD，保持轮子圆形；重排滚轮中心并生成小型橡胶防滑履带块，左右各64段，深色履带与灰白轮毂搭配原车浅色车身。只移除原履带网格和遮挡滚轮的两块可视侧盖，保留支撑骨架、原传感器、原接地轮、质量、碰撞和导航配置。履带长约1.07m、宽0.15m，在原车外廓内；动画使用与Car9相同的实测仿真轮角反馈。此处仅为外观适配，不构成新的导航全程验收。

Unity的Car3场景导出与Car9资源包导出已接入上述外观：`prepare_unity_farm.py --appearance-only --source unity_car3=PATH` 准备Car3显示覆盖资源，常规 `--source tracked_harvester=PATH --source harvester_header=PATH` 准备Car9的裸车和割台组合。预览从生成的URDF及履带零位姿同源构建，PNG贴图随模型一起校验、复制；Car3保留当前场景的传感器与装备导出，不覆盖物理参数。Unity主界面两张缩略图也由相同预览生成，并记录来源SHA以阻止新旧外观混用。派生模型仍为本地资源，不进入Git。

全尺寸主机的雷达/相机安装在前部送料平台，雷达参考点 `(4.3, 0, 1.2501)` m；原较高方案 `(3.9, 0, 3.6984)` m 在玉米地头测试中出现定位退化，失败记录保留。新方案完成玉米地头 6 m 直线目标，真值位移 5.746 m、水平定位 RMSE 0.193 m、最大误差 0.520 m、RTK 创新量拒绝 0、进程正常退出。这是短程链路检查，不是全路线或高精度验收。带割台必须使用大于其 12.72 m 外宽且留有缓冲的开阔运输空间，未验证收割作业或狭窄行间路线。记录为 `20261019_fullsize_harvester_feeder_arable`。

拖车牵引连接限制偏航 ±0.25 rad、俯仰 ±0.15 rad；采用接触柔顺参数使双轴四轮都接地滚动。组合的名义最小转弯半径为 16 m，车速仍不超过 0.30 m/s。导航和自车过滤使用覆盖全部允许偏航、俯仰的保守扫掠外廓，不沿用裸车轮廓。玉米场景的初始位置仅对该组合改为 `(13, 38, 0)`，避开原起点后方围栏，原场景规定路径点保持不变。它需要开阔运输空间；尚不支持拖车倒车规划，也未通过狭窄行间或全部预设路线验收。

模型处理与运行目录：

- Car9：`~/.local/share/agribot/farm_assets/tracked_harvester/ros_fullsize_v3`；割台 `combine_header/normalized_v2`；组合 `mounted/harvester_header_v3`。旧 `ros_v7` 缩比配置及 `ros_fullsize_v2` 留作备份。

兼容复核发现原雷达/相机测量点在割台碰撞包络内，v3 仅把名义收割机支柱从 0.2 m 提高到 0.8 m；雷达和相机各升高 0.6 m，其他传感器、车体尺寸和动力学不变，也不改真机。该修复不等于带割台运输通过：农场试放仍有地形/出生高度问题，暂不推荐该组合行驶。旧失败与复测记录保留。
- 大蒜/覆膜机：`garlic_planter/ros_v1`、`mulch_planter/ros_v1`；组合 `mounted/tractor_garlic_v1`、`mounted/tractor_mulch_v1`。
- 拖车：`farm_trailer/ros_v1`；组合 `mounted/cab_trailer_v4`。
- 车辆 CAD 与派生网格仍只在本地忽略目录中，未验证再分发许可；本轮不更改真机参数。

历史记录：已验证大蒜播种机和覆膜机组合使用 FAST-LIVO2+RTK、Smac Hybrid-A*、MPPI 跑完番茄温室原 3 点约 12 m 路线，水平定位 RMSE 分别约 0.0160 m、0.0069 m。旧缩比 Car9 的前部安装方案连续两轮完成同一原 3 点路线，水平 RMSE 分别为 0.0196 m、0.0702 m，固定解因子未被创新量拒绝。拖车完成 6 m、横向偏移 0.4 m 的缓弯运输，水平 RMSE 约 0.0267 m；独立关节订阅确认四个车轮均滚动超过 13 rad，牵引偏航约 0.10 rad、俯仰约 -0.018 rad，均在限位内。这些是上述场景和方案的验收，不代表所有场景、算法或真实作业过程均已验证。

证据在 `~/.local/state/agribot-algorithm-validation/20261019_tractor_garlic_tomato`、`20261019_tractor_mulch_tomato`、`20261019_harvester_hood_tomato`、`20261019_cab_trailer_joint_validation_v4`。未通过的较高安装、围栏重叠、狭窄走廊和后轮离地测试记录仍保留。

本轮资源、机具、场景和关节生成检查为 96 通过、1 项本地资源跳过。更广的车辆生成器检查为 34 通过、2 个原 Car0 外参断言失败：已有 `Pcl.y=-0.07900238 m` 与测试写定的 `-0.079 m` 相差约 2.38 微米，略超测试的 2 微米容差。未为这项既有差异更改真实标定或放宽断言。

上一版 `farm_forms` 的 Unity 资源/导出检查 29 项、原装备交互 8 项通过，14 个预设打包哈希一致，记录保留于 `20261019_farm_forms_release`。其中长拖车曾缩小展示；现已取消，改为扩大原车库，主机及机具始终维持米制 1:1 展示。新版本为 `~/Builds/AgriculturalCar_Interior_Linux_20261019_fullsize_harvester_v2`，共 15 个预设；资源/导出检查 31 项、完整交互回归 9 项通过，最终传感器安装方案及裸车/挂载取景另做专项回归。旧构建保留，本轮未提交或推送 Git。

## Car8 前置割草刀杆（2026-10-19）

新增可选 `front_sickle_bar`，仅适配 Car8 `ride_on_mower`。它是本项目生成的参数化仿真组件，不是采购 CAD 库或厂家实物。整套约长 0.91 m、宽 1.63 m、高 0.30 m，含刀齿、护指、护罩、滑靴和连接车架的支臂；只作固定运输挂载，不模拟割草、往复驱动或升降。名义质量 30 kg、工作宽度 1.45 m、离地间隙 0.12 m，不用于真实承载评估。

固定参考点位于 `base_link` 前方约 1.584 m；机具最前沿约 1.934 m。支臂穿过前脚轮之间，检查了轮胎完整回转包络与机具碰撞盒间的间隙；传感器外参不变。导出自动包含机具外廓、Nav2 缓冲轮廓、雷达自车过滤盒、工作坐标系和固定连接，不得沿用未挂载时的裸车轮廓。新增组合同时正确重定向前脚轮网格路径。

本地资源：`~/.local/share/agribot/farm_assets/front_sickle_bar/ros_v2`；当前受控前轮组合：`mounted/mower_front_cutter_v2_ackermann`；运行 ID：`mower_front_cutter`。Unity 原装备面板增加“前置割草刀杆”，安装、卸下决定导出组合；不兼容车型仅禁用安装按钮。原 `mower_front_cutter_v1` 保留为差速脚轮版本历史。

重建时使用新的输出目录，保留旧资产：

```bash
blender -b --python src/agribot_farm_sim/scripts/prepare_front_cutter.py -- /tmp/front_cutter_new
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run agribot_farm_sim farm_resources mount \
  "$HOME/.local/share/agribot/farm_assets/ride_on_mower/ros_v6_ackermann/vehicle_config.json" \
  /tmp/front_cutter_new --vehicle-id mower_front_cutter --output /tmp/mower_cutter_new
ros2 run agribot_vehicle_config_tools agribot_vehicle_config generate \
  /tmp/mower_cutter_new/vehicle_config.json --output /tmp/mower_cutter_bundle \
  --workspace-src "$HOME/agribot_ws/ros2_ws/src"
```

2026-10-19 配置、挂载及脚轮回归检查 42 项通过；扩大到农机资源包后共 81 项通过、1 项资源条件跳过。Unity 资源/导出检查 21 项、交互回归 7 项通过。FAST-LIVO2+RTK、Smac Hybrid、MPPI DiffDrive 的实测如下，退出均正常：

| 场景 | 范围 | 水平定位 RMSE | 实时率 |
| --- | --- | --- | --- |
| 番茄温室 | 全部 3 点，约 12 m | 0.0114 m | 0.847 |
| 蔬菜试验田 | 全部 16 点，约 92 m，含两次掉头 | 0.0273 m | 0.883 |

证据在 `~/.local/state/agribot-algorithm-validation/20261019_mower_front_cutter_tomato`、`20261019_mower_front_cutter_vegetable` 和 `20261019_unity_front_cutter`。这只是名义机具的上述仿真运输验证，不是实际割草能力或全部场景/算法组合验收。

## 场景与来源

| 选择项 | 启动参数 | 模型来源 | 适配内容 |
| --- | --- | --- | --- |
| 玉米试验田 | `arable_field` | [Field Robot Event](https://github.com/FieldRobotEvent/virtual_maize_field) 的摄影测量玉米 | 五组双行玉米、土壤纹理、围栏、地头通道 |
| 温室行间 | `greenhouse` | [HARBOT](https://github.com/AV-Lab/harbot-greenhouse-models) | 保留棚架、作物和果实，棚顶和侧面采用乳白半透明塑料薄膜，去掉棚外高面数装饰树 |
| 农场运输区 | `farmyard` | [Clearpath Agriculture](https://github.com/clearpathrobotics/cpr_gazebo) | 保留谷仓、围栏、太阳能设施和起伏地面 |
| 番茄温室 | `tomato_greenhouse` | [LCAS AOC Tomato Farm](https://github.com/LCAS/aoc_tomato_farm) | 三段棚架、两行共 38 株番茄、花盆与种植床、中央通道 |
| 草莓种植区 | `strawberry_field` | [Harvester Sim](https://github.com/jsather/harvester-sim) | 三行种植垄、72 株草莓及果实纹理、周界围栏和入口、五点行间转弯路线 |
| 葡萄园 | `vineyard` | [CropCraft](https://github.com/Romea/cropcraft) | 三行共 33 株葡萄藤、原叶片纹理、自建支架和架线、七点行间转弯路线 |
| 蔬菜试验田 | `vegetable_field` | [PIC4SeR Automatic Row Crop Generator](https://github.com/PIC4SeR/AutomaticRowCropGenerator) | 洋蓟、西葫芦、莙荙菜、生菜四个分区，158 株原尺度作物、土壤纹理、围栏及巡检通道；验收范围见下文 |

纹理最大分辨率为 2048；视觉网格与碰撞网格分别导出。二维地图由碰撞几何投影，排除地面；温室和玉米田投影高度带为 0.18～2.2 m。各场景启动时根据车辆主天线杆臂和出生位姿生成 ENU 到地图的变换，不再沿用果园原点。

温室薄膜沿用原有棚顶和端面/侧面网格，材质不透明度为 0.28，调整仅影响外观和相机渲染；碰撞网格、雷达碰撞几何和二维地图未改动。不模拟真实塑料薄膜的激光透射率。

```bash
source /opt/ros/humble/setup.bash
source ~/agribot_ws/ros2_ws/install/setup.bash
ros2 launch agribot_vehicle_description configured_ackermann_sim.launch.py \
  scenario:=greenhouse gui:=true rviz:=true \
  localization_mode:=fastlivo_rtk planner_mode:=smac_hybrid controller_mode:=mppi
```

Unity 的“ROS 仿真方案”面板有同名场景选择项。场景 ID 传给 `unity_sync_and_simulate.sh --scenario`；默认仍为 `orchard`。

## 资源授权

`assets/*/source.json` 记录上游地址、固定提交、改动和几何统计。上游许可随各资源目录保留：

- Clearpath 农场：BSD-3-Clause，2020 Clearpath Robotics。
- HARBOT 温室：MIT，2026 Autonomous Vehicle Laboratory。
- AOC 番茄温室：Apache-2.0；保留模型许可和固定来源提交。转换时读取原 SDF 的尺度，修正原材质的透明纹理引用，不运行 ROS 1 插件。
- 草莓与种植垄：BSD-2-Clause，2019 Jonathon Sather，固定提交 `14ff00da0e3c5955f8f6934171978c9523ce8003`。原 SDF 的无引号属性通过结构化解析读取，校验源文件哈希；不运行原 ROS 1 机械臂和插件。周界木栅栏是本项目补充的场地几何，不宣称来自上游。
- 葡萄植株：CropCraft Apache-2.0，固定提交 `7128cd2acade50cc4a5a1761210b55989ab62527`。保留四种 `vine1` OBJ 植株的米制尺寸和纹理；支架、架线及布局由本项目生成，土壤为单独署名的 Poly Haven CC0 纹理。
- 蔬菜植株：PIC4SeR 固定提交 `7d8ebd6e52ade4ce8e0c89b67e2f048ec9e6989e`；作者在 README 中声明仓库代码和模型采用 MIT，全文保存在 `LICENSE.upstream.txt`。每类使用两种植株及所附纹理，不执行上游生成器；分区布局和围栏由本项目生成，地面另用 Poly Haven CC0 土壤纹理。
- 玉米模型：CC BY-SA 4.0，2021 Kamaro Engineering e.V.；按该许可保留模型衍生资源的署名和许可。未引入上游 GPL 场景生成器代码及禁止商用素材。
- 玉米田地面：[Brown Mud](https://polyhaven.com/a/brown_mud)，Rob Tuytel / Poly Haven，CC0；Ground003 草地纹理为 CC0Textures.com 的 CC0 资源。

`scripts/import_open_scene.py` 是资源转换工具。它读取固定版本模型，不运行上游 ROS 1 插件。重新制作场景时使用新输出目录，核对地图、碰撞和纹理后再替换 `assets`。

## 验证范围

2026-10-15：三个新场景均已加载真实网格与纹理，检查了相机、雷达、IMU、模拟 RTK、地图和路径输出。农场和玉米田完成约 6 m 短程运动，温室完成约 6 m 带横向调整的运动；无界面复测的实时率约为 0.82～0.90。这些不是全部车型、全部算法组合或整条作业路线的验收。RTK 验证同时检查实际接受的因子数量，不能仅以固定解状态判断融合生效。

当时的 Unity 场景选择构建为 `/home/cgz/Builds/AgriculturalCar_Interior_Linux_20261015_fixed_mounts`，最新构建见下方“2026-10-16 扩展与验收”。进入“启动仿真”的方案面板选择场景。传感器和组件恢复固定默认位置，不提供拖动编辑。

运行证据位于 `~/.local/state/agribot-algorithm-validation/20261015_*`。图形界面的测试还记录到 RViz 在统一 SIGINT 退出阶段异常结束；不应将该记录当作全流程无异常通过。算法无界面复测与图形显示证据分别保存。

作物和果实当前是静态环境，不模拟作物生长、农药雾化、土壤变形或采摘接触力。购入 CAD 包的清单和转换工具不代表其再分发授权，也不代表所有 CAD 已经成为可运动的车辆或机具。

## 本地 CAD 扩展

原始资源与衍生网格保存在 `~/.local/share/agribot/farm_assets/`，不纳入远程仓库。转换过程保留源文件 SHA-256、米制尺寸、坐标变换和独立零件。不得通过整体缩放把大型农具伪装成适用于小车的机具。

- `compact_tractor_cad.json` 配合 `scripts/prepare_tractor.py`：分离四个轮组，保留前后轮不同半径，生成名义仿真参数。Unity 在原车库中增加该车，仍不用于真机。
- `compact_chisel_plow_cad.json` 和 `mounted_disc_harrow_cad.json` 配合 `scripts/prepare_implement.py`：将约 2.26 m 宽的凿式犁、约 2.86 m 外宽的圆盘耙转为 ROS FLU，原点设在网格前沿地面中心。此原点不是已标定的三点悬挂销轴；固定运输挂载已完成短程验证，液压升降、铰接、入土力和真实作业效果尚未实现。两者分别约 6.05 万和 6.51 万三角面，网格尺寸与原始实体保持一致。
- 其他已转换候选包括播种机和大型凿式犁。只有转换/预览，不等于可在仿真中使用；六行开沟机的后续挂接验证见下文。

新拖拉机的 FAST-LIVO2+RTK、Smac Hybrid 和 MPPI 已通过 6 m 直线及 8 m 前进/1.5 m 横向偏移短程，水平定位 RMSE 分别约 0.030 m 和 0.029 m，实时率约 0.91。RPP 6 m 对照测试通过；原阿克曼车型 6 m 回归通过。上述均不是全路线验收。

大转弯半径车型的仿真 MPPI 若 `wz_std > wz_max`，将采样标准差限制为 `0.5 * wz_max`；原小车未触发此规则。新拖拉机通过 `metadata.simulationVelocityFromPose` 在仿真桥接配置中开启位姿差分速度，避免 FAST-LIVO2 未填写的全零速度字段直接传给控制器；既有车型和真机默认仍使用原速度来源。单独补齐速度并未解决偏离，缩小过大的角速度采样才使这两项测试通过。

测试记录：

- `~/.local/state/agribot-algorithm-validation/20261015_tractor_bounded_sampling`
- `~/.local/state/agribot-algorithm-validation/20261015_tractor_turn_sampling`
- `~/.local/state/agribot-algorithm-validation/20261015_original_vehicle_regression`

## 固定悬挂机具

`farm_resources mount` 将独立机具资源挂到适配拖拉机后方，不修改输入配置或源 CAD。凿式犁、圆盘耙适配紧凑型拖拉机；六行开沟机、悬挂式中耕机、林地粉碎机适配带驾驶室拖拉机。不通过缩放将其装到原阿克曼小车。固定运输状态离地约 0.15 m；机具质量暂用名义值，凿式犁 60 kg、圆盘耙 100 kg，其余见后文，不代表厂家参数或真机标定结果。

导出内容同时包含：

- 独立 OBJ/MTL、固定安装关节、机具工作坐标系和名义惯性。
- 包含机具外廓及缓冲量的 Nav2 轮廓；因此转弯时也会检查机具扫过的区域。
- 与碰撞几何同源的多包围盒雷达自车过滤，不把车体和机具之间的全部空白区域合并过滤。
- `simulationOnly: true` 与 `fixed_transport` 标识。机具滚筒和圆盘此时随整体刚性运动，尚未设置单独转动关节。

示例在本地资源已转换的前提下运行，输出目录必须是新目录：

```bash
source /opt/ros/humble/setup.bash
source ~/agribot_ws/ros2_ws/install/setup.bash
ASSETS=$HOME/.local/share/agribot/farm_assets
ros2 run agribot_farm_sim farm_resources mount \
  "$ASSETS/compact_tractor/ros_v6/vehicle_config.json" \
  "$ASSETS/compact_chisel_plow/ros_v1" \
  --vehicle-id tractor_chisel --output "$ASSETS/mounted/tractor_chisel_new"
ros2 run agribot_vehicle_config_tools agribot_vehicle_config generate \
  "$ASSETS/mounted/tractor_chisel_new/vehicle_config.json" \
  --output "$HOME/agribot_ws/ros2_ws/src/agribot_vehicle_description/generated/tractor_chisel" \
  --workspace-src "$HOME/agribot_ws/ros2_ws/src"
cd ~/agribot_ws/ros2_ws
PATH=/usr/bin:/bin:$PATH colcon build --packages-select agribot_vehicle_description --symlink-install
source install/setup.bash
ros2 launch agribot_vehicle_description configured_ackermann_sim.launch.py \
  vehicle_id:=tractor_chisel scenario:=arable_field \
  gui:=true rviz:=true run_waypoints:=false \
  localization_mode:=fastlivo_rtk planner_mode:=smac_hybrid controller_mode:=mppi
```

圆盘耙使用 `mounted_disc_harrow/ros_v1`，车辆 ID 为 `tractor_harrow`。当前资源为拖拉机 `ros_v6` 和挂接组合 `mounted/tractor_chisel_v4`、`mounted/tractor_harrow_v4`，大蒜和覆膜播种组合为 `mounted/tractor_garlic_v2`、`mounted/tractor_mulch_v2`。旧资源保留。它们保留 OBJ 材质修正，并复用原车库的雷达、海康相机、IMU 和双天线 RTK 网格。`prepare_fixed_sensors.py` 根据车体表面和传感器网格底部确定固定安装高度，`prepare_unity_farm.py` 从同一 URDF 生成 Unity 预览，因此展示与 ROS 导出使用相同安装参数。原六辆车的安装点不改。

2026-10-19 带图像显示复测发现 Car6 原雷达安装点 x=1.60 m 前方的车体部件遮挡相机。现按 `config/compact_tractor_sensors.json` 将雷达与相机组合移至机罩前部 x=2.00 m，仍由机罩支撑，不通过隐藏车体网格或增大近裁剪面掩盖遮挡。`check_camera_clearance.py` 检查相机中央水平正负 15 度、垂直正负 10 度的 35 条射线，旧位置 27 条被挡，新位置全部畅通；实际画面仍须在仿真中检查。这是名义仿真安装点，不是真机外参标定。

2026-10-15 实测：

| 组合 | 动作 | 真值位移 | 水平定位 RMSE | 实时率 | 状态 |
| --- | --- | --- | --- | --- | --- |
| 拖拉机 + 凿式犁 | 前进 6 m | 5.827 m | 0.034 m | 0.92 | 短程通过 |
| 拖拉机 + 圆盘耙 | 前进 8 m，横向偏移 1.5 m | 7.940 m | 0.027 m | 0.95 | 短程通过，含 Gazebo 显示 |

记录分别位于 `~/.local/state/agribot-algorithm-validation/20261015_tractor_chisel`、`20261015_tractor_harrow_gui`，对应修改传感器安装点之前的短程测试。目标检查器允许约 0.25 m 的终点误差，所以真值位移不必等于指令距离。这不是田间整条路线或耕作效果验收。

## Unity 安装流程

在原车库滚动选择紧凑型轮式拖拉机，打开原装备面板。机械臂、凿式犁和圆盘耙位于同一面板；点击安装后自动使用固定挂接位，不提供拖动。犁和耙适配新拖拉机，共用后方挂接点，互斥安装；机械臂仍适配原阿克曼车和履带车。不兼容时只禁用安装按钮，不显示适配说明文字。

安装四类传感器后可沿用原“启动仿真”入口，导出当前无农具、犁或耙的完整配置。导出会校验资源哈希，并保留机具碰撞体、质量、外廓和传感器外参等扩展字段，不能通过 Unity 的有限字段结构重新序列化而丢失它们。模型及图标从本地资源生成，不纳入远程仓库。

本轮相关检查共 82 项通过。扩大到整个配置工具测试集时另有两项原有外参期望值测试失败：实际相机 y 向转换值为 -0.07900237675 m，旧期望为 -0.079 m，差约 2.38 微米，超过其 2 微米容差。已核对 Git HEAD 的计算结果与当前完全相同；未为通过测试修改传感器外参。

固定安装界面另通过 19 项 Unity 检查：原六车传感器与两车机械臂回归 10 项、资源和导出检查 8 项、新车安装/卸载/机具互斥/导出交互 1 项。实际桌面逐项检查了原装备面板中的犁、耙与传感器安装，并修正新车装备视角穿入车库墙后的问题。构建目录为 `~/Builds/AgriculturalCar_Interior_Linux_20261015_fixed_implements`，旧的 `fixed_mounts` 构建保留备份。

Unity 测试结果和实际安装截图保存在 `~/.local/state/agribot-algorithm-validation/20261015_unity_fixed_installs/`。

采用固定 CAD 传感器位置的最新短程记录在 `~/.local/state/agribot-algorithm-validation/`：

- `20261015_fixed_cad_sensors_chisel_retry`：6 m 前进指令通过，真值位移 5.831 m，水平定位 RMSE 0.018 m，实时率 0.91。
- `20261015_fixed_cad_sensors_harrow`：8 m 前进/1.5 m 横向偏移指令通过，真值位移 7.977 m，水平定位 RMSE 0.017 m，实时率 0.90。
- 首次 `20261015_fixed_cad_sensors_chisel` 行驶通过，但退出时 ROS 桥接节点异常结束，因此总结果为 FAIL；该失败记录保留，复测退出正常。

上述只验证固定运输挂载和短程导航，不表示已完成全路线或土壤作业动力学验收。

## 2026-10-16 扩展与验收

新增 Car7 带驾驶室轮式拖拉机，仍在原滚动车库选择，不新增页面。四类传感器继续使用已有模型并自动安装；不兼容的农具安装按钮禁用。初次发布时尚无挂接农具；随后新增的六行开沟机使用独立挂接参数，不套用紧凑型拖拉机参数。

最终资源为 `cab_tractor/ros_v7`，车体长约 5.71 m、宽约 3.19 m，不做整体缩小。IMU 和 RTK 位于车顶，雷达与相机固定在前部发动机罩位置。最初的高位雷达在低矮玉米场景出现定位发散；降低安装高度后同一 6 m 测试通过，未放宽 RTK 创新量拒绝阈值。安装坐标和导出哈希在 `sensor_mount_report.json`、Unity 的本地资源清单及运行记录中保存。

该车型使用名义仿真质量 1500 kg、驱动轮端扭矩上限 1500 Nm、最高速度 0.3 m/s。扭矩由小车默认值 200 Nm 调整后，起伏场地的静止保持测试通过。这些值不是厂家实测参数。局部地图扩大为 14 m，膨胀半径为 5 m；原六车的 8 m 局部地图和真机参数不变。Nav2 仍有圆形优化碰撞检查的膨胀代价警告，完整轮廓检查保留，不宣称所有警告已经消除。

固定版本 FAST-LIVO2+RTK、Smac Hybrid、MPPI 的最终测试：

| 场景 | 测试范围 | 水平定位 RMSE | 实时率 | 结果 |
| --- | --- | --- | --- | --- |
| 玉米试验田 | 前进 6 m 短程 | 0.0314 m | 0.95 | 通过 |
| 农场运输区 | 前进 6 m 短程 | 0.0458 m | 0.93 | 通过 |
| 番茄温室 | 原生路线执行器完整 3 点、约 12 m | 0.0112 m | 0.85 | 通过 |

证据分别在 `~/.local/state/agribot-algorithm-validation/20261016_cab_hood_arable`、`20261016_cab_hood_farmyard`、`20261016_cab_hood_tomato_route`。全部接受了 RTK 因子，退出正常。实时率包含本机当时的其他负载，不是独占资源基准。失败的高位安装测试记录保留。这些不是所有车型、全部场景或全部算法组合验收；原果园 23 点不改。

本轮 Unity 编辑器检查 12 项和安装/导出交互检查 6 项通过。构建为 `~/Builds/AgriculturalCar_Interior_Linux_20261016_farm_expansion`，桌面入口已更新，旧 `20261016_car0` 构建保留。原来已打开的旧程序需要正常退出后重新从桌面启动，不能自动变成新版本。

## 全量资源入库

用户目录“240套农业机械农具农机收割播种旋耕SolidWorks三维3D模型图合集”实际含 244 个 ZIP：105 个含中性 CAD/网格，138 个需要原生 CAD 转换，1 个只有内嵌压缩包待展开审查。226 份中性几何文件按 SHA-256 去重后为 204 份，这不是 204 辆完整车辆：其中包含零件、装配体和不同导出格式。

完整本地目录：`~/.local/share/agribot/farm_assets/catalog_20261016.json`。所有条目保留，包括不能立即转换的文件；文件名分类仅为待人工复核的提示，不据此判定阿克曼/差速、挂接方式或作业能力。CAD 和派生网格仍不进入 Git。

```bash
source /opt/ros/humble/setup.bash
source ~/agribot_ws/ros2_ws/install/setup.bash
ros2 run agribot_farm_sim farm_resources catalog \
  "$HOME/Downloads/240套农业机械农具农机收割播种旋耕SolidWorks三维3D模型图合集" \
  --hash-neutral --output "$HOME/.local/share/agribot/farm_assets/catalog_20261016.json"
```

`scripts/batch_prepare_cad.py` 按明确的目录 ID 顺序转换 STEP/IGES，优先 STEP，并验证输入哈希、限制单项内存和运行时间、保存可恢复的结果。多 STEP 文件不擅自挑选一个零件充当整机；须另行确认完整装配体。转换完成只标记 `converted_geometry_only`，不会自动进入可驾驶车型列表。

2026-10-16 已转换下一批四个候选：大蒜播种机、带装载臂拖拉机、New Holland CR1090 CAD、土钻农业地钻设备。四项均已生成减面预览；拖拉机原始轴向需要归一化，收割机预览存在装配姿态及尺度疑点，不能自动放大后当作整车使用。地钻验证了 IGES 转换流程，外观是独立支架式工具，未确认拖拉机挂接接口；不自动装到车辆上。四项均还需运动学、碰撞、安装及运行测试，不能视作新增四个已验证可运行资源。

本地结果位于 `~/.local/share/agribot/farm_assets/intake_batch_20261016/`，`batch_report.json` 记录转换和缓存复用，`geometry_review.json` 记录人工预览结论。批量工具和相关回归测试为 35 项通过、3 项跳过；四项缓存复用检查通过。

网络场景来源与待办记录在 `config/scene_sources.json`，与可运行的 `config/scenarios.json` 分离。目前记录 15 个来源：7 个已接入（蔬菜试验田仅完成 Car0 全程验收），加上原苹果园共 8 个场景；另外 8 个候选或待补资源。候选包括 BACCHUS 葡萄园、CottonSim、程序化番茄田、FIRA 农场、Forest3D、Feldfreund Devkit、土壤检测农场和 Botanbot。逐项检查资产许可，不直接把外部 ROS 1、Harmonic 插件装入当前 ROS 2 Humble/Gazebo Classic 栈。

具体阻碍也要保留：CottonSim 当前提交缺少 URDF 引用的六份 DAE，BACCHUS 需另外获取外部模型；Forest3D 的生成器许可和资产许可须分别检查。无法保证穷尽全网，也不能把“找到仓库”或“下载成功”当作接入完成。

后续顺序为：完整资源目录、逐批几何审查、运动与挂接适配、Unity 原界面接入、Gazebo/算法验证、最后扩大自然语言可选择资源集合。真实农田扫描放在这些步骤之后。待验证资源不向任务执行器声明可运行。

## 六行开沟机运输挂载

资源来自目录中的 `Implement - Planting groove maker.STEP`，目录 ID 为 `b9c0efbf6e84d4ec`。保留原比例，转换后约长 1.20 m、宽 5.51 m、高 2.10 m，60121 个三角面。`groove_maker_cad.json` 将源坐标变换为 ROS FLU；本地资源在 `groove_maker/ros_v1`，挂接组合为 `mounted/cab_groove_v1`，运行车辆 ID 为 `cab_groove`。

Unity 在原装备面板新增“六行开沟机”，仅 Car7 可以安装。固定挂在车后，运输离地间隙 0.15 m、名义质量 220 kg；传感器坐标与 Car7 单车一致。导出保留碰撞几何、整体轮廓、工作坐标系与来源哈希。没有实现 PTO 传动、液压升降、开沟入土或土壤变形，任务标签描述机具类别，不代表这些作业已经可仿真。

大尺寸车型的仿真 MPPI 在缩小角速度采样后，同步按方差比例调整 `gamma`，将路径航向纠偏阈值限制在预测时域可转动的角度内，并按 `min(1, 1.3 / 最小转弯半径)` 缩放 Humble 的软转弯半径代价；速度、转角和硬曲率限制不放宽。原六辆车未触发这一大半径适配。来自 `inspected_local_CAD_profile` 且标记为 `unity_nominal_simulation_only` 的新 CAD 车型，还按带农具的完整轮廓推导膨胀范围及衰减上限，使外接圆处仍保留非零代价。原配置文件、原六车和真机膨胀参数不改。

2026-10-16 的独立运行记录：

| 记录目录后缀 | 测试 | 结果 |
| --- | --- | --- |
| `cab_groove_rpp_diagnostic` | RPP，前进 8 m、左移 1.5 m | 通过，定位 RMSE 0.0172 m |
| `cab_groove_curvature_cost` | MPPI，同一弯线 | 通过，定位 RMSE 0.0150 m |
| `cab_groove_repeat` | MPPI，独立重复运行 | 通过，定位 RMSE 0.0173 m |
| `cab_groove_inflation` | MPPI，补齐宽机具膨胀后 | 通过，定位 RMSE 0.0165 m，实时率 0.93 |
| `compact_curvature_regression` | 紧凑拖拉机及圆盘耙弯线回归 | 通过，定位 RMSE 0.0120 m |
| `compact_inflation_regression` | 紧凑拖拉机及圆盘耙，补齐膨胀后 | 通过，定位 RMSE 0.0122 m，实时率 0.93 |
| `original_groove_regression` | 原阿克曼车在苹果园前进 6 m | 通过，定位 RMSE 0.0198 m，实时率 0.70 |

完整路径均位于 `~/.local/state/agribot-algorithm-validation/20261016_` 加表内后缀。早期未调通的 MPPI 失败记录没有删除；`cab_groove_right_turn` 的右侧目标被规划器拒绝，检查代价地图确认该目标下机具外廓与障碍区重叠，未缩小轮廓强行放行。补齐膨胀后不再出现外接圆优化检查的膨胀范围警告，但仍有部分 10 Hz 控制周期超时日志；这些只构成短程运输导航验证，不是所有场景、全部路线或稳定实时性的全面验收。

该轮资源和参数回归 73 项通过、1 项跳过；Unity 编辑器 14 项和安装交互 6 项通过。已检查桌面程序的原装备面板及安装预览，无新增页面或大字适配说明。当时构建为 `~/Builds/AgriculturalCar_Interior_Linux_20261016_groove_maker`；后续场景扩展见下一节，旧构建保留，旧窗口不会自动升级。测试 XML、构建日志及安装截图在 `~/.local/state/agribot-algorithm-validation/20261016_unity_groove/`。未提交或推送 Git，CAD 派生资源继续处于忽略范围。

## 草莓种植区

`strawberry_field` 在现有场景选择栏内提供。采用上游真实草莓叶片、果实、种植垄和土壤纹理，保留米制尺度，使用约 6000 面的单株 LOD；三行共 72 株、12 段种植垄。场地约 32 x 26 m，垄间中心线相距 6 m，植株最高约 0.67 m。另补充 1.2 m 高周界围栏和 3.6 m 宽入口，围栏不是上游模型。只验证静态作物巡检导航，不包含草莓采摘动力学。

默认从 `(-10, -3, 0)` 出发，五个规定点为 `(-3,-3,0)`、`(6,-3,0)`、`(9,0,pi/2)`、`(6,3,pi)`、`(-6,3,pi)`；地头三点为半径 3 m 的半圆及切线航向。原苹果园 23 点未改。大尺寸车辆或宽农具不保证能通过本场景所有通道，不能将小车结果推广为全车型验收。

第一次仅含平行低矮种植垄的版本发生沿行方向严重漂移，已停止，保留 `20261016_strawberry_full_route` 的中断记录。补齐正常围栏后定位稳定，但最初外凸的地头目标点仍有 0.86 m 跟踪偏差，`20261016_strawberry_enclosed_route` 如实记录失败。改为连续半圆布局后，`20261016_strawberry_arc_route` 中 Car0 的 FAST-LIVO2+RTK、Smac Hybrid、MPPI 完成全部五点：水平定位 RMSE 0.0193 m、最大误差 0.0384 m、实际接受 106 个 RTK 因子、无创新量拒绝；全部规定点最近真值距离均小于 0.75 m，其中转弯点约 0.69 m。实时率约 0.88。这些定位误差不是路径跟踪误差，也不证明无围栏的特征稀疏农田具有同等性能。

Car3 履带车随后独立完成同一条五点全路线，使用 MPPI DiffDrive 运动模型，记录为 `20261016_strawberry_tracked_route`：水平定位 RMSE 0.0196 m、最大误差 0.0349 m、112 个 RTK 因子、无创新量拒绝，规定点最近真值距离最大 0.37 m，实时率约 0.90。两次测试都检查了全路径先生成再运动、动态相机图像、非空点云、定位与真值时间配对以及进程正常退出；没有并行运行两种定位计算。

## 悬挂式中耕机与轴荷检查

本轮继续审查目录中的骑乘割草机、手扶旋耕机、机械式播种机和悬挂式中耕机，四项转换记录在 `~/.local/share/agribot/farm_assets/intake_tools_20261016/`。前两项不是可直接挂到拖拉机后的农具；播种机参考图有牵引环和独立轮轴，转换网格仍有装配疑点，需后续实现铰接拖挂。未把这些候选当成已验证安装资源。

中耕机使用 `mounted_cultivator_cad.json`，源 STEP SHA-256 为 `22039ccd70823bab4b71b8e794549f6bc88499959658b26668216ceaee4f1b41`。转换后约长 2.27 m、宽 2.68 m、高 1.13 m，64933 面；保留原尺寸，不根据压缩包中的“3m”字样缩放模型。最终标准化资源为 `mounted_cultivator/ros_v2`，Car7 挂接资源为 `mounted/cab_cultivator_v2`，车辆 ID 为 `cab_cultivator`。名义质量 180 kg，离地 0.15 m；不实现入土、土壤受力或圆盘旋转。

最初 Car6 挂接测试失败，生成 SDF 的合成重心 x 为 -0.0314 m，在后轴之后，前轮卸载并严重转向不足。未削减质量或缩小网格以通过测试，改配 Car7；组合质量 2170 kg、重心 x 约 0.8556 m、水平地面名义前轴载荷比例约 28.2%。`mounts.transport_balance` 从实际生成的 SDF 惯性参数检查静态纵向轴荷，拒绝重心落在前后轴支承区间之外的组合。报告写入导出配置，但不表示完成真实承载、坡地稳定或侧翻验收。

Car7 首次复测定位稳定、转向执行正常，但在目标处朝向未收敛，车辆越过终点后控制器中止。Humble `GoalAngleCritic` 只在距离目标小于 `threshold_to_consider` 时评分；原 0.5 m 窗口对于约 5.56 m 转弯半径过短。现在仅对 `inspected_local_CAD_profile` 名义仿真车型，将其介入距离设为 `max(原值, 0.5 * 最小转弯半径)`，单位是米；不是修改航向容差。原六车、真机、差速控制器和 0.25 m/0.2 rad 到达阈值不变。机制核对来源：[Nav2 Humble GoalAngleCritic](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/critics/goal_angle_critic.cpp)。

| 记录目录后缀 | 测试 | 结果 |
| --- | --- | --- |
| `tractor_cultivator` | Car6，玉米田，8 m 前进及 1.5 m 横移 | 失败：前轮卸载、目标越过；Gazebo 退出被强制终止 |
| `cab_cultivator_arable` | Car7，同一目标，原终点角度介入窗口 | 失败：终点朝向收敛过晚，定位 RMSE 0.0226 m |
| `cab_cultivator_heading` | Car7，同一目标，提前终点角度修正 | 通过，定位 RMSE 0.0192 m，最大 0.0459 m，23 个 RTK 因子，无创新量拒绝，实时率约 0.95 |
| `cultivator_groove_regression` | Car7 开沟机，农场运输区，相同弯线回归 | 通过，定位 RMSE 0.0181 m，实时率约 0.93 |
| `cultivator_compact_regression` | Car6 圆盘耙，农场运输区，相同弯线回归 | 通过，定位 RMSE 0.0105 m，实时率约 0.93 |

完整记录目录前缀均为 `~/.local/state/agribot-algorithm-validation/20261016_`。定位误差不等于路径跟踪误差；中耕机通过记录中规定目标的最近真值距离约 0.42 m，采用原有 0.75 m 规定点邻域验收。测试没有并行运行不同定位方法。这仍是短程运输与挂载验证，不是中耕作业或全场景全路线验收。

Unity 编辑器检查 16 项、实际安装与滚动交互检查 7 项通过。原装备面板的第三排保留圆盘耙、六行开沟机和悬挂式中耕机；修正滚动条被原面板裁剪、隐藏的问题，不另建页面。桌面程序实测可通过滚轮显示最后一排，原有机械臂与凿式犁保留。构建为 `~/Builds/AgriculturalCar_Interior_Linux_20261016_cultivator`，桌面入口同步更新，旧构建保留。截图、测试 XML、构建日志和发布记录位于 `~/.local/state/agribot-algorithm-validation/20261016_unity_cultivator/`。

## 葡萄园扩展

`vineyard` 保留 CropCraft 四种葡萄藤原尺度，三行各 11 株，株距 2 m、行距 6 m；另生成 15 根立柱和两层架线。地面约 44 x 30 m，场景视觉网格约 6.96 万面，纹理最大边长 1024。碰撞几何生成同源二维地图，投影高度带为 0.18～2.2 m。这里只验证静态作物环境下的行驶，不模拟葡萄采摘或植物受力。

出生点为 `(-15,-3,0)`。路线保留最初五点 `(-4,-3,0)`、`(12,-3,0)`、`(15,0,pi/2)`、`(12,3,pi)`、`(-12,3,pi)`，并在半径 3 m 的地头半圆上增加两个 45 度切点及相应切线航向，共七点。未改原苹果园 23 点、现有车辆参数或 0.75 m 规定点邻域验收限。

第一轮五点路线实际到达终点，但地头目标的最近真值距离为 0.7518 m，因此 `20261016_vineyard_car0` 如实保留失败。增加两个切点后，以软件同一预设任务执行器依次验证：

| 记录后缀 | 车型 | 规定点 | 最大规定点偏差 | 水平定位 RMSE / 最大误差 | RTK 因子 | 实时率 |
| --- | --- | --- | --- | --- | --- | --- |
| `vineyard_tangent_route` | Car0 阿克曼 | 7/7 | 0.479 m | 0.0377 / 0.1990 m | 171 | 0.883 |
| `vineyard_tracked_route` | Car3 履带 | 7/7 | 0.543 m | 0.0247 / 0.0862 m | 175 | 0.908 |

记录目录前缀为 `~/.local/state/agribot-algorithm-validation/20261016_`。两次都使用 FAST-LIVO2+RTK、Smac 和 MPPI，履带车采用 DiffDrive 控制模型；确认先生成完整路径再运动、相机非空、点云非空、RTK 因子实际接受且无创新量拒绝、进程正常退出。定位误差与跟踪误差分别记录，不能将这些结果解释成全部车辆或全部算法组合的验收。Gazebo 场景实图在阿克曼通过记录的 `gazebo_scene.png`，该图片只用于几何与材质检查。

复测命令：

```bash
source /opt/ros/humble/setup.bash
source ~/agribot_ws/ros2_ws/install/setup.bash
/usr/bin/python3 ~/agribot_ws/ros2_ws/src/agribot_vehicle_description/scripts/verify_algorithm_runtime.py \
  --case stvl_off_fastlivo_rtk_smac_hybrid_mppi \
  --vehicle-id ackermann_current --scenario vineyard --scope full-route \
  --domain 96 --startup-timeout 180 --motion-timeout 2000 --gui
```

打包后的新版位于 `~/Builds/AgriculturalCar_Interior_Linux_20261016_vineyard/`，桌面“农机配置平台”入口已更新。保留旧构建及用户正在使用的旧窗口。16 项编辑器检查、7 项 PlayMode 交互检查通过；另在独立运行程序中确认葡萄园可选、原装备面板能够滚动显示全部机具、履带车不兼容的中耕机安装按钮禁用。截图与测试日志保存在 `~/.local/state/agribot-algorithm-validation/20261016_unity_vineyard/`。ROS 资源检查 44 项和算法矩阵/启动入口检查 30 项通过。

同期补充的 CAD 转换位于 `~/.local/share/agribot/farm_assets/intake_more_20261016/`。`tractor-assembly-2` 的 SHA 与现有 Car6 完全相同，不计作新车型。粉碎机初版轻量网格的大支架失真，未加入安装项；`prepare_mesh.py` 增加可选的大结构件保面数下限，默认行为不变。独立的 `lod_structural` 版本保留 48 个大结构件的面数下限，将 392 万面压至约 11.2 万面，正视预览中的大支架比初版完整，但安装轴、挂接位置、颜色与仿真仍待验证，不能算作可安装机具。行间铣土机和水稻种植机只完成初始几何转换，安装和运动能力尚未验收。下载、转换和入库验证是分开的状态。

## 林地粉碎机挂接

上述粉碎机随后完成了挂接验证。`forest_chain_crusher_cad.json` 保留原外廓约 `1.125 x 2.846 x 1.293 m` 和 112052 个三角面，将源 CAD 的 `+Y` 上方、`+Z` 朝拖拉机方向转换到 ROS FLU。源 STEP 不带参考图的涂装，保留中性材质，不声称复现厂家配色。预览保留大支架、罩壳、侧传动壳和链条。

在原车库 Car7 的装备面板内新增“悬挂式林地粉碎机”，与机械臂和其他农具同一列表，向下滚动可见。固定安装在车后方，保留约 0.15 m 地面间隙；与其他后悬挂机具互斥，可卸载。Car0～Car6 不兼容的安装按钮禁用，未新建页面或改变已有车型安装点。导出 ID 为 `cab_crusher`，Unity 与 ROS 共享同一挂接配置与网格。质量 350 kg 是明确标记的名义仿真值，静态前轴载荷比例约 23%，不代表真实举升能力。

| 记录后缀 | 场景与目标 | 实际位移 | 水平定位 RMSE / 最大误差 | 实时率 | 结果 |
| --- | --- | --- | --- | --- | --- |
| `cab_crusher_transport` | 玉米试验田，前方 8 m、左偏 1.5 m | 7.928 m | 0.0263 / 0.0719 m | 0.947 | 通过 |
| `cab_crusher_right_turn` | 玉米试验田，前方 12 m、右偏 2 m | 未执行 | 不作定位验收 | 不作验收 | Smac 未找到可行路径，失败记录保留 |
| `cab_crusher_farmyard` | 农场运输区，前方 12 m、右偏 2 m | 11.944 m | 0.0228 / 0.0765 m | 0.924 | 通过 |

目录前缀为 `~/.local/state/agribot-algorithm-validation/20261016_`。通过记录使用 FAST-LIVO2+RTK、Smac 和 MPPI，分别接受 24/35 个 RTK 固定解因子，创新量拒绝均为 0，先规划后运动，未出现异常进程退出。这些是运输短程测试，不是该机具在全部场景的全程验收，更不代表实现了 PTO、转子或枝条粉碎物理过程；未修改碰撞包络来绕过失败目标。

本轮 ROS 资源和机具生成检查 62 项通过，Unity 编辑器 18 项及 PlayMode 交互 7 项通过。独立程序中另行检查了末排滚动、实际安装、卸载入口和侧后方模型显示。新版为 `~/Builds/AgriculturalCar_Interior_Linux_20261016_crusher/`，桌面入口已更新，旧构建保留。证据放在 `~/.local/state/agribot-algorithm-validation/20261016_unity_crusher/`，未提交 Git。

行间铣土机新生成 `intake_more_20261016/70de5d21675bb06e/lod_structural/`，约 14.92 万面，保留四行机构和料箱，已与源参考图对照；源坐标、挂接位置和仿真仍未验收，不加入可安装列表。

## 四行铣土机与转向限位

上述候选随后按 `inter_row_milling_cutter_cad.json` 转为 ROS FLU：源 `+Y` 向上、`-Z` 朝牵引端，保留约 `1.812 x 3.000 x 1.868 m` 的外廓和 149215 个三角面，不整体缩放。资源为 `inter_row_milling_cutter/ros_v1`，Car7 组合为 `mounted/cab_interrow_v1`，导出 ID 为 `cab_interrow`。四行机构与料箱保留中性 CAD 材质，未宣称复现参考照片涂装。

新增项仍在原装备面板的滚动列表中，与其他后悬挂机具互斥、固定挂接。不支持的车辆禁用安装按钮。名义机具质量 450 kg，组合质量 2440 kg，前轴静态载荷比例约 17.4%，运输离地 0.15 m；这不是实际承载或液压举升验收，也未模拟转子、播种、入土和土壤动力学。

检查发现，新 CAD 车型把虚拟自行车转角与左右实际转向关节限位都设成同一个值。现在仅对 `inspected_local_CAD_profile` 且 `unity_nominal_simulation_only` 的阿克曼车型，使用 `atan2(L, L/tan(delta)-T/2)` 推导允许的内侧轮角度，与驱动插件分配角度的公式一致。SDF、URDF 使用相同限位；插件仍限制虚拟转角 `delta`，规划器与控制器的最小半径不放宽。原六车和真机配置不变。

该几何修正不能解决所有路线跟踪问题，测试结果必须分别保留：

| 记录后缀 | 场景与目标 | 结果 |
| --- | --- | --- |
| `cab_interrow_transport` | 玉米田，前方 8 m、左偏 1.5 m | 短程通过；实际位移 7.903 m，定位 RMSE 0.0226 m，最大 0.0578 m，实时率 0.947 |
| `cab_interrow_jointlimit` | 农场，前方 12 m、右偏 2 m，Smac 规划约 49.5 m 绕行 | MPPI 失败；约行驶 10.25 m 后停止，定位 RMSE 0.0193 m，不计作通过 |
| `cab_interrow_jointlimit_rpp` | 同一绕行目标，RPP 对照 | 检测前方碰撞后停止，不计作通过 |
| `interrow_crusher_regression` | 原 Car7 粉碎机组合，农场前方 12 m、右偏 2 m | 回归通过；实际位移 11.876 m，定位 RMSE 0.0153 m，实时率 0.931 |
| `interrow_compact_regression` | 原 Car6 圆盘耙组合，农场前方 8 m、左偏 1.5 m | 回归通过；实际位移 7.929 m，定位 RMSE 0.0092 m，实时率 0.933 |

目录前缀为 `~/.local/state/agribot-algorithm-validation/20261016_`。四行铣土机短程通过记录接受 24 个 RTK 因子，创新量拒绝为 0，先规划再运动且退出正常；规定目标最近真值距离约 0.306 m，不与定位 RMSE 混为一谈。早期 MPPI 时域/权重试调没有解决复杂绕行，未保留这些试调；没有缩小碰撞轮廓来绕过失败。当前状态是 `short_transport_verified_complex_turn_pending`，不宣称全路线、全场景通过。

ROS 资源、挂载、生成器及算法矩阵回归共 86 项通过，包含本地 CAD 实体检查；另有 2 项来源入库检查通过。JSON Schema 依赖仍输出原有 metaschema 警告。

Unity 编辑器 20 项、PlayMode 交互 7 项通过；独立程序检查了滚动末排、实际安装、卸载入口和侧后方模型显示。新构建为 `~/Builds/AgriculturalCar_Interior_Linux_20261016_interrow/`，桌面入口已更新，旧构建和用户已打开的旧窗口保留。证据位于 `~/.local/state/agribot-algorithm-validation/20261016_unity_interrow/`。未提交或推送 Git。

本轮另下载 Feldfreund Devkit 固定提交 `140413d7f947c3cf89510fc402c892da6af68c63`。仓库 MIT 许可与其中 Forest3D 模型资源的许可需分别核对；目前只登记为已检查来源，没有新增运行场景。水稻种植机仍处于几何候选阶段，不计入可驾驶车型。

## 蔬菜试验田

`vegetable_field` 使用 PIC4SeR 的洋蓟、西葫芦、莙荙菜和生菜模型，每类保留两种植株，共 158 株、约 59.1 万三角面。保持原米制尺度，单株减面后重新以根部落地；纹理上限 1024。四个分区、地面、巡检通道和带入口的周界围栏由本项目布置，不冒充上游原场景。静态作物不包含采收或土壤动力学。

路线从 `(-14,0,0)` 出发，包含两个半径 4.5 m 的地头掉头和通向 `(0,5,pi/2)` 的中央通道，共 16 个规定点。最初 12 点路线的终点位于出发直道上，Nav2 在首次接近终点时提前结束；验证器正确判为未经过全部规定点。原 12 点全部保留，另加 4 点将终点延伸到独立通道，未放宽点位检查或到达阈值，也未改动苹果园原 23 点。

| 记录后缀 | 车型 | 结果 |
| --- | --- | --- |
| `vegetables_car0` | Car0 | 旧终点布局提前结束，FAIL，保留记录 |
| `vegetables_car0_off_track_goal` | Car0 | 全部 16 点通过，真值采样累计路程约 91.64 m，规定点最近距离最大 0.328 m |
| `vegetables_car3` | Car3 | 完成两处掉头并到达终点附近，但末端位置/朝向未收敛；823.7 秒后人工中断，不能算通过 |

记录目录前缀为 `~/.local/state/agribot-algorithm-validation/20261016_`。Car0 使用 FAST-LIVO2+RTK、Smac Hybrid、MPPI，水平定位 RMSE 0.0273 m、最大误差 0.0529 m，接受 269 个 RTK 因子且无创新量拒绝；实时率 0.873，退出正常。已检查先生成完整路径再运动、相机非空、点云非空。路程、点位跟踪误差和定位 RMSE 分别统计，不互相代替。

Car3 使用 DiffDrive 控制模型，水平定位 RMSE 0.0227 m、最大误差 0.0560 m；终点收敛失败时定位仍与仿真真值接近。其原配置还报告膨胀半径 0.500 m 小于车体内切半径 0.523 m，完整车体轮廓检查仍开启。本轮不以修改原车控制参数或缩小轮廓来宣称场景通过，履带终点控制问题留待独立处理。

ROS 场景资源、挂载及算法入口回归共 115 项通过，原 JSON Schema 警告仍保留。Unity 编辑器 20 项、PlayMode 安装交互 7 项通过；独立运行程序确认原场景选择栏中的“蔬菜试验田”可见、可选，没有新增页面。测试、构建日志和截图位于 `~/.local/state/agribot-algorithm-validation/20261016_unity_vegetables/`。

最新构建为 `~/Builds/AgriculturalCar_Interior_Linux_20261016_vegetables/`，桌面“农机配置平台”指向该版本，旧构建与用户已打开的旧窗口保留。CAD 资源仍只保存在本地忽略目录；没有提交或推送 Git。这一轮资源扩展不代表所有候选已能运行，尤其 Car3 的蔬菜场景终点收敛和 Car7 四行铣土机的复杂绕行仍未通过。

## 骑乘式割草机 Car8

### 当前运动学：受控前轮阿克曼

2026-10-21 按用户要求，将裸车与前置割刀组合均从后轮差速/自由脚轮改为后轮驱动、前轮受控转向。保留原车轮心、轮径、前叉网格、传感器安装位姿；前叉绕原安装轴受控回转，不再生成无限旋转脚轮关节。`steeringAssembly` 记录前叉转轴与网格，SDF 与 URDF 导出均保留该结构。

轴距 1.246889 m，中心最大转角 0.5 rad，最小转弯半径 2.282415 m；默认速度 0.3 m/s，对应最大角速度约 0.131440 rad/s。Smac Hybrid 和 MPPI Ackermann 使用同一半径；不支持差速原地旋转。当前本地规范源为 `ride_on_mower/ros_v6_ackermann`、`mounted/mower_front_cutter_v2_ackermann`。这是用户要求的名义仿真结构转换，不是厂家机构或真机标定声明。

旧差速源和下文旧版测试记录保留，仅描述旧结构，不作为新阿克曼结构的全路线通过证据。新结构须重新进行人工场景验证。

### 原差速脚轮版本历史

2026-10-19 新增 `ride_on_mower`，来自本地 CAD 集中的骑乘式割草机，源 STEP SHA-256 为 `b3e065b96e7715eca6199dffa1eafa4df046e92939c8280b8821adfaa5a8376d`。配置见 `config/ride_on_mower_cad.json` 和 `config/ride_on_mower_sensors.json`；规范资源为 `~/.local/share/agribot/farm_assets/ride_on_mower/ros_v5/`。原始模型及转换网格仍在本地忽略目录，不进入 Git，也未确认允许对外再分发。

保留原尺寸，外廓约 `1.796 x 1.602 x 1.184 m`；基准点为后驱动轴中点，后轮半径约 0.253 m，前轮约 0.140 m。前部脚轮的轮架和轮胎单独拆分，轮架绕竖直轴被动回转，只有左右后轮受差速驱动。约 93.2 万面的原 CAD 轻量化后，ROS 车身与四轮约 3.76 万面；完整 Unity 预览含轮架和既有高精度传感器约 27.9 万面。红灰材质为展示配色，不声称是厂家涂装。

沿用原车库、原装备面板和安装/卸载交互，复用现有 IMU、C16、相机和双 RTK 总成；相机在雷达上方，IMU 在前部平台，RTK 位于平台两侧。安装点与导出坐标一致，预览没有重新缩放。不适合该车的农具禁用安装按钮。新车使用名义质量 180 kg、默认速度 0.3 m/s、最大角速度 0.4 rad/s；规划曲率半径 0.75 m 是保守的仿真参数，不代表差速底盘不能原地转向。它用于车辆导航仿真，不包含刀盘动力学、草料切割、土壤作用或真机标定。

使用 FAST-LIVO2+RTK、Smac Hybrid 和 MPPI DiffDrive，已完成以下运行检查，记录目录前缀为 `~/.local/state/agribot-algorithm-validation/20261019_`：

| 记录后缀 | 场景 | 结果 |
| --- | --- | --- |
| `mower_maneuver`、`mower_maneuver_retry` | 玉米田短程弯线 | 运动通过；RViz 在退出时断言失败，整体记录保留为 FAIL，不算完整通过 |
| `mower_tomato_route` | 番茄温室 3 个规定点 | 全程 PASS，位移 11.786 m，水平定位 RMSE 0.0094 m，实时率 0.848 |
| `mower_vegetable_route` | 蔬菜试验田 16 个规定点 | 车辆完成路线，但旧验收器将第一次点位匹配到返程，原 FAIL 记录保留 |
| `mower_vegetable_route_ordered` | 同一蔬菜路线重新完整运行 | 全程 PASS，累计真值路程 92.04 m，完成两次掉头，按序点位误差最大 0.280 m，水平定位 RMSE 0.0283 m，实时率 0.885，退出正常 |

验收器改用顺序约束下的最小最大误差匹配，避免贪心最近点跳到后一次经过的位置；仍要求完整 16 点、邻域半径仍为 0.75 m，未修改路线或放宽标准。增加重复点往返、逆序、漏点和穷举对照检查。最后一次完整运行先规划后移动，图像与点云非空，接受 272 个 RTK 固定解因子，创新量拒绝为 0。不把此结果推广为所有算法组合、所有场景均通过。

Unity 编辑器 22 项、PlayMode 8 项通过，另有 1 项视角回归通过；ROS 三包构建成功。全目录 ROS 检查为 213 项通过、13 项跳过、2 项失败：原 Car0 外参测试硬编码 2 微米容差，实际 Pcl 与期望相差约 2.38 微米，已用 Git HEAD 原始函数和配置复现相同结果。本轮未改动该真机外参或放宽测试。

新版程序 `~/Builds/AgriculturalCar_Interior_Linux_20261019_mower/`，桌面“农机配置平台”指向该版本；进入原车库向下滚动可见 Car8。旧版本和用户其他已开启窗口保留。本轮没有提交或推送 Git。

## 原拖拉机发动机盖安装修复

Car1、Car2、Car4、Car5 的名义米制版本由 `prepare_scene_tractors.py` 生成，安装设计保存在该脚本的 `SENSOR_MOUNTS` 中。前部雷达与相机移到发动机盖，不再依据整车最高包络放到方向盘上方；原始 Unity 车型文件不改，输出到独立的 `original_tractors_metric_v5` 本地资源目录。传感器本体尺度不变，IMU、RTK 杆臂和定位外参随安装坐标重新生成。

`mesh_surface.py` 对指定车壳三角面求竖直交点，生成贴合车壳的支架脚；不再用附近顶点最大值代替落脚点。`check_camera_clearance.py --check-lidar --check-contacts` 在 Blender 中独立检查实际接触点与相机、雷达前向视线。Car4 保留高于进气筒的合理支架高度，避免降低支架后挡住视线。它们是 CAD 仿真设计，不作为真车外参标定结果。

四车型各安装一个机具，在启用 Gazebo GUI 和 RViz 的情况下完成番茄温室原 3 点路线。见 `docs/farm_compatibility_results_20261019.md` 的最新进展；其他场景仍需逐一验证，不能以本轮直行测试推断所有组合可用。
