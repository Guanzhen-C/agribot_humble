# Agribot vehicle configuration pipeline

This package is the ROS side of the Unity-to-simulation-to-physical-vehicle
workflow. `config/vehicles` stores the reviewed canonical vehicle contract.
`generated` stores deterministic ROS files and must only be updated by
`agribot_vehicle_config`.

The current Ackermann contract uses ROS FLU coordinates and defines
`base_link` at the rear-axle midpoint. Unity authors in its native axes; the
Unity exporter performs the coordinate conversion. Do not manually swap axes
after export.

## 1. Configure and export in Unity

Open `AgriculturalCar_Interior/Assets/Scenes/scene1.unity` on the 218 workstation.
Enter Play mode, finish the vehicle and sensor setup in the Game configuration
interface, then click `启动仿真`. The runtime exporter builds or refreshes
the active vehicle configuration from the current scene automatically; the Editor
initialization menu is not required for this normal workflow.

Car0 opens `scene2` and retains the calibrated Ackermann configuration and
the existing six action buttons. Car1, Car2, Car4 and Car5 use their own
Ackermann geometry; Car3 uses four-wheel differential contact dynamics with
animated visual rollers and tread segments. Install
all four sensor types, wait for the installation animation to finish, and use
the shared algorithm selector before exporting. Stop the managed simulation
with `停止仿真` before switching vehicles.

Car3's configured Gazebo runtime uses `tracked_contact_simulation.yaml` to
align lateral friction with each cylinder axle and preserve longitudinal
traction. The current `car3_visible_idler_centers_v1` profile places each contact
center at the corresponding front/rear visible idler: x approximately +0.44894
and -0.38806 m, y +0.30368 and -0.29751 m, z -0.10694 m. Wheelbase is 0.83700 m,
track width 0.60118 m; contact radius 0.11815 m includes the outer tread envelope.
Canonical JSON, SDF, URDF, drive diameters, animation ratios and Unity exports
use the same geometry. Body/base reference and sensor poses are unchanged.
The current runtime contact profile uses lateral friction 0.36 at both axles,
longitudinal friction 1.8, and the official Gazebo `WheelSlipPlugin` with lateral
compliance 0.1 and longitudinal compliance 0 when stopped or driving straight.
The local `SimulationTrackedWheelSlip` adapter subclasses the official plugin
without modifying Gazebo sources. It uses the measured left/right wheel-speed
difference: 0.02--0.08 m/s progressively enables turning slip up to 0.003 m/s/N,
so low-speed skid turns retain their maneuverability. Wheel normal force is
estimated from total model mass and four contact wheels. ODE slip starts at
zero; straight-motion slip is scaled by wheel speed rather than retaining the
old constant slip1=0.01 that caused lateral creep on farmyard slopes.
This runtime policy applies across all scenes, not
only the greenhouse. Legacy short-roller exports retain their old 0.36/0.60 coefficients.
This is still a four-contact approximation, not a physically measured track/soil
model or a guarantee of zero slip. Previous manual results belong to their
original configuration hashes and do not validate this longer-wheelbase model.

The same adaptive-slip runtime is enabled for Car9's `tracked_harvester` and
`harvester_header` presets. Their larger mass and different wheel radii are
used for the load estimate; turning slip remains 1e-6 m/s/N, not Car3's 0.003.
The existing front/rear lateral-friction ratios are preserved while scaling
both by 2 (rear 0.2, front approximately 0.15301 bare / 0.05699 with header)
to improve gentle cross-slope grip. Longitudinal friction stays 1.8. Geometry,
wheel animation, mass, drive plugin, navigation and physical export are not
changed. These are simulation contact settings, not a full-route acceptance
or measured harvester/soil calibration.

### Final-position stopping in configured simulations

All configured vehicles and MPPI/RPP/DWB profiles use Nav2's
`PositionGoalChecker` with an XY tolerance equal to half the vehicle's
`geometry.wheelbaseM`. Reaching the final position
ends the task and publishes zero velocity; final yaw is not an acceptance
condition. MPPI's terminal angle critic and DWB's terminal rotation critic are
disabled. Pose headings still guide the collision-checked global route.
Intermediate through-poses goals remain ordered, and the preset continuous
route retains its existing progress guard.

This is a shared rule for Ackermann, differential and tracked vehicles,
including future Unity imports; it is not a per-model allowlist. New vehicles
must provide their own positive, finite front-to-rear `geometry.wheelbaseM`.
Track width and implement length are not used to compute this tolerance, and
there is no fixed minimum or maximum overriding half the wheelbase.

This tolerance changes only final-position acceptance, not intermediate goal
removal, planner search tolerance, localization, footprint or clearance.

Configured simulations do not periodically replan global paths. Dedicated
simulation behavior trees replace the global-planner
`RateController` with Nav2's `GoalUpdatedController`: the initial route is
planned immediately, and a changed request or a reset by failure recovery can
plan again. All existing recovery subtrees and their retry limits remain.
MPPI/local obstacle updates retain their existing rates. Physical-vehicle and
shared XML files are unchanged; explicit custom BT files remain under the
caller's control. With no periodic planner tick, through-poses feedback may
retain the initial remaining-goal count between recovery/goal updates; the
full ordered path is still sent to the controller.

Native Nav2 BT nodes and plan topics are used. The temporary force-finish BT
adapter and ROS path relay have been removed. The RViz-only NavigationPath
display clears task paths after completion without changing backend messages.

Navigation uses the host body plus its existing clearance, without implements
or arms. Temporary SDF/URDF tool descendants have no collision geometry;
visuals and inertia remain, as do host body/wheel contact and hard obstacle
checks. Trailer wheel contacts are also excluded by this policy, so this is
not validation of implement clearance or realistic trailer/soil dynamics.
Canonical exports and physical deployment templates are unchanged.

The export is written to:

```text
AgriculturalCar_Interior/ConfigExports/ackermann_current/
  vehicle_config.json
  vehicle_visual.obj
  vehicle_visual.mtl
```

New vehicles use separate `ConfigExports/unity_car1` through `unity_car5`
directories, canonical JSON and generated ROS bundles. They never overwrite
`ackermann_current`. Wheel dimensions and sensor mounts come from the active
scene. Their 35-degree steering default, mass and dynamics are nominal
simulation settings, not physical calibration. Only the original validated
Ackermann preset is eligible for physical deployment export.

Sensor meshes are excluded from the exported body mesh even when their CAD
roots differ from measurement anchors. New vehicles export separate IMU,
LiDAR, camera and dual-antenna assembly OBJ/MTL pairs, expressed in their
measurement-anchor frames. Gazebo and RViz display these CAD meshes instead
of generic sensor placeholders. The Gazebo visual plugin makes sensor CAD
visible to the GUI but not to the simulated camera, so the camera does not
see the inside of its own housing. Missing material assignments use neutral
gray instead of inheriting the previous submesh material. New vehicles use an
ODE collision-category plugin to exclude their simplified chassis and wheel
proxies from laser rays. This avoids a whole-CAD box hiding the environment
when the sensor lies in empty space inside that box. Ground friction and
physical obstacle contact remain enabled; world obstacles still return rays.
This is a nominal simulation approximation, not an accurate self-occlusion
model for physical sensor validation. It does not apply to Car0.
New-vehicle LiDAR self-returns are
removed in the exported base-frame bounding box before localization and
obstacle processing; point fields and acquisition timestamps are preserved.

On the 218 simulation workstation, the Game export button also starts
`scripts/unity_sync_and_simulate.sh`. The script validates and imports this
bundle, performs an incremental build, stops only the previous simulation it
started, and launches the configured Gazebo/RViz entry point. The default ROS
domain is isolated from physical vehicles (`ROS_DOMAIN_ID=37` and
`ROS_LOCALHOST_ONLY=1`).

Normal startup does not dispatch an authored route. Unity explicitly passes
`--run-waypoints false`; both the shell script and configured ROS launch also
default to manual goals. The vehicle remains stationary until the user sends
a navigation task. The default RViz layout includes the official Navigation 2
panel and Nav2 Goal tool; this selection interface belongs to RViz, not Gazebo.

To collect multiple poses, first click `Waypoint / Nav Through Poses Mode` in
the Navigation 2 panel. Use `Nav2 Goal` to click and drag each position and
heading, then click `Start Nav Through Poses` to execute. A single goal in the
normal mode starts navigation immediately. `Start Waypoint Following` is the
separate waypoint-task workflow, not the continuous-through-poses command.

Preset routes, including the original 23-point orchard route, remain available
only for explicit automated validation with `--run-waypoints true` or
`run_waypoints:=true`. Their historical validation records are not evidence
that an arbitrary user-selected route is feasible for every vehicle.

### Current simulation implement policy

At the user's request, configured simulation navigation now uses the host
vehicle's body extents and existing clearance only. Implements retain their
visuals and inertia, but their SDF/URDF contact geometries are removed in
temporary runtime copies, including descendant links. This is a visual-only
implement collision policy, not validation of implement clearance, trailer
ground support or agricultural contact dynamics. Body and vehicle-wheel
collisions, sensor mounts and implement self-return filtering remain active.
Canonical export files and physical deployment profiles are not rewritten.

`nominal_ackermann_navigation.yaml` adjusts large nominal Ackermann simulations:
a 6 s MPPI horizon, earlier path-angle scoring, and lower soft clearance costs;
Smac uses the original 5 cm grid and lower turn/traversal penalties. Actual
speed, steering and minimum-radius limits and hard collision costs are retained.
Car0's established MPPI overlay and physical parameters are unchanged. Historical
full-implement-footprint runtime evidence does not certify this new policy.

Before export, the Unity interface presents the available algorithm library.
Catalog/configuration validation is not evidence that every combination has
completed a closed-loop route. See the sequential runtime acceptance below.
The selected profile is passed as one immutable ID to the ROS launcher:

| Profile ID | Localization | Planner | Controller |
| --- | --- | --- | --- |
| `fastlivo_rtk_mppi` | FAST-LIVO2 + fixed RTK | Smac Hybrid-A* | MPPI |
| `fastlio_mppi` | FAST-LIO2 | Smac Hybrid-A* | MPPI |
| `navsat_mppi` | NavSat ESKF | Smac Hybrid-A* | MPPI |
| `fastlio_rpp` | FAST-LIO2 | Smac Hybrid-A* | Regulated Pure Pursuit |
| `navsat_rpp` | NavSat ESKF | Smac Hybrid-A* | Regulated Pure Pursuit |

`fastlivo_rtk_mppi` remains the default. RPP configurations are derived at
launch time from the same Unity-generated footprint, speed and minimum turning
radius as MPPI, so selecting a controller never falls back to stale geometry.

### Differential and tracked controllers

Car3 retains the same MPPI, RPP and DWB choices in Unity. The launcher detects
the exported `DiffDrive` profile and applies `config/differential_controllers.yaml`:

- MPPI uses DiffDrive without Ackermann curvature constraints, conservative
  angular sampling and terminal-heading scoring only within 0.25 m of the goal.
- RPP uses differential heading rotation, a 0.5 m minimum lookahead and the
  exported speed limit. When reversing is enabled, heading rotation is disabled
  because Humble RPP cannot use both modes together.
- DWB uses the standard differential trajectory generator, zero lateral
  velocity and RotateToGoal rather than the Ackermann trajectory generator.

The differential goal checker checks both position and heading. Footprint,
clearance, speed limits and collision checks remain enabled. Ackermann vehicles
continue to use their original overlays. This does not replace Smac Hybrid with
State Lattice, change the physical differential stack, or validate every
planner/controller combination. See
`docs/unity_tracked_controller_validation_20261013.md` for the measured scope.

The default paths can be overridden before starting Unity:

```bash
export AGRIBOT_ROS_WORKSPACE=/home/cgz/agribot_ws/ros2_ws
export AGRIBOT_ROS_SYNC_SCRIPT=/home/cgz/agribot_ws/ros2_ws/src/agribot_vehicle_description/scripts/unity_sync_and_simulate.sh
```

List or select the same profiles from a terminal:

```bash
src/agribot_vehicle_description/scripts/unity_sync_and_simulate.sh --list-algorithms
src/agribot_vehicle_description/scripts/unity_sync_and_simulate.sh \
  --export-dir /path/to/ackermann_current \
  --algorithm fastlio_rpp
```

Stop the Unity-managed simulation without touching other ROS processes:

```bash
src/agribot_vehicle_description/scripts/unity_sync_and_simulate.sh --stop
```

## 2. Import into ROS

Copy that directory to the ROS machine, then run from `ros2_ws`:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select agribot_vehicle_config_tools agribot_vehicle_description
source install/setup.bash
ros2 run agribot_vehicle_config_tools agribot_vehicle_config \
  import-unity /path/to/ackermann_current \
  --canonical src/agribot_vehicle_description/config/vehicles/ackermann_current/vehicle_config.json \
  --workspace-src src \
  --output src/agribot_vehicle_description/generated/ackermann_current
```

The generated physical files retain the existing tuned MPPI, Smac,
FAST-LIVO2 and synchronization settings. Only geometry, motion limits and
sensor-derived transforms are replaced.

The original high-detail OBJ remains in the Unity export directory. During
ROS import, the body and four wheels are converted to material-preserving
visual LOD meshes using 20 mm and 10 mm clustering grids respectively. Only
Gazebo/RViz visuals are reduced; collision primitives, joint axes, wheel
centres and sensor transforms continue to use the exact exported values.

Compare an export with the currently validated production configuration:

```bash
ros2 run agribot_vehicle_config_tools agribot_vehicle_config \
  verify-current \
  /path/to/ackermann_current/vehicle_config.json \
  --workspace-src src
```

## 3. Simulation acceptance

Rebuild after import. The configured entry point uses the exported model,
generated SDF/URDF, generated Nav2 geometry and generated FAST-LIVO2
extrinsics. It does not alter the legacy simulation entry points.

```bash
colcon build --symlink-install --packages-up-to agribot_vehicle_description
source install/setup.bash
export ROS_DOMAIN_ID=37
export ROS_LOCALHOST_ONLY=1
ros2 launch agribot_vehicle_description configured_ackermann_sim.launch.py \
  gui:=false rviz:=true
```

Add `vehicle_id:=unity_car1` (or another exported ID) to select a new vehicle.
The default remains `ackermann_current`. Import and build its bundle first;
the launcher does not substitute another vehicle when a bundle is missing.

Validate TF, sensor topics, steering, wheel motion, footprint and planned path
before moving to the physical vehicle.

### Sequential runtime acceptance

The configured simulation defaults to Cyclone DDS to avoid the observed
Fast DDS lifecycle-response startup race. Install `ros-humble-rmw-cyclonedds-cpp`
through rosdep or apt before building. This is launch-scoped; physical launch
commands and global shell settings are unchanged. An existing
`RMW_IMPLEMENTATION` is respected; use `rmw_implementation:=rmw_fastrtps_cpp`
or the test runner's `--rmw rmw_fastrtps_cpp` for an explicit comparison.

Build and source this workspace, then use an isolated domain. This test never
starts a physical chassis node. Each case starts its own Gazebo process, waits
for localization and active Nav2 servers, plans through all 23 authored route
points, and follows the initial 2 m of that plan. It stops its own session
before starting the next case. The three AI modes load their real model files
and must publish both annotated images and structured results.

```bash
ros2 run agribot_vehicle_description verify_algorithm_runtime.py --suite coverage
ros2 run agribot_vehicle_description verify_algorithm_runtime.py --suite navigation-matrix
ros2 run agribot_vehicle_description verify_algorithm_runtime.py --suite direct-matrix
ros2 run agribot_vehicle_description verify_algorithm_runtime.py --suite matrix
```

Use `--vehicle-id unity_car1` to validate the same runtime contract for a new
vehicle. Each vehicle requires its own results; passing the original vehicle
does not validate the new geometry, mounts or controller behavior.

`coverage` exercises every component branch in 14 cases; `navigation-matrix`
tests all 120 navigation combinations with the independent visual module off;
`matrix` tests all 480 combinations including visual inference. A smoke PASS
means the listed checks passed, not that a complete route or collision test
passed. Empty detections in an orchard do not prove recognition accuracy.

`direct-matrix` checks the 24 direct-route combinations using the installed
production waypoint runner's interpolation function and its default sampling.
This baseline does not provide obstacle-aware or curvature-constrained planning.

Complete-route execution must be requested separately, for example:

```bash
ros2 run agribot_vehicle_description verify_algorithm_runtime.py \
  --case stvl_off_fastlivo_rtk_smac_hybrid_mppi \
  --scope full-route --motion-timeout 4000
```

Results, paths, Gazebo truth/estimated-position traces and full logs are saved
under `~/.local/state/agribot-algorithm-validation/<timestamp>/`. The summary
records host details, source hashes, the working-tree state, every requested
case and its execution scope. Existing evidence directories cannot be
overwritten. A failure or timeout is never converted into PASS.

Every case creates a fresh ROS context and uses a Linux-safe DDS domain,
excluding physical domain 0 and Unity domain 37. `--reuse-domain` explicitly
tests repeated startup in the same domain. Abnormal process exits, including
shutdown crashes, fail acceptance even when the navigation action succeeded;
`execution_status` records that separate motion result.

The configured simulation filters obstacle marking points by height in
`base_link`, while clearing still uses the raw LiDAR cloud. This prevents
estimated world-frame height drift from promoting ground points to obstacles.
The raw cloud remains unchanged. The LIO-specific time adapter below is used
only for configured simulation localization. XY-only planners paired with MPPI use
position-only goal checking, since their paths do not constrain final heading.

2D planners do not natively impose Ackermann steering constraints. DWB now
uses `agribot_vehicle_description::AckermannTrajectoryGenerator`: it reuses
Nav2's DWB integration and collision critics but samples feasible curvature,
prevents stationary rotation and constrains acceleration steps to the exported
minimum turning radius. It does not add a steering-rate dynamics model or make
an infeasible 2D path feasible. Short-distance motion success is insufficient
to certify long-route kinematic feasibility. Collision avoidance, obstacle injection, localization
accuracy, resource limits and Unity export/start behavior require separate
acceptance cases before third-party delivery.

### 2026-10-03 runtime result

Third-party delivery is **NOT_READY**. On the x86_64 Ubuntu 22.04 / Humble test
host, 14 component-coverage cases passed. The sequential 120-case navigation
baseline had 96 strict passes and 24 failures: 12 DWB motion timeouts, one
planning failure, and 11 abnormal shutdowns after successful navigation.
The corrected production direct-route input was checked separately in 24
cases: 21 strict passes, one Gazebo startup crash and two shutdown failures.
These reruns do not overwrite or erase the earlier failures.

One 55 m default-combination run passed and visited two mandatory waypoints;
the complete 23-point route has not passed acceptance. Earlier planning
failures remain recorded. DWB and Gazebo stability failures were unresolved in that baseline,
and neither all 480 combinations nor clean-machine/Unity end-to-end operation
is certified. Unit tests passing is not proof that every runtime combination
is usable. Evidence and the acceptance report are under
`~/.local/state/agribot-algorithm-validation/`.

### 2026-10-04 fixes and regression scope

The DWB arrival failures were reproduced before editing. The Ackermann
trajectory generator above and its DWB scoring/horizon profile replace the
incompatible stationary-turn samples. The goal tolerance, 90-second motion
timeout and all 23 authored waypoint positions remain unchanged.

Gazebo shutdown backtraces identified a task drain after IO-thread teardown;
repeat runs also exposed `boost thread: trying joining itself`. A private
Gazebo 11.10.2 transport library drains connection tasks on the same IO arena
that queued them, before IOManager destruction. Draining from the main thread
alone did not resolve the cross-arena wait. The library retains the IO service
until its worker returns and avoids
joining that worker from itself. The public installed class layouts stay
unchanged. This library is loaded only by the configured Gazebo server through
a scoped launch environment. System libraries and physical-vehicle launches
are not replaced. Model spawning also starts with physics paused and unpauses
after insertion, reducing initialization/update races.

On Gazebo 11.10.2 with oneTBB 2021+, the normal package build fetches the
official, SHA-256-pinned source archive once (about 57 MB) and compiles only
the transport library. Subsequent builds use the build-directory cache.
The build switch `AGRIBOT_BUILD_GAZEBO_TRANSPORT_COMPAT=OFF` is available for
isolated comparison builds, not as a fix for the affected installation.
Other Gazebo versions do not load this version-specific library.
The upstream license and modification notices accompany the private build;
the package does not vendor the entire Gazebo source tree.

Unit tests include 100 releases from inside the IO callback, 100 normal
owner releases, and a single-worker test draining 100 queued tasks from their
own IO arena. AddressSanitizer and UndefinedBehaviorSanitizer checks passed.
Runtime acceptance checks the server's actual mapped library path and
snapshots both the compatibility binary and DWB plugin. The final 40-case DWB
regression passed every planning, sensor, motion and normal-exit check with
zero abnormal process exits (`20261004_dwb_matrix_final_v5/summary.json`).
The subsequent 19 MPPI/RPP, restart-stress and independent visual-inference
attempts also passed (`20261004_other_failures_final/summary.json`). These two
runs preceded the additional input-time and initial-anchor corrections below.
Earlier failures and
interrupted runs remain in their separate evidence directories. No complete
23-point route or full 480-case acceptance is implied by these smoke tests.

#### Long-distance initialization and LiDAR time correction

A 55 m NavFn/DWB extension exposed a separate FAST-LIO initialization error:
the vehicle settled to a nonzero yaw, but repeated initial-pose messages forced
its map pose to the nominal zero yaw. The resulting lateral offset grew with
travel distance and prevented a safe turn. Simulation ground truth now
publishes the actual full quaternion. The configured simulation initializes
FAST-LIO/KISS map anchors once from the settled spawn pose, using the existing
one-shot initializer. Subsequent ground-truth messages cannot update the
anchor, and movement remains estimated by the selected localization algorithm.
Original launches keep `use_settled_spawn_pose:=false` by default; physical
launches do not use this simulation initialization.

Gazebo's ray cloud is also an instantaneous sample, not a rotating scan.
Its zero per-point times previously triggered the LIO azimuth-time fallback,
inventing a scan duration and incorrect motion compensation. The simulation
adapter publishes `/configured/localization_points` with a logical window
start 0.1 s earlier and the same positive 0.1 s offset for every point:
`window_start + point_offset` remains the original sampling time. Point
coordinates, sensor mounts, raw `/lidar/points`, and physical inputs are not
modified. The bounded reliable output is compatible with FAST-LIVO2's reliable
subscription and FAST-LIO's best-effort subscription. A discovered QoS
mismatch and its failed runtime attempt remain in the evidence history.

With the corrected one-time anchor, the 55 m FAST-LIO/NavFn/DWB test completed
the first turn and passed both mandatory points in that prefix, with normal
process exits (`20261004_dwb_turning_spawn_anchor`). The latest-estimate versus
truth horizontal diagnostic RMS fell from about 0.46 m to 0.060 m; these
asynchronous diagnostics are not a hardware positioning-accuracy certificate.
The planned full route retained all 23 authored waypoints; only the 55 m
prefix was driven in this test. The subsequent 30-case regression passed all
27 previously failed combinations and three real visual-inference modes
(`20261004_failed_regression_final_v2`). This is short-motion acceptance, not
complete-route or all-480-combination certification.

An additional default-stack run exposed an intermittent startup yaw error:
the weak RTK-only yaw prior differed from the stationary Gazebo pose by 4.8
degrees. Projected obstacles narrowed the left U-turn and Smac correctly
rejected segment 4 before motion (`20261004_default_long_final`). Configured
FAST-LIVO2/RTK simulations now receive a synchronized known spawn pose once,
with the same strong initial orientation convention as the other simulated
localizers. The seed publisher removes both truth and local-odometry
subscriptions after publication; RTK and FAST-LIVO2 alone handle all later
motion. Neither physical initialization nor collision checking is changed.
Final regression after this additional fix passed all 30 cases
(`20261004_failed_regression_final_v3`), with unchanged runtime-file hashes,
normal exits for every launched process, and no forced termination. The
default FAST-LIVO2/RTK/Smac/MPPI 55 m prefix also passed both mandatory points
(`20261004_default_long_seed_fix`). All 36 unit checks passed. The complete
report is `~/.local/state/agribot-algorithm-validation/acceptance_report_20261004.md`
with machine-readable results in the adjacent JSON file. These results repair
the recorded failures; they do not certify all 480 combinations, the complete
23-waypoint drive, Unity GUI operation, or a clean third-party machine.

## 4. Physical-vehicle acceptance

### Configured Gazebo model browser

The configured simulation uses local scene and vehicle assets. Its `gzclient`
disables the online model catalog by default. Gazebo 11 starts detached Fuel
HTTPS catalog requests; on this system a pending request raced OpenSSL teardown
and crashed the GUI after an otherwise successful drive. A process-scoped
preload returns an empty catalog-server list, while local model loading and the
simulation server retain their normal behavior. No system library or user-wide
Gazebo configuration is replaced. `gazebo_online_model_catalog:=true` on the
underlying `ackermann_waypoint_depth_collect_sim.launch.py` restores upstream
online browsing for sessions that need it. Runtime verification records the
loaded protection library and still rejects abnormal process exits.

The configured physical entry point defaults to **no chassis output**:

```bash
ros2 launch agribot_vehicle_description configured_ackermann_physical.launch.py \
  map_base:=/absolute/path/to/map \
  rviz:=true \
  enable_chassis_output:=false
```

Only after localization, TF, costmaps and planned paths pass inspection should
the same launch be repeated with `enable_chassis_output:=true` at low speed.
The original production launch commands and their defaults remain unchanged.
## 停止与真机配置交付

Unity 提供“停止仿真”和“导出真机配置”按钮。停止操作只处理本脚本拥有的仿真会话，
不会停止真实车辆或无关 ROS 任务。构建期间收到停止请求会等待构建退出，并取消后续启动。
只有停止确认成功后，界面才允许导出；脚本也独立检查受管仿真是否已经退出。

```bash
~/agribot_ws/ros2_ws/src/agribot_vehicle_description/scripts/unity_sync_and_simulate.sh --stop
~/agribot_ws/ros2_ws/src/agribot_vehicle_description/scripts/unity_sync_and_simulate.sh \
  --export-physical ~/ackermann_physical_runtime.json
```

本次导出只包含当前已验证的真机参数模板和 STVL / FAST-LIVO2+RTK /
NDT-GICP / Smac Hybrid-A* / MPPI 标识，不携带任意仿真组合、模拟时间、地图路径或运动授权。
它不会证明某次仿真已完成完整路线，也不会把 Unity 中新改的未验证几何直接用于真车。
将 JSON 交给农机控制台“地图 → 运行配置 → 导入真机配置”；未导入时仍使用原程序默认配置。
