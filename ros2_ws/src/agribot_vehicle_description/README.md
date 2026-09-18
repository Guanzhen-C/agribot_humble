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

Open `AgriculturalCar_Interior/Assets/Scenes/main.unity` on the 218 workstation.
Enter Play mode, finish the vehicle and sensor setup in the Game configuration
interface, then click `导出 ROS 配置`. The runtime exporter builds or refreshes
the Ackermann configuration from the current scene automatically; the Editor
initialization menu is not required for this normal workflow.

The export is written to:

```text
AgriculturalCar_Interior/ConfigExports/ackermann_current/
  vehicle_config.json
  vehicle_visual.obj
  vehicle_visual.mtl
```

On the 218 simulation workstation, the Game export button also starts
`scripts/unity_sync_and_simulate.sh`. The script validates and imports this
bundle, performs an incremental build, stops only the previous simulation it
started, and launches the configured Gazebo/RViz entry point. The default ROS
domain is isolated from physical vehicles (`ROS_DOMAIN_ID=37` and
`ROS_LOCALHOST_ONLY=1`).

The simulation automatically loads the validated 23-point orchard route,
preplans one continuous Smac path through every point, and starts the selected
controller only after that path has been verified. Set
`AGRIBOT_SIM_RUN_WAYPOINTS=false` only when an interactive RViz goal test is
explicitly required.

Before export, the Unity interface presents the validated algorithm library.
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
ros2 launch agribot_vehicle_description configured_ackermann_sim.launch.py \
  gui:=false rviz:=true
```

Validate TF, sensor topics, steering, wheel motion, footprint and planned path
before moving to the physical vehicle.

## 4. Physical-vehicle acceptance

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
