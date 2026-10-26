# K2 Scanned Campus Simulation

## App Scene Import

In the existing Unity App, open the simulation setup's scene page and select
`导入场景`, choose a ZIP and enter its display name. Successful imports are added
to the same scrollable scene grid and can use the current vehicle and selected
algorithm components. No goal or movement is dispatched by importing a scene.
After launch, goals are selected in RViz as for the existing farm scenes.

The importer supports LCC2 0.0.3 portable packages containing SOG2/DC Gaussians,
original PLY collision geometry, and `info/poses.json`. The ZIP may contain a
single outer directory. Arbitrary Gaussian PLY-only, SHN/quality, non-unit scale
or nonzero shift exports are rejected with an explanation rather than silently
converted into a different appearance. The full renderer currently requires
fewer than 2^24 Gaussians. Input archives are never edited.

Local imports live under `~/.local/share/agribot/scanned_scenes/` (override with
`AGRIBOT_SCANNED_SCENE_ROOT` for testing). Names are independent of directory
paths. Duplicate names are rejected; the same ZIP under a new name reuses the
prepared asset. Catalog publication is atomic and happens only after successful
conversion. Cancel closes the import process group; failed temporary output is
not added to the catalog. Scanned assets persist across App restarts, but the
current selection is not restored automatically.

User-imported cards have a top-right delete button and confirmation. Deletion
removes only that catalog entry, clears its current selection and recent-import
assistant reference, and is blocked while the managed simulation is active.
Built-in scenes cannot be deleted. Original ZIPs are retained. The conversion
cache is removed when its last catalog reference is deleted; other named imports
sharing that cache remain usable. Cache deletion is confined to the import
library's generated directories, with rollback if catalog publication fails.
Reimporting after the last reference is removed converts the original ZIP again.
The assistant's next scene query reads the updated catalog.

The standalone import command is also available:

```bash
ros2 run agribot_k2_sim import_scanned_scene \
  --archive /path/to/scan.zip --name '园区场景'
```

The renderer sources and shaders are bundled under `vendor/`; new imports do
not depend on the earlier private `/home/cgz/3dgs` project. The local machine
needs the Gazebo/Ogre development packages, compiler, GLEW, JSON headers, VTK,
NumPy, SciPy and Pillow. Unity's Linux native file chooser uses `zenity`.
The derived 2D map and selected spawn are conservative scan-derived estimates,
not proof that every road is traversable by every machine. Spawn selection
prefers road interiors near the capture poses and checks the surrounding mesh
for height discontinuities. This avoids placing the small test vehicle on a
curb; it is not a clearance certificate for large machinery. Simulated GNSS is
generated from the simulation, not replayed from the scanned package.

The independent importer implements the same validated K2 attribute decoding.
Raw archives, decoded Gaussians, derived meshes and maps stay outside Git.

The physics and CPU-ray lidar use all original K2 mesh triangles. Appearance
uses all original highest-resolution Gaussians plus environment, through the
reference EWA renderer. No mesh appearance proxy, color baking or extra Gaussian
downsampling is used. GUI and server render status are recorded separately;
server camera images must be checked, not inferred from the GUI screenshot.

Preparation preserves the source files and refuses to overwrite an output:

```bash
source /opt/ros/humble/setup.bash
source ~/agribot_ws/ros2_ws/install/setup.bash
ros2 run agribot_k2_sim prepare_k2_scene \
  --reference /home/cgz/3dgs/gazebo-k2-automation-20261022 \
  --archive /home/cgz/Downloads/其域k2数据/K2数据自动化所_render2.zip \
  --output ~/.local/share/agribot/k2_automation_gaussian_20261022
```

Launch in a domain and Gazebo master separate from real vehicles and other scenes:

```bash
source /opt/ros/humble/setup.bash
source ~/agribot_ws/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=86
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11375
export IGN_PARTITION=agribot_k2_20261022
export GZ_PARTITION=$IGN_PARTITION
ros2 launch agribot_k2_sim k2_ackermann.launch.py gui:=true rviz:=true
```

The default is stationary until a goal is selected in RViz. Set
`run_waypoints:=true` to opt into the prepared short route. This runs the existing
configured Ackermann vehicle, FAST-LIVO2 + simulated RTK, Smac Hybrid-A* and MPPI.
The one-time settled spawn anchor is the existing simulation initialization, not
an NDT/GICP relocalization test. Simulated GNSS is not the K2 capture's GNSS.

The running scene can be inspected without publishing motion commands:

```bash
ros2 run agribot_k2_sim check_k2_sensors --seconds 12 --output /tmp/k2_sensor_check
```

The same domain/localhost environment above is required. A nonzero `--distance`
for `verify_running_k2` explicitly requests a short simulated motion test; do not
run that tool while a user is selecting goals in RViz.

## Validation Scope

On 2026-10-22 the native App selected the actual K2 ZIP, accepted a custom Chinese
name, imported it, and retained its card after restarting without restoring the
previous selection. Cancelling an active import stopped its process and left
no partial catalog entry or staging folder.

The resulting scene preserves 8,051,840 original Gaussians and 1,260,465 collision
triangles. The Gaussian binary SHA-256 matches the earlier validated decoder
output. Both GUI and native 640x512 RGB camera rendered the full Gaussians.

The exact App-launched Car0 / STVL / FAST-LIVO2+simulated RTK / Smac Hybrid-A* /
MPPI stack completed a short 3 m target test: controller status succeeded,
physical displacement 2.849 m. The native App stop button then shut down the
simulation processes. Earlier curb-side spawn failures are retained in the
local evidence, and spawn selection was corrected before this successful run.

This is short integration acceptance, not full-campus navigation, every vehicle
or algorithm combination, or precise sensor synchronization acceptance. Camera
timing jitter and bounded FAST-LIVO2 frame drops remain. A successful import
does not validate all roads or certify a scan for real vehicle operation.
