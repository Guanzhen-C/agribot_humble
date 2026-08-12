# Agribot offline mapping

This optional package adapts the physical C16, dual-antenna RTK and N300Pro
recordings to the official ROS 2 branch of LIO-SAM. It is intended for offline
Jetson or workstation processing; the RDK runtime navigation stack does not
depend on LIO-SAM or GTSAM.

The workspace vendors the pinned upstream source so mapping builds reproducibly
without a second manual import. The origin revision remains recorded in
`third_party.repos`. The local source keeps every added factor optional so the
upstream defaults remain available. The physical C16 configuration uses 1 m GPS
factor spacing, a 0.01 m^2 horizontal variance floor (10 cm standard deviation),
and a Huber robust kernel. GPS elevation is not used as an external height
measurement.

The offline configuration also prevents horizontal RTK factors from rotating
the entire 3D pose graph. The first key pose anchors roll and pitch with a 0.5
degree standard deviation and Z with a 0.1 m standard deviation. Each key pose
then receives GTSAM's `Pose3AttitudeFactor`, built from the calibrated N300Pro
gravity direction with a 1 degree standard deviation and a Huber kernel. This
factor constrains roll and pitch only; yaw remains governed by lidar odometry
and loop closure, and no RTK heading factor is inserted.

To compare or refresh the upstream tree independently:

```bash
vcs import src < src/agribot_offline_mapping/third_party.repos
```

## Standard offline pipeline

After sourcing ROS 2 and this workspace, one command performs playback, strong
robust horizontal RTK and gravity-factor insertion, optimized PCD saving, Nav2 projection,
georeferencing and result-trajectory recording:

```bash
ros2 run agribot_offline_mapping run_rtk_mapping_pipeline.py \
  /home/cgz/agribot_bags/INPUT_BAG \
  /home/cgz/agribot_maps/test_site/MAP_NAME \
  --domain-id 71 \
  --playback-rate 0.5
```

The command creates one coherent result family: `MAP_NAME.pcd`, `MAP_NAME.pgm`,
`MAP_NAME.yaml`, `MAP_NAME_georeference.yaml`, `MAP_NAME_result/` and
`MAP_NAME_manifest.yaml`. Existing output is rejected unless `--force` is
explicitly supplied. The manifest records the input bag, RTK-factor policy and
gravity-leveling policy,
so a viewer cannot silently combine a trajectory with an older map. The viewer
also verifies the PCD fingerprint stored by the georeference exporter before it
starts publishing any result.

Display the matching 2D map, 3D map, rear-axle RTK path and rear-axle optimized
LIO-SAM path with:

```bash
ros2 launch agribot_offline_mapping lio_sam_rtk_result.launch.py \
  map_base:=/home/cgz/agribot_maps/test_site/MAP_NAME
```

Set `show_3d_map:=false` on a resource-constrained display computer.

To compare independently recomputed localization, replay only the raw sensor
topics through the current physical FAST-LIO2, FAST-LIVO2 and KF-GINS
configurations:

```bash
ros2 run agribot_offline_mapping run_localization_comparison.py \
  /home/cgz/agribot_bags/INPUT_BAG \
  /home/cgz/agribot_maps/test_site/MAP_NAME_comparison \
  --domain-id 74 --playback-rate 0.5

ros2 launch agribot_offline_mapping lio_sam_rtk_result.launch.py \
  map_base:=/home/cgz/agribot_maps/test_site/MAP_NAME \
  show_comparison_paths:=true
```

When FAST-LIVO2 is recomputed independently, keep the other trajectories
from the comparison bag and replace only its path with:

```bash
ros2 launch agribot_offline_mapping lio_sam_rtk_result.launch.py \
  map_base:=/home/cgz/agribot_maps/test_site/MAP_NAME \
  show_comparison_paths:=true \
  fastlivo_bag:=/home/cgz/agribot_maps/test_site/FASTLIVO_RESULT \
  fastlivo_topic:=/aft_mapped_to_init
```

The comparison runner explicitly excludes any previously recorded
`/Odometry`, `/fastlio/odometry` or registered-cloud outputs. FAST-LIO2 is
anchored to the first timestamp shared with the optimized LIO-SAM path using
one rigid transform, so later differences remain visible. KF-GINS already
outputs the rear-axle pose in local ENU and is transformed with the matching
map's georeference. For the standard `MAP_NAME_comparison` output naming, the
runner also fixes the KF-GINS ENU reference to the same RTK reference stored in
`MAP_NAME_georeference.yaml`; this prevents an initial float solution from
introducing a constant trajectory offset. RViz uses red for quality-4 RTK,
green for LIO-SAM, blue for recomputed FAST-LIO2, cyan for recomputed
FAST-LIVO2, yellow for recomputed KF-GINS and orange for the final contiguous
quality-5 RTK interval. Both RTK paths apply the measured master-antenna lever
arm and represent the rear-axle center; the float path is not connected across
lower-quality intervals.
The conservative 0.5 playback rate keeps all estimators from dropping input
while they run together on the Jetson.

The pipeline records RTK quality 4 antenna positions independently of heading.
It converts the C16 scan-end cloud stamp and point timing to the start-referenced
`ring/time` layout required by LIO-SAM. Before feeding horizontal position to
the official `GPSFactor`, it requires a fresh, quality-checked dual-antenna yaw
and converts the left master-antenna measurement to the `lidar_link` graph-node
origin. The unmodified antenna trajectory remains available separately for
georeferencing. No RTK heading factor is added. A robust `map <- ENU` transform is
estimated from the final loop/GPS-optimized key-pose path rather than transient
online odometry. The resulting `*_georeference.yaml` is verified against the
PCD fingerprint before runtime automatic initialization.

The planar `map <- ENU` transform is estimated from synchronized RTK and final
optimized LIO-SAM positions. An excessive horizontal RMSE is recorded in the
output file and reported as a warning, but it no longer prevents exporting the
calibration for inspection. Runtime consumers retain their own quality limits.
Dual-antenna yaw is an independent quality check: when
`require_yaw_validation` is false, an excessive yaw RMSE is stored as
`yaw_validation_passed: false` instead of discarding an otherwise accurate
position transform. Such a file is accepted only by the mapped FAST-LIO startup,
where RTK supplies a coarse seed and the chassis remains inhibited until local
NDT/GICP registration succeeds. Pure NavSat navigation continues to require a
validated yaw.

With `useGpsElevation: false`, RTK contributes horizontal position only; the
tested receiver's altitude failed to return to its initial value by about 3 m
on a closed route, so map height remains governed by lidar, IMU and loop
constraints. The official `GPSFactor` has no antenna lever-arm state. The
adapter therefore performs the lever correction before the factor, using only
accepted fixed or float dual-antenna headings and propagating heading
uncertainty into horizontal position covariance. The raw antenna trajectory is
used when georeferencing the finished lidar trajectory.

The Nav2 projection uses the final optimized LIO-SAM trajectory as its local
height reference. This keeps the obstacle band tied to the lidar height on
sloped terrain instead of cutting the ground with one global Z interval.

The projection also treats the inner `0.28 m` on either side of the optimized
rear-axle trajectory as observed free space. The measured vehicle half-width is
about `0.336 m`, so this removes transient returns left by a person following
the vehicle while retaining a margin before static walls. Override it with
`--trajectory-clearance-half-width`, or set the value to zero to disable it.

For a site independently known to be horizontal, the finalizer accepts
`--level-horizontal-trajectory`. It fits the optimized trajectory plane and
applies one rigid rotation to both the final PCD and the projection reference.
The option is deliberately disabled by default because flattening a real slope
would destroy valid terrain geometry. The applied transform is recorded in a
`*_leveling.yaml` sidecar.

Leveling changes the map coordinate frame. A georeference generated from the
unleveled optimized path must not be paired with a leveled PCD; apply the
recorded leveling transform to the georeference first or regenerate it in the
leveled frame.

The raw bag remains unchanged. During offline playback, the C16 adapter removes
points in the rear-axle-frame region `x=[-4.0, -0.1275] m`, `|y|<=0.60 m`
before LIO-SAM consumes them, suppressing a person following directly behind
the vehicle without changing the live sensor topics. Two additional 3D boxes
remove returns from the left and right vehicle-mounted RTK antennas. These
filters run before scan matching and accumulation, so excluded returns do not
appear in either the final PCD or its 2D projection. The adapter also converts
the CX driver's scan-end cloud stamp and start-relative point times into the
scan-start stamp plus start-relative `ring/time` layout expected by LIO-SAM.
