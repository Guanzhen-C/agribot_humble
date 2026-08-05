# Agribot offline mapping

This optional package adapts the physical C16, dual-antenna RTK and N300Pro
recordings to the official ROS 2 branch of LIO-SAM. It is intended for offline
Jetson or workstation processing; the RDK runtime navigation stack does not
depend on LIO-SAM or GTSAM.

The workspace vendors the pinned upstream source so mapping builds reproducibly
without a second manual import. The origin revision remains recorded in
`third_party.repos`. The local source carries one mapping-policy patch:
`gpsFactorMinDistance` makes the upstream hard-coded 5 m GPS-factor spacing
configurable while retaining 5 m as its default. The C16 configuration uses
1 m spacing after the unchanged 5 m initialization distance.

To compare or refresh the upstream tree independently:

```bash
vcs import src < src/agribot_offline_mapping/third_party.repos
```

The pipeline accepts RTK quality 4 positions independently of heading. It
converts the C16 scan-end cloud stamp and point timing to the start-referenced
`ring/time` layout required by LIO-SAM and feeds position to the official
`GPSFactor`. No RTK heading factor is added. A robust `map <- ENU` transform is
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
constraints. The official `GPSFactor` has no antenna lever-arm state. Exact
lever compensation would require a separate attitude estimate before the
factor or a modified backend; neither is used in this official-source mode.
The measured lever arm is still applied when comparing and georeferencing the
finished lidar trajectory.

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
the vehicle without changing the live sensor topics. It also converts the CX
driver's scan-end cloud stamp and start-relative point times into the
scan-start stamp plus start-relative `ring/time` layout expected by LIO-SAM.
