# Agribot offline mapping

This optional package adapts the physical C16, dual-antenna RTK and N300Pro
recordings to the official ROS 2 branch of LIO-SAM. It is intended for offline
Jetson or workstation processing; the RDK runtime navigation stack does not
depend on LIO-SAM or GTSAM.

Import the pinned upstream source when offline mapping is required:

```bash
vcs import src < src/agribot_offline_mapping/third_party.repos
```

The pipeline accepts only RTK quality 4 positions and integer-fixed
`L1_INT`/`NARROW_INT` headings. It converts the C16 scan-end cloud stamp and
point timing to the start-referenced `ring/time` layout required by LIO-SAM,
supplies GPS factors at the lidar optical center, and estimates a robust `map <- ENU`
transform from the final loop/GPS-optimized key-pose path rather than transient
online odometry. The resulting `*_georeference.yaml` is verified against the
PCD fingerprint before runtime automatic initialization.

RTK factors constrain horizontal position only. The tested receiver's altitude
failed to return to its initial value by about 3 m on a closed route, so map
height remains governed by lidar, IMU and loop constraints.

The Nav2 projection uses the final optimized LIO-SAM trajectory as its local
height reference. This keeps the obstacle band tied to the lidar height on
sloped terrain instead of cutting the ground with one global Z interval.

The raw bag remains unchanged. During offline playback, the C16 adapter removes
points in the rear-axle-frame region `x=[-4.0, -0.1275] m`, `|y|<=0.60 m`
before LIO-SAM consumes them, suppressing a person following directly behind
the vehicle without changing the live sensor topics. It also converts the CX
driver's scan-end cloud stamp and start-relative point times into the
scan-start stamp plus start-relative `ring/time` layout expected by LIO-SAM.
