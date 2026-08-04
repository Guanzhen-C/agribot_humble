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
`L1_INT`/`NARROW_INT` headings. It converts the C16 end-referenced point times
to the start-referenced `ring/time` layout required by LIO-SAM, supplies GPS
factors at the lidar optical center, and estimates a robust `map <- ENU`
transform from the final loop/GPS-optimized key-pose path rather than transient
online odometry. The resulting `*_georeference.yaml` is verified against the
PCD fingerprint before runtime automatic initialization.
