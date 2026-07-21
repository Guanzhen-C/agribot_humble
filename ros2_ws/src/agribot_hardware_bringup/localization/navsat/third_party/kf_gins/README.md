# KF-GINS subset

This directory contains the minimum KF-GINS source subset used by
`rtk_eskf_localization`:

- `common/`: angle, Earth model, rotation, and navigation data types
- `kf-gins/`: IMU mechanization and KF-GINS state types

Upstream: https://github.com/i2Nav-WHU/KF-GINS

KF-GINS is distributed under GPL-3.0. The complete license text is in
`LICENSE`. Keeping this subset inside the ROS package removes the former
build-time dependency on the sibling `KF-GINS` source directory.
