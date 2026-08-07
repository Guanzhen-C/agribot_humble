# Upstream Version

- Repository: https://github.com/Robotic-Developer-Road/FAST-LIVO2.git
- Branch: `humble`
- Commit: `837b7bbc1431cb04cf936528e52c83c835efba8e`

Local changes adapt the package to this ROS 2 workspace, add configurable
PointCloud2 timestamp units, reduce per-frame console output, and defer visual
patch allocation until the feature is retained to avoid an upstream leak. The
visual keyframe decision also compares against the newest observation because
`addFrameRef()` inserts observations at the front of the list; this prevents a
full reference image from being retained on every frame after initial motion.
