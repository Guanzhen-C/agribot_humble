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

For the tracked differential vehicle, select its measured sensor geometry.
This changes only the adapter/configuration layer; the LIO-SAM source remains
unchanged:

```bash
ros2 run agribot_offline_mapping run_rtk_mapping_pipeline.py \
  /path/to/map_diff_outdoor_BAG \
  /path/to/map_diff_outdoor_MAP \
  --vehicle-profile differential \
  --domain-id 71 --playback-rate 0.5
```

The command creates one coherent result family: `MAP_NAME.pcd`, `MAP_NAME.pgm`,
`MAP_NAME.yaml`, `MAP_NAME_georeference.yaml`, `MAP_NAME_result/` and
`MAP_NAME_manifest.yaml`. Existing output is rejected unless `--force` is
explicitly supplied. The manifest records the input bag, RTK-factor policy and
gravity-leveling policy,
so a viewer cannot silently combine a trajectory with an older map. The viewer
also verifies the PCD fingerprint stored by the georeference exporter before it
starts publishing any result.

For an indoor recording without RTK, run the same mapping pipeline with
`--without-rtk`. This mode replays only the raw lidar and IMU topics, does not
start the RTK adapter or add GPS factors, and intentionally does not create a
`MAP_NAME_georeference.yaml` file:

```bash
ros2 run agribot_offline_mapping run_rtk_mapping_pipeline.py \
  /home/cgz/agribot_bags/INDOOR_BAG \
  /home/cgz/agribot_maps/test_site/INDOOR_MAP \
  --domain-id 71 --playback-rate 0.5 --without-rtk
```

Validate the production indoor localization chain with:

```bash
ros2 run agribot_offline_mapping \
  run_indoor_fastlivo_fusion_validation.py \
  /home/cgz/agribot_bags/INDOOR_BAG \
  /home/cgz/agribot_maps/test_site/INDOOR_MAP \
  /home/cgz/agribot_maps/test_site/INDOOR_MAP_fastlivo_fusion \
  --domain-id 76 --playback-rate 0.5 \
  --initial-x 0.0 --initial-y 0.0 --initial-yaw-deg 0.0
```

The supplied initial pose has the same role as RViz `2D Pose Estimate`: it is
only a coarse prior. The existing localizer still runs coarse NDT, fine NDT and
GICP, and only its accepted `/localization_pose` seeds the fusion node. FPFH is
not used in `manual` mode. The validator rejects bags containing RTK fixes and
checks that no RTK factor becomes active, the global correction stays frozen,
and the fused path remains identical to aligned FAST-LIVO2 propagation.

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

Pass `--vehicle-profile differential` for the tracked vehicle. This selects
its FAST-LIO2/bridge/KF-GINS lever arms and layers its lidar/camera calibration
after the shared validated FAST-LIVO2 tuning.

Recompute the production FAST-LIVO2+RTK trajectory separately so NDT/GICP
initialization and fixed-lag fusion are not competing with the other three
estimators for CPU time:

```bash
ros2 run agribot_offline_mapping run_fastlivo_rtk_comparison.py \
  /path/to/INPUT_BAG /path/to/MAP_NAME \
  /path/to/MAP_NAME_fastlivo_rtk \
  --vehicle-profile differential \
  --domain-id 75 --playback-rate 0.5

ros2 launch agribot_offline_mapping lio_sam_rtk_result.launch.py \
  map_base:=/path/to/MAP_NAME \
  show_comparison_paths:=true \
  fastlivo_rtk_bag:=/path/to/MAP_NAME_fastlivo_rtk
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

## Semantic navigation graph

`build_map_semantic_navigation_graph.py` creates the executable topology from
the two-dimensional map, not from a manually driven trajectory. It identifies
the nested outer and inner boundaries in a road-boundary occupancy map, smooths
both boundaries, extracts their equidistant centerline, projects the line into
free space with a configurable clearance margin, and samples a closed set of
places at equal arc-length intervals. Each `drivable` connection stores its
smooth centerline geometry and raster clearance. The production graph has no
trajectory source field or recorded vehicle footprint.

`describe_opengraph_instances_ollama.py` is the production semantic stage. It
projects every final OpenGraph 3D instance back into its source images, selects
up to two high-overlap mask crops and asks the local `qwen3.8:27b` model for an auditable
Chinese caption, category, visible evidence, static flag and drivable-surface
flag. Every OpenGraph object remains in the output. Only a clear, static,
non-drivable object above the confidence threshold becomes a landmark. A local
safety policy always rejects people, animals and movable vehicle categories,
even if the model incorrectly calls a parked vehicle static.

Each promoted landmark is embedded once with local `qwen3-embedding:8b`
(4096 dimensions). The normalized vector and its source-text digest are stored
inside the semantic metadata and copied into the navigation graph. Neo4j import
therefore reuses the vector and does not issue a second embedding request. The
original OpenGraph caption/category and all selected source views remain as
audit fields; there is no separate English-to-Chinese translation stage.

The verified outdoor result contains 834 OpenGraph objects, 434 promoted
instance landmarks before geometric filtering, 42 uniformly spaced places,
173 route-adjacent landmarks and 42 closed drivable edges. Its centerline
clearance contract is 1.0 m and every edge has complete road-semantic support.
The verified indoor result contains 212 objects, 95 promoted instance landmarks,
9 places, 19 route-adjacent landmarks and 8 drivable edges. Every landmark is
associated with exactly one nearest place; landmark associations remain
`executable: false` and never become vehicle edges. A* searches only the
place graph. Smac and Nav2 still independently apply the real Ackermann
footprint and minimum turning radius before execution.

Example map-derived construction sequence:

```bash
ros2 run agribot_offline_mapping describe_opengraph_instances_ollama.py \
  --opengraph-pickle /path/to/full_pcd.pkl.gz \
  --semantic-metadata /path/to/raw/semantic_instances.json \
  --dataset-root /path/to/aligned/opengraph_dataset/00 \
  --caption-directory /path/to/opengraph/caption \
  --work-directory /path/to/semantic_enrichment \
  --output /path/to/semantic_instances_ollama_zh.json \
  --cache /path/to/ollama_instance_semantic.sqlite3 \
  --base-url http://172.18.80.26:11434 \
  --stride 10 --maximum-views 2 --batch-size 4

ros2 run agribot_offline_mapping build_map_semantic_navigation_graph.py \
  --map-yaml /path/to/map.yaml \
  --road-boundary-map-yaml /path/to/road_edges.yaml \
  --semantic-metadata /path/to/semantic_instances_ollama_zh.json \
  --minimum-centerline-clearance 1.0 \
  --output /path/to/navigation_graph.json
```

`opengraph_semantic_map.launch.py` accepts an optional `navigation_graph`
argument. When set, RViz displays places, landmarks, drivable connections and
thin landmark-association links alongside the occupancy and OpenGraph maps.

Resolve a start, ordered intermediate places and a goal without starting Nav2:

```bash
ros2 run agribot_offline_mapping plan_semantic_route.py \
  --graph /path/to/navigation_graph.json \
  --start place_000 \
  --via place_010 \
  --via place_020 \
  --goal place_030 \
  --output /path/to/route_preview.json
```

The start may instead come from the current localization estimate with
`--start-position X Y`. It must lie within `--maximum-start-distance` of the
nearest semantic place. The planner validates all identifiers, connection
clearances and connectivity before running deterministic A* searches
between consecutive requested stops. Its output is explicitly marked
`preview_only`; it is not a Nav2 goal and cannot command the chassis.

Pass the resulting file as `route_plan:=/path/to/route_preview.json` to the
semantic-map launch. It publishes a transient `nav_msgs/Path` on
`/semantic_navigation/route_preview` plus labeled stop markers. Smac and Nav2
remain responsible for generating and collision-checking the executable path.

`plan_semantic_route.py` is also the deterministic routing stage used by the
phone semantic-navigation service. Its A* centerline selects a broad route
corridor but is never sent to Nav2 as a dense waypoint list. Production
navigation sends only the ordered model-selected destinations to Nav2; Smac
generates and collision-checks the complete Ackermann path within the corridor.

`config/semantic_task_plan.schema.json` is the language-model boundary. The
model may return only an immutable graph SHA-256, a task identifier, an ordered
list of destination `place_NNN` identifiers and a separate list of place nodes
to avoid. Landmarks are never executable destinations; their unique
`NEAREST_PLACE` relationship supplies the navigation anchor. The robot supplies
its own current start pose.
Extra fields such as
speed, steering, coordinates or direct commands are rejected, as are stale
graph hashes, unknown nodes, duplicate identifiers and a node appearing in
both lists. `plan_semantic_route.py --task-plan FILE` consumes this contract.

## Manual landmark draft editor

`manual_landmark_editor.launch.py` opens an existing occupancy map without
starting localization, Nav2, sensors, a chassis node or a semantic graph
publisher. Select RViz `Publish Point`, click the map, enter a landmark name
and category, then choose Save. The clicked `map` coordinates are read-only in the dialog.
Saved draft landmarks are immediately shown on
`/manual_landmarks/markers` and survive an editor restart.

The editor writes only a map-specific YAML draft. It does not modify the
semantic graph JSON, graph hash, Neo4j, embeddings or the server. A draft also
cannot become a Nav2 destination by itself.

```bash
ros2 launch agribot_offline_mapping manual_landmark_editor.launch.py \
  map_yaml:=/path/to/map.yaml \
  map_id:=map_lio_sam_0811 \
  output_file:=/path/to/map_lio_sam_0811_manual_landmarks.yaml \
  rviz:=true
```

## Server-side Bailian semantic task planning

The live planner does not send the complete semantic graph to the language
model. Production uses two Neo4j 5.26 containers on `172.18.80.26`: the outdoor
graph uses HTTP `7476`/Bolt `7689`, and the indoor graph uses HTTP `7478`/Bolt
`7691`. Every container holds one map only. Each landmark has one
`NEAREST_PLACE` edge, while `DRIVABLE` edges join executable places.
`build_semantic_model_context.py` remains an offline audit/export utility only.

`plan_semantic_task_bailian.py` uses two bounded Bailian calls from the 172 server:

1. Parse the user instruction into ordered Chinese destination search phrases
   and Chinese avoidance phrases without exposing any map node or translating
   the request to English.
2. Retrieve at most `--retrieval-top-k` place candidates per phrase using
   Neo4j vector and full-text indexes, then ask the model to select only among
   those candidates.

The final result must contain exactly one place for every ordered destination
query. Local validation rejects a place selected for the wrong query index,
stale graph hashes, unknown nodes, duplicate destinations, missing avoidance
matches and all extra fields. A deterministic A* stage then removes prohibited
nodes and road connections and returns a centerline for the vehicle-side wide
corridor cost. The implementation uses Python's standard HTTP library and adds
no Neo4j or OpenAI SDK dependency.

The production models are Bailian `qwen3.7-flash` and `text-embedding-v4`.
The phone and RDK reach only the bounded semantic HTTP service on
`172.18.80.26:8090`; only that server calls Bailian. `DASHSCOPE_API_KEY` remains
in the server-side systemd environment and is never stored in the app, RDK,
route document or Git. Neo4j also remains private to the server.

Import or refresh a graph after generating a new navigation-graph JSON. The
Neo4j credentials are read only from environment variables. Both full-text and
vector retrieval operate on the Chinese caption and category fields.

```bash
source ~/.config/agribot/neo4j.env

ros2 run agribot_offline_mapping import_semantic_graph_neo4j.py \
  --graph /path/to/navigation_graph.json \
  --map-id map_lio_sam_0811
```

The importer reports how many vectors were reused and generated. A production
schema-v2 semantic map should report all vectors reused and zero generated. A
missing or digest-mismatched vector is regenerated by the local embedding
model, and a dimension change recreates the Neo4j vector index.

```bash
ros2 run agribot_offline_mapping plan_semantic_task_bailian.py \
  --graph /path/to/navigation_graph.json \
  --map-id map_lio_sam_0811 \
  --model qwen3.7-flash \
  --base-url https://dashscope.aliyuncs.com/compatible-mode/v1 \
  --embedding-model text-embedding-v4 \
  --embedding-dimensions 1024 \
  --instruction "先巡检园区北门，再去白色建筑附近" \
  --task-id outdoor_inspection_001 \
  --start-position 0.70 0.80 \
  --intent-output /path/to/parsed_intent.json \
  --context-output /path/to/retrieved_candidates.json \
  --task-plan-output /path/to/validated_task_plan.json

```

Both server-side requests use JSON mode with thinking disabled and intentionally do
not set `max_tokens`, avoiding a truncated JSON document. Captions returned by
Neo4j are treated as untrusted sensor observations, not instructions. The model
cannot select the start, invent coordinates or authorize motion. A returned
landmark ID or a place outside the corresponding query's candidate list is
rejected. The returned task contains only ordered targets and keepout places;
Nav2 must independently plan and collision-check every executable segment before any
future integration may command the vehicle.

Natural-language exclusions such as "do not pass the blue bicycle area" are
translated to that landmark's nearest place and returned as `avoid_node_ids`.
The deterministic router blocks every drivable place and connection
intersecting the configured `--avoidance-radius` around those places. It rejects
the task when a destination or the current start lies in a forbidden zone, or
when no alternate route exists. RViz displays these preview zones as
translucent red cylinders. This graph-level exclusion does not by itself
constrain Nav2: the same zones must be installed in Nav2's Keepout Filter before
an executable route may be authorized.
