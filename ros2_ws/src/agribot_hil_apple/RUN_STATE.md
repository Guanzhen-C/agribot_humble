# HIL Resume State, 2026-11-10

## Current: Direction-Corrected Shredder Run02 Finished

Direction-corrected run02 completed with `status=succeeded`, one970-pose Smac
path retaining original1/2/3 goals and one successful MPPI FollowPath execution.
Independent finalXY=(3.257450835,32.382912515), error0.282825762m below0.418501555m.
Last5s actualXYspeed<=0.001677159m/s and cutter speed<=0.007990533rad/s;
native CAN decoder errors0, bridge rejected0 and fault empty. All173 frozen
inputs and all13 accepted sprayer baseline files rechecked unchanged.

Important residual: X5 USB-CAN disconnected AFTER navigation completed.
Journal says goal reached at0946768530.7585, disconnect shutdown at0946768557.08,
about26.3s later. CDC log contains `USB-CAN disconnected` and an EIO during
channel disable; do not claim lifetime zero IO errors or omit this observation.
Independent record confirms the chassis and cutter were already stopped before
disconnect. Client/core terminated fail-closed, and PC motion/work authorizations
were explicitly setfalse. Device later re-enumerated; physical cause unconfirmed,
asked user whether they unplugged it. No automatic restart was attempted.
Image decoder also exited-11 while the resulting SIGINT shutdown was in progress;
this is not evidence of a camera failure during navigation.

PC `agribot-shredder-pc` remains as an unarmed display; recorder02 and X5 core/
mission02 are inactive. Raw records were copied to private PC state. Newest source
and configuration changes are limited to this HIL package. No formal report edits.

## Previous: Direction-Corrected Run02 Startup

6750 target responded again today; real bidirectional CAN250k recovered without
flashing. A pre-existing gs_usb error-passive state was cleared with empty RTR
diagnostics before any experiment, not fabricated feedback or motion commands.
`shredder-bench-20261110-01.json` passes all10 real-chain checks, including actual
cutter rotation, native0x5B4 reverse ISOBUS feedback and work-timeout isolation.
Full orchard run01 completed1->2->3 on one970-pose native Smac path with MPPI.
Independent final error0.301573m, speed<=0.001624m/s and cutter<=0.007944rad/s in
the last5s; native decode errors0, bridge rejected2, CDC IO errors0.

User then found the implement facing backward. Entire CAD and actual cutter mount
were rotated180deg aboutZ, feed hopper now toward vehicle+X. Chassis/sensors and
algorithm YAMLs remain unchanged. Added orientation/rotor-axis assertions;166
tests pass. Old scene and run01 evidence preserved, not relabelled as run02.
New frozen173 inputs: `orchard-input-sha256-20261110-02.json` in private state
`.local/state/agribot-shredder-hil-20261109`. Current PCunits: `agribot-shredder-pc`
and `agribot-shredder-record-02`; X5 `agribot-shredder-rdk` and
`agribot-shredder-mission-02`. Mission file onX5 is
`.local/state/agribot-isobus-hil-20261109/shredder-orchard-mission-20261110-02.json`;
CDC trace has matching `shredder-orchard-can-20261110-02.json` name.
Read-only monitor exec84698 reached succeeded and exited; it is no longer live.
Startup notes below are historical. No JTAG was used while motion was active.
Do not edit formal reports or mix run01 and direction-corrected run02 evidence.

## Previous: Shredder HIL Pending Physical ECU Recovery

Stable sprayer checkpoint `748e58427d39d4e1d338edf6767e6f8a21e9d528` was pushed
and confirmed on GitHub main. New shredder changes are not yet a hardware pass.
166 regression tests and the separate Gazebo torque/actual-angle/stop fixture
passed. Original STEP source hash is f9fcd09fdc541ea46150eec0054bc2730cb15e111e0f4d44587a33a2e0001df6.
The correctly upright rigged CAD is in `.cache/agribot/isobus/work_models/shredder/rigged`;
the separate scene is `.cache/agribot/isobus/tracked_shredder_orchard`.
Chassis/contact/sensors and all algorithm YAMLs remain the accepted baseline.
X5 isolated package was rebuilt; configs copied to `scenario_shredder`.

First physical preflight: PC native bridge latched ENOBUFS, X5 trace
`shredder-bench-can-01.json` has no received frames. Both physical test units
`agribot-shredder-pc` and `agribot-shredder-rdk` were stopped before JTAG inspection.
FireDAP is enumerated but privileged OpenOCD target scan returns all ones, with
target unexamined. No reset/flash or nonzero hardware test was performed.
Asked user to check 6750 main-board power/debug cable and power-cycle.
Continuation recheck at100kHz also returns all-ones JTAG/unknown target, despite
OpenOCD exiting0 (`preflight-6750-03.log`); exit status alone is not readiness.
PC can0 is now DOWN to flush stranded transmit queues until hardware recovers.
All13 accepted sprayer file hashes rechecked unchanged; targeted tests4/4 pass.
Third consecutive goal-turn hardware audit still fails target examination
(`preflight-6750-04.log`, all-ones JTAG). Goal is blocked on physical ECU recovery,
not complete. PC simulation units and X5 CAN owners are confirmed inactive.
Evidence is `.local/state/agribot-shredder-hil-20261109/`, including JTAG logs and
software-only `model-physics-01.json` / actual Gazebo `model-preview-01.png`.
No formal report has been edited for the shredder.

Next: once physical ECU is reachable, stop software-only visual unit if running,
reinitialize serial-guarded PC CAN250k, start both real endpoints with fresh trace
names, run `verify_shredder_hil.py`, then full original orchard1->2->3 using new
shredder launch modes and independent PC recorder. Preserve failed evidence;
do not relabel the software fixture as hardware evidence or overwrite sprayer cache.

## Previous: Run12 Finished; Baseline Saved Before Shredder HIL

Run12 has finished with `status=succeeded`, one native971-pose Smac path and
MPPI execution. Independent endpoint error0.391045m is below the configured
0.418502m final tolerance. Actual chassis and fan/pump shafts stopped. The PC
recorder was stopped cleanly; X5 core is inactive; PC motion/work authorizations
were both revoked. Raw mission, independent and CDC records are retained locally.
CDC trace has25906 sent/54600 received, no decoder-invalid frames or IO errors.
Do not claim zero rejected native commands: the independent bridge log contains4.
The run06 intermediate-corner limitation remains; this is not precision via-point
acceptance or long-duration reliability certification.

Latest user requested a Git/GitHub checkpoint, then a separate physical-CAN
Gazebo test of the supplied shredder implement. Do not add that experiment to
the formal test reports. The sprayer baseline/configuration must remain intact.

## Historical: User-Requested Restart, Restored Run06 Parameters

User then explicitly asked to restart. All13 restored scene/config hashes checked
again, physical CAN250kERROR-ACTIVE, X5vcan/real USB present. Started PCorchard-pc
and X5orchard-rdk with can-mux-orchard-12.json. Real gateway01/01/03, localization
ready, no bridge fault. Live PathAlign3.5 and obstaclemax0.55 verified before motion.
PC motion/work explicitly authorized after preflight. Mission12 and independent
recorder12 started with fresh evidence files; original1->2->3. No tuning changes.
This is a requested running test, NOT a new complete-route pass yet.
Startup confirmed: missionwall9.72s, actual(3.521,35.669), actualfan/pump
93.779/83.359rad/s, native Smac971poses. Gazebo screenshot viewed; vehicle/mist
visible. Short startup observer exited cleanly; services intentionally keep running.

Units: agribot-isobus-orchard-pc / agribot-orchard-record-12 onPC;
agribot-isobus-orchard-rdk / agribot-orchard-mission-12 onX5.
Reports under each host's private agribot-isobus-hil-20261109 state directory.
Do not start duplicate CAN/mission owners. Keep camera under manual control.
On user stop, stop mission/recorder gracefully, disable bothPC authorizations,
and preserve raw reports. Run06's intermediate corner limitation remains expected.

## Historical: User-Requested Run06 Parameter Restoration

Latest user explicitly requested ALL parameters restored to the first complete
successful orchard run06. Further tuning/testing goal is paused externally; do
not resume experimental traction/height/corner changes or start motion unasked.
All test missions/recorders/X5 core/PC were stopped before restoration.

Active source preparation restored original imported Car3 physics, original
PathAlign3.5/step4/offset6, PathFollow7/offset6, PathAngleoffset8, footprintpadding0,
obstacleheightband[.08,.55], normal goal tolerance0.41850m. New strict intermediate
via gate, strict retirement and extra planner retry were removed. Original
controller-failure retry and physical CAN zero-handshake fixes from06 remain.
Experimental physics module/tests moved OUT of the active package into private
rollback-run06-20261109/experimental. Both build/install caches rebuilt fresh;
experimental module absent from both source and installed package on both hosts.

Regenerated vehicleSDF/URDF/world/vehicleconfig and nine RDK configs are byte-for-
byte equal to the preserved pre-physics scene and actual nav2-orchard-run06.yaml.
The regenerated mission object equals manifest embedded in orchard-mission-06.json.
13 files checked. Appearance, calibrated sensor layout, mist/occlusion and6750
FIFO fix retained.163 regression tests pass. This is a parameter restoration,
NOT a new complete-route run;06's~1m intermediate-corner limitation returns too.

Deployment complete: all13 local file hashes and all9 X5 configuration hashes
verified against the restoration receipt. Both overlays built successfully.
All experimental runtime units remain STOPPED; no new Gazebo launch or motion
test was performed after restoration. Last motion/work authorizationFALSE.
No live-parameter or new route-pass claim: source/config equality was verified.
Raw06..11 reports and all experimental scene/source backups preserved.

## Historical: Traction Fix, Test11

Latest user: "又撞树上了，现在底盘的摩擦力不够容易被地形影响".
Run10 failed after3 native controller attempts and4 failed planning requests,
finalpose~(37.825,31.906). Its minvias0.03343/0.02660m, maxroll6.82/pitch8.77deg.
User reported tree contact; a later stationary snapshot only had wheels/ground,
which does NOT disprove an earlier contact. No combined complete-route pass yet.
Mission10/recorder10/core/PC stopped safely, raw10 copied to PC; no Git push.

Read native tracked_slip_policy.hpp: at full turning blend it deliberately gives
ODE lateral slip0.003, allowing substantial downhill force-dependent drift. Old
straight compliance0.1 also allows creep while rolling. Latest orchard-only
traction profile: lateralmu0.36->0.6, longitudinalmu1.1->1.8, straightcompliance
0.1->0.005, turningODEslip0.003->0.0001; finite skid is retained. Torque12->20Nm
to preserve steering with higher lateral grip, still far below original200Nm.
Wheel acceleration0.5, loweredCOM,72.1kg load distribution and soft/damped contacts
unchanged. No wheel/scene/collision geometry or pose/velocity-state override.
Source/cache/manifest regenerated and X5 synced. Old cache before-traction-20261109.
Scene SDF byte-identical.170 tests pass, both overlays built.

Full test11 is starting from original spawn1->2->3; PCorchard-pc and X5orchard-rdk
are started with tracecan-mux-orchard-11.json. Preflight probe pending; then start
recorder11/mission11 only after fresh real CAN/FASTLIVO ready. Do not reuse reports.
Upperbranchband[.08,.85], padding.05, PathAlign18 and bounded planning retry retained.
Do NOT claim traction/tree/corner success before physical run11 measurements.

## Historical: Combined Full Test10

Run09 stopped before physical tree contact, but ended after Smac reported start
in lethal space during MPPI recovery (reportfailed). A later READ-ONLY plan from
the exact same pose/parameters succeeded with424 poses. Current contact snapshot
showed only wheels/ground, no tree; this is not a claim of zero contacts throughout09.
Diagnosis: immediate recovery can see transient voxel/pose footprint occupancy.
No obstacle clear, height rollback or physical bypass was used.

`PlannerPathError` now distinguishes a native no-path result. Orchard recovery
records failed planning requests, holds zero and workoff, waits1.25 wallseconds
while processing fresh data, retries within the SAME total six-replan budget.
Permanent no-path remains a hard failure; no motion on a rejected path. Unrelated
exceptions are not hidden.170 regression tests passed; both overlays rebuilt.

Full run10 active from ORIGINAL spawn and1->2->3, with all prior body, corner,
upper-branch band[.08,.85] and padding.05 fixes. PCorchard-pc + recorder10,
X5orchard-rdk + mission10, tracecan-mux-orchard-10.json. Read-only SSH monitor in
current turn prints every15s. Do not start a second mission or alter live parameters.
No full pass of the combined latest changes yet;06 remains the only full pass,
with its documented ~1m via limitation.07 tipped,08 hit tree,09 failed recovery.

## Historical: Upper-Branch Obstacle Test09

Latest user reported tree collision/stop during08. Stopped mission08/recorder08,
disabled both PC authorizations, preserved raw08, then stopped X5 core/PC to rebuild.
Actual Gazebo contact snapshot:vehicle_body_collision vs orchard trunks_collision
atworldz~0.88; basez~0.207. This is above the old base-level filter max0.55, so a
real upper-body/branch interference was excluded from the obstacle band. Filter
code was read to verify height is relative tohil_base_level, not globalground.
Two optional raw-cloud diagnostic snippets failed due mixed datatype fields in
read_points_numpy; no invented upper-point counts from these failed probes.

Run08 body changes DID keep it upright: independent maxroll6.74deg/pitch8.76deg,
but it still hit a tree, so NOT a full-route pass. Actual vias0.02247/0.06328m,
CAN faultsnone/rejected4. Its recovery policy attempted a new plan after prolonged
stall; original route was interrupted by operator action, not silently succeeded.

New filter max is derived from actual chassis collision top plus0.15m tilt/grid
margin, rounded up5cm: band[0.08,0.85]. Input remains real simulated LiDAR; no
truth-to-planner obstacle injection. Ground rejection lower limit unchanged.
Global/local footprint_padding0->0.05m; no tree collision removal, no scene flattening,
no speed/collision-critic relaxation. Source helper and prepared manifest store the
band; X5 launch reads it. Old scene archived before-obstacle-band-20261109.
168 tests pass. Both overlays built and X5 configured band verified before motion.

Full run09 is active: PCorchard-pc + recorder09, X5orchard-rdk + mission09, trace
can-mux-orchard-09.json. Read-only monitor execsession from current turn prints
every15s. Original spawn/1->2->3, PathAlign18 and loaded body fixes retained.
Do not call it passed before both vias, prior tree region and final stop are checked.

## Historical: Loaded-Chassis Stability Test08

Run07 resolved both intermediate corners (actual nearest0.03607/0.02563 m), but
then the vehicle physically tipped near(35.54,32.47) on the return. User explicitly
requested body-parameter changes. Mission07 was stopped and correctly persisted
interrupted; NOT a full-route pass. Recorder and CDC07 are preserved locally.
Independent quaternion shows ~85 degree pitch. Do not describe this as a CAN
failure or hide it by reporting only the two accurate vias.

Found poor physical proxies:200 Nm per-wheel limit, wheel contactkp1e9/kd1,
body mass at visual bbox centre, implement mass at the rotated CAD edge origin,
and slip normal loads computed before adding the10.1 kg implement/rotors.
These are risk factors; a single root-cause impulse has NOT been proven.

New orchard-only `tracked_physics.py`: same total72.1 kg, base mass centre lowered
toz0.04, loaded aggregate COMz0.09609 relativebase; implement bbox-centre proxy
transformed correctly instead of using CAD origin. Positive box inertias written
to SDF and matching URDF. Wheel torque12 Nm, acceleration0.5 m/s2, contactkp2e5/
kd1500/maxcorrection0.05, longitudinalmu1.1, wheel damping0.05. Loaded normal
forces front~169N and rear~184N, summingmass*g. No change to original wheel centres,
diameters, chassis collision shapes, scene mesh, sensors, mounts, drive/CAN path
or target cruising speed. No upright constraint/pose or velocity-state override.
All values are labelled illustrative simulation proxies, NOT measured calibration.
Old scene cache preserved as tracked_orchard-before-loaded-physics-20261109.
Scene world.sdf is byte-identical to that archive.167 tests pass, both builds done.

Full run08 active: PCorchard-pc and recorder08, X5orchard-rdk and mission08, trace
can-mux-orchard-08.json. Read-only monitor execsession20022. Original spawn/1->2->3,
PathAlign18 corner fix retained. Do not claim physical stability before the return
problem region and endpoint have been tested. End safely and preserve all reports.

## Historical: Intermediate Corner Accuracy Test07

User asked to resolve the ~1 m intermediate corner cutting in completed06.
Offline audit: actual Smac path approaches both original vias within0.03535 m,
while fused AND true trajectories miss them by~1 m. This is MPPI tracking, not
missing original planning goals or a ground-truth/localization offset explanation.

Only orchard tracking profile changed: PathAlign weight3.5->18, sample step4->2,
offset6->3; PathFollow weight7->5/offset6->3; PathAngle offset8->4. No speed,
footprint, collision validator, obstacle layer or final tolerance relaxation.
Previous nav2 YAML archived in private state as nav2-orchard-run06.yaml.

Orchard route recovery now retires vias using ordered fused-pose proximity only
(0.8*half-wheelbase), not Nav2 remaining-path feedback. Ground truth is used only
to audit minimum actual via distances; it is never fed to localization/control.
The mission fails if either actual intermediate via misses half-wheelbase.
Shared Navigator retains its legacy default feedback policy for old Ackermann.
Regression163 passed; both overlays built and X5 live PathAlign weight18 confirmed.

Clean full run07, original spawn and1->2->3 points. PC unitorchard-pc and independent
recorder07, X5 orchard-rdk and mission07, tracecan-mux-orchard-07.json. Read-only
SSH monitor execsession70968 prints report every15 s; do not spawn another mission.
No runtime changes while this run is moving. Keep camera manual. Finish by auditing
all via distances and actual stop, preserving raw reports, stopping recorder/X5
core and disabling PC motion/work. NOT yet a measured corner-accuracy pass.

## Historical: Full Orchard Test06 Completed

Native Smac/MPPI route06 completed in ~406 wall seconds, with ~78.38 m observed
movement and actual fan/pump operation in both rows. Native action succeeded with
one 971-pose plan, no replans. Independent PC record: no CAN errors, no bridge
faults, rejected commands=0. CDC trace:17266 sent/36255 received, no decode errors.
Final actual error: mission check0.40593 m, settled0.38564 m, inside the0.41850 m
half-wheelbase tolerance. Last5 s max planar speed0.00143 m/s, yaw0.00468 rad/s;
final fan/pump ~0.000273 rad/s. Real stop and work-off verified, not just command0.

IMPORTANT LIMITATION: intermediate vias were retained in the actual Smac plan,
but continuous MPPI tracking cut the two headland corners. Actual closest
distances were0.98113/0.95149 m, NOT inside half-wheelbase tolerance. Thus complete
navigation/work is demonstrated, not precise intermediate-via acceptance. No
forced failure/CAN dropout was injected in successful06; recovery has unit coverage.

Mission, recorder and X5 experimental core are stopped. PC Gazebo remains visible
with motion/work authorization FALSE, no camera lock. Validation summary and raw
hashes:validation/tracked_orchard_hil_20261109.json. Raw reports copied to PC private
state.162 regression tests passed. Nothing committed/pushed for this request.

## Historical: Full Orchard Test06 Startup

User requested the complete moving HIL test after spray clipping. PC orchard
physics and the full X5 algorithm core have been cleanly restarted. Mission06
and independent recorder06 are running with fresh files. Do not call this a pass
until all three original vias, native action success, actual endpoint/stop and
actual implement work/stop have been audited. Camera remains manually controlled.

Run04 diagnosis: recorder and mission have different start offsets. The native
CAN recorder has a 0.600 s gap at recorder wall133 s, corresponding to mission
wall~104 s, exactly when motion stopped. X5 CDC trace has no outbound drive gap;
two native frames were lost farther along the physical path. PC CounterLease
expired and correctly rejected nonzero commands until a zero command; previous
wording about an earlier unexplained stop used mismatched time origins.

The PC now reports drive/work lease freshness. X5 `tracked_drive_guard` forwards
MPPI commands only with fresh command/status, motion authorization, fresh truth,
fresh physical lease and no fault. Otherwise it sends zero THROUGH the unchanged
native CAN driver and physical 6750 path; only real receiver readiness re-enables
motion. The 0.5 s receiver lease, replay checks and restart-neutral requirement
remain intact. This is no actuator bypass and does not disable localization guards.

Run05 failed at x~8 m: MPPI action aborted as the live obstacle map changed from
the initial path. CAN rejected=0, no bridge fault. Its reports are retained, NOT
a pass. The orchard mission now retries native Smac ONLY after an ended or
explicitly cancelled controller path, at most six replans. It retains all not-yet-
passed original vias, suspends work while replanning, and does not periodically
replace the path. Unrelated exceptions are not retried. Interrupted missions now
persist `interrupted` instead of stale `executing`. Regression suite:162 passed.

Run06 units: PC `agribot-isobus-orchard-pc`, recorder `agribot-orchard-record-06`;
X5 core `agribot-isobus-orchard-rdk`, mission `agribot-orchard-mission-06`.
Evidence: orchard-mission-06.json / orchard-independent-06.json /
can-mux-orchard-06.json, each in the appropriate host's private state directory.

## Previous: Spray Obstacle Clipping, Full Route Paused

User redirected the moving test to fix mist passing through obstacles. Mission04,
full X5 algorithm core and its independent PC recorder were stopped; no full-route
pass. Raw mission04 says executing because SIGINT only ran the old finally block;
this is stale persisted state, NOT evidence of a still-running mission or success.
It advanced to x~22.48 at wall104 s, then stopped progressing and spraying before
the user interruption at wall178 s. PC bridge later had 373 rejected commands,
no latched fault. Preserve reports and investigate that separate issue before
claiming a full navigation pass; do not explain the earlier stall as user action.
Local read-only monitor85303 was explicitly terminated and drained.

New `work_spray_obstacles.cpp` uses actual Gazebo physics ray intersections with
wall/ground/tree collision meshes, 9x9 samples per real nozzle, 10 Hz, 4.5 m range.
It ignores only this vehicle's coarse chassis proxy, not environment mesh bounds.
World-space range grids are sent over local Gazebo transport to the visual plugin;
surrounding samples' minimum depth conservatively clips particles with sprite-size
and movement clearance. Expired geometry stops emission/clears particles, no
unblocked fallback. This is sampled visual occlusion, NOT full liquid/impact physics;
render-only geometry without collision shapes is not tested by physical rays.
Water above a wall may pass OVER it; intersecting wall directions are clipped.
No new implement collision body/footprint, mounting or nozzle-direction change.

Both C++ plugins compiled; 157 tests passed, including native clipping geometry
and shared source positions for visual/physics rays. Real static HIL observer
`mist-obstruction-observer-12.json`: pump peak83.3605 rad/s, CAN errors0, motion
authorization false, drift .00393 m. Physical blocked rays ~220..232/648, nearest
~1.05 m; GUI log `~/.gazebo/client-11357/default.log` confirms depth_fresh=1 and
active particle clipping. Actual screenshot `mist-obstacle-clipped-12.png` viewed,
RTF .73 (previous .74); some spray can clear the low wall's top. GUI stdout is in
Gazebo client log, not the service journal. Original geometry/maps unchanged.

PC `agribot-isobus-orchard-pc` running, motion disabled. X5 basic mist-only unit
`agribot-isobus-mist-rdk` is active with `can-mux-mist-obstruction-12.json`, work
authorized there but no mission. Any preview is finite and disables PC work on exit.
Firmware remains the verified FIFO-drain fix, no further flash for spray clipping.

## Historical Full Orchard Test04 (Interrupted)

After clean restart, gateway bytes 01/01/03, localization ready, PC physical
bridge fault empty. PC motion/work enabled and X5 console entry point
`ros2 run agribot_hil_apple tracked_orchard_mission` started as
`agribot-orchard-mission-04`. Real Smac path has 971 poses. At wall40.12 s the
independent truth is (9.908,35.902), fused (9.879,35.898); actual fan/pump
93.777/83.357 rad/s. This is genuine motion and spraying over the physical CAN
chain, not an intent-only preview. Full route is still IN PROGRESS.
Same core unit `agribot-isobus-orchard-rdk`, trace `can-mux-orchard-04.json`;
PC read-only `agribot-orchard-record-04` saves `orchard-independent-04.json` on
SIGINT or at 1000 s. Mission report is X5 `orchard-mission-04.json`.
Read-only local monitor exec session85303 polls that report every15 s and exits
at succeeded/failed; do not start a second mission or touch ECU debug while moving.
End by stopping recorder gracefully, copying mission/CAN trace, verifying endpoint
and actual stop, then disabling PC motion/work. Do not label a partial run passed.

## Full Orchard Startup Diagnosis Before Test04

Test02 was blocked BEFORE motion: gateway never became ready. The first `python
-m` invocation did not call the console entry point (no test/report); corrected
`ros2 run` invocation produced a genuine failed-preflight report. Preserve it.
Physical trace02: 1889 outbound frames, ZERO received, decoder errors zero.
Read-only JTAG snapshot `orchard-02-can-state.log` found CAN1 CMD=0x00c38000:
secondary TX FIFO full, TSONE/TSALL both clear, error counters zero. SDK's
nonblocking helper only triggers TSONE; when STB is full it returns before any
trigger and can leave queued frames permanently stranded after adapter restart.

Firmware `send()` now also calls SDK `can_start_all_message_transmit()` after
success OR FIFO-full, draining pending hardware frames on both buses. No new
protocol or command added. Old ELF/BIN saved as demo-before-fifo-fix.*; original
full-Flash backup remains. New ELF programmed and Verified OK; log
`flash-isobus-03-fifo.log`. Real CAN1 gateway/feedback immediately returned in
run03 (FF60/FF50/FF52/FF53, native 532/533/534), confirming recovery, but the PC
bridge had latched ENOBUFS while the ECU was halted for programming. It correctly
reported fault and prevented work; do not mask it. Stop/restart both ROS sides
cleanly after flashing, then use fresh trace/mission/recorder filenames for run04.
Runs02/03 are NOT route passes. PC motion/work disabled during diagnosis.

## Previous Test02 Startup Details (Superseded Above)

User explicitly requested the full moving work test after accepting appearance.
PC `agribot-isobus-orchard-pc` is running; motion/work authorizations enabled only
after real CAN feedback and FAST-LIVO2/RTK readiness were observed. X5 mist-only
run06 has been stopped to keep one physical CDC owner. X5 full algorithm unit is
`agribot-isobus-orchard-rdk`, with `can-mux-orchard-02.json` trace, localization
guard enabled, native Smac + MPPI DiffDrive. The unmodified 1->2->3 route is
(41,36,0), (41,32.5,pi), (3,32.5,pi), not a direct velocity command or ROS bypass.
Mission unit `agribot-orchard-mission-02` writes X5 `orchard-mission-02.json`;
independent read-only PC unit `agribot-orchard-record-02` writes
`orchard-independent-02.json` when interrupted cleanly or after 1000 seconds.
All reports are under each host's `~/.local/state/agribot-isobus-hil-20261109/`.
The test is in progress, NOT a full-route pass. Monitor the same units/reports;
do not start duplicate CAN or mission owners. End by recording endpoint/stopping
status, stopping the recorder with SIGINT, copying X5 results/trace, and revoking
PC motion/work authorization. Keep GUI visible, camera manually controlled.

## Current Display: Grey Chassis, White Tank, Textured Sprayer

Latest user instruction supersedes black chassis: use medium-grey shell, white
HDPE tank, green painted steel frame, satin-metal fan/nozzles/fasteners and dark
polymer caps/pipes. The atlas is stored in `models/tracked_sprayer/`, generated
with the built-in image tool; exact prompt and rebuilding steps are in its README.
`prepare_work_material_parts.py` separates the original 123 STEP solids by
material role, then `blender_textured_work_model.py` makes a 150748-triangle mesh
(old neutral mesh 150000), with four real UV/textured materials. Source STEP hash,
normalization and 0.55 mounting scale remain unchanged; no new collision geometry.
Prepared visual cache: `~/.cache/agribot/isobus/work_models/plant_protection/textured/`.

`appearance.py` copies the chassis OBJ byte-for-byte into each scenario and only
changes shell MTL colours. Grey diffuse RGB is (.32,.33,.34); lights, track rubber,
wheel hubs and all separately mounted sensors keep original colours. SDF and URDF
reference this same isolated body mesh. Original Unity/vehicle exports untouched.
The black intermediate cache is archived as
`tracked_orchard-black-chassis-textured-20261109`; do not restore black by mistake.
PC overlay rebuilt and all 156 tests passed. Actual black intermediate screenshot
`orchard-textured-white-tank-black-chassis-11.png` was viewed and showed the atlas
loaded correctly; it is not the final grey-body screenshot.
Final `orchard-textured-white-tank-grey-chassis-12.png` was also viewed: grey
shell, white tank, green frame and metal fan render correctly, real-time factor .74.

PC Gazebo is restarted for display, motion/work authorization remain false;
X5 mist core run06 remains active and no second CDC/CAN owner has been started.
The user briefly requested walking, then redirected the interrupted turn to
materials/colours. No new autonomous orchard mission has been started yet.
Blue-grey mist, original radial outlet coordinates and stronger range remain as
accepted below. Latest colour-only run10 was 12 seconds, no CAN errors, actual
pump/fan peaks 83.3605/93.7804 rad/s, maximum chassis drift .003944 m;
`mist-blue-grey-10.png` was viewed and clearly shows the blue-grey spray.

## Latest: factory nozzle mouths and stronger radial mist

User confirmed the corrected nozzle positions on 2026-11-09, then requested
greater spray range and a more forceful appearance. Keep the accepted implement,
RTK and IMU mount unchanged. Use the eight actual 0.4 mm bore outer-end circles
in STEP solids 66/68/70/72/74/76/78/80, plus 1 mm outward clearance. Emit radially
away from the fan axis, not axially from fasteners 82/86/.../106. The earlier
seven-point interpretation was incorrect; do not restore it or add nozzle meshes.

The invisible anchor's measured Ogre scale is (2e-5,2e-5,2e-5). The independent
particle child ignores this scale and measures (1,1,1), fixing collapsed origins.
The physical-CAN run `mist-radial-bores-observer-08.json` recorded pump 83.36 rad/s,
fan 93.78 rad/s, no CAN decode errors and motion authorization false. User accepted
the resulting locations; `mist-radial-bores-close-08.png` instead shows a wall
because of camera movement and must NOT be cited as visual acceptance evidence.

The new appearance parameters are velocity 2.0..2.8 m/s (previously 1.1..1.6),
lifetime 0.9..1.3 s (previously 0.65..0.95), cone half-angle 0.18 rad (previously
0.10), maximum emission 2400/s (previously 1600/s), growth 0.10 (previously 0.06).
Quota follows maximum emission and lifetime, with 20% headroom. Original bench
defaults are retained. The user accepted the stronger range, then requested more
visible colour: orchard RGBA is now (0.36,0.60,0.78,0.60), blue-grey instead of
near-white (0.84,0.94,1,0.28). Positions, directions and range stay unchanged.
The previous stronger-range run09 recorded pump peak 83.3602 rad/s, fan 93.7801,
no CAN errors, motion authorization false and 0.00395 m maximum chassis drift.
Its actual screenshot `mist-strong-radial-09.png` was viewed; mist was visible
around the original fan-rim nozzles (not a full-route navigation test).
These are visual parameters, not calibrated fluid dynamics
or an increase of actual pump command. Visual plugin rebuilt; 153 tests passed,
including direct checks of the eight original STEP bore edges and radial axes.

PC orchard service is for stationary inspection only; motion remains disabled.
X5 mist core `agribot-isobus-mist-rdk.service` run06 is active (not localization
or navigation). Earlier USB retries below are historical: after the last replug,
PC can0 was DOWN and X5 vcan73 was missing after reboot; both were restored using
the existing targeted scripts. Run05 failed because vcan73 was missing; run06
and the subsequent 18-second run08 observer successfully used the physical
X5 -> 6750 -> PC CAN chain. No new firmware flash was needed. A finite preview
automatically expires and turns work authorization off; do not bypass CAN to
show the mist. No full orchard navigation acceptance is claimed.

## Historical Mounting and USB Retry Timeline (Superseded Above)

PC unit `agribot-isobus-orchard-pc.service` is running for inspection only;
motion/work authorization are false. X5 orchard core/mission are stopped.
Sprayer uses uniform 0.55 scale; latest user instruction is tank forward, fan
rearward (-X), superseding the earlier sideways orientation. Full 3D mount aligns
STEP tabs (solids 29/30/43/44):
their common outer contact faces lie on the deck z=0.42665, hole axes vertical.
Previous yaw-only installation left these tabs vertical and was rejected.
No added green mounting plate, rear overhanging platform or rails.
RTK housings with original 62 mm short stems/feet (same style as Ackermann),
no tall posts/rails, at (0.44, +/-0.29, 0.52665) m (each moved outward 3 cm). Display, TF,
sensor JSON, fusion lever arm and georeference agree; updated configs and
package built on PC and X5. Original exports/CAD/bench remain unchanged.
IMU is now at (0.31,0,0.436081) m on the exposed chassis deck ahead of the
implement and behind the sensor pedestal (deck z=0.426081), not on the implement.
It remains laterally centred, without a new bracket. SDF sensor+visual, URDF/TF and JSON agree;
FAST-LIVO2 extrinsics and odom-bridge body lever arm regenerated from this config.
The prior (0,0,0.46965) crossmember position was rejected by the user. Its TF
check and orchard-centred-imu.png are historical, not the final mounting evidence.
Latest live TF is (0.310,0,0.436), and /imu/data still publishes imu_link headers.
Viewed orchard-imu-on-chassis.png: black IMU clearly rests on exposed white
chassis deck between implement and front pedestal, separated from implement.
Spray plugin nozzle positions now support scaled CAD-local placement.
Latest mist revision uses ONLY the supplied CAD nozzle outlets, 7 end-face axes
(solids 82/86/90/94/98/102/106), with no added yellow cylinders in SDF or URDF.
The earlier 12 generated nozzles were rejected and removed. Soft alpha particles
fade/grow/fall and are driven by integrated pump-shaft velocity; no fluid claim.
Physical stationary observations mist-stationary-observer-02.json confirmed
native (3,4,0), pump 83.3655 rad/s and fan 93.7854 rad/s; no CAN decode errors,
chassis drift 0.00496 m. This was the TWO-emitter intermediate visual version,
NOT final factory-outlet ring acceptance. New geometry/152 tests passed.
X5 USB-CAN disconnected again during mist runs 01/02, and run03 startup failed;
user has been asked whether these new events were manual, no answer yet. Do not
reuse the earlier confirmed manual unplug explanation for these new events or
claim a proven hardware fault. Preview sender and X5 mist core are stopped;
no automatic reconnect/nonzero-command restart. Final ring still needs a stable
physical-CAN visual run. Do not modify unrelated network interfaces (new third
PC adapter belongs to another task), USB hub or power settings speculatively.
Latest explicit user retry also failed: run04 identity check caught the USB
device disappearing during startup. A subsequent read-only 12-second observation
of the exact serial saw USB device numbers 93 -> 94 -> 95; missing at second 6
and seconds 8..11. Run05 was NOT started after this failed stability preflight.
PC can0 remains UP/ERROR-ACTIVE at 250 kbit/s, bridge has no CAN fault and both
motion/work authorizations false. No nonzero work or drive intent was sent in
this retry. Needs user inspection of the X5-side old USB-CAN USB connection.
Next user-requested retry: 15-second USB inventory plus another 10-second
identity/access check both failed even WITHOUT starting a CAN process. The
adapter repeatedly disappeared/re-enumerated; kernel reported config error -71.
When present it was accessible to sunrise, so this check did not show a tty
permission blocker. Run05 remains NOT started, movement/spraying remain off.
After user explicitly replugged: serial was recognized again, but the 8-second
check lost the device at second 7; kernel then showed devices 12 -> 13 with
another hub-disable/disconnect. Device power/control=on and runtime_status=active,
so USB autosuspend was not enabled on this adapter. No settings changed and no
CAN/spraying process started; suggest another USB data cable or X5 USB port.
152 tests pass. No added mounting plate; this is not complete navigation acceptance.
The previous foreground screenshot (without short stems) is historical:
`~/.local/state/agribot-isobus-hil-20261109/orchard-compact-mount.png`.
Latest actual Gazebo screenshot viewed with horizontal tabs and tank cap up:
`~/.local/state/agribot-isobus-hil-20261109/orchard-mount-tabs-on-deck.png`.
Camera remains manually controllable (no follow/lock).

The first orchard run ended ABORTED (action status 6), not a full-route pass.
Preserve `orchard-mission-01.json` and `orchard-independent-01.json`; do not
overwrite them or conflate them with the successful short bench tests below.
Old large mounting cache archived as `tracked_orchard-before-compact-20261109`.
No additional firmware flash, Git commit or real implement motion for this edit.

## Current: tracked ISOBUS spraying HIL

User scope: implement BOTH native agricultural protocols (plant protection and
shredder), but demonstrate ONLY spraying on Car3 now. Driving and ISOBUS share
one upstream physical CAN, both segments classic 250 kbit/s. No ROS drive bypass.

Three finite physical HIL tests PASSED all 8 checks through actual X5 ZQWL ->
6750 CAN1 -> 6750 CAN0 -> PC gs_usb -> Gazebo physics. Forward displacement was
about 1.70 m each run, actual shaft velocities rose, independent work-input loss
stopped shafts while drive continued, final chassis stopped. Actual active-spray
GUI screenshot viewed: gazebo-physical-spray-03.png. Software stack SIL covers
all three fields including shredder; no shredder animation or real implement
calibration was claimed. 147 package regression tests pass.

Evidence: PC ~/.local/state/agribot-isobus-hil-20261109/ contains
tracked-spray-hil-01/02/03.json, work-motors-01.json, isobus-sil-03.json,
CDC can-mux-02/03.json and original firmware backup/readbacks/logs. Source and
new launch scripts are in this package; main real-robot/Unity sources unchanged.
X5 isolated workspace ~/agribot_isobus_hil_ws; compiled ARM endpoint in its
~/.local/state/agribot-isobus-hil-20261109/build-host/. Old apple HIL preserved.
Local baseline Git tag pre-isobus-hil-20261109 exists; changes not committed yet.

6750 original full 16 MiB Flash backup SHA256:
37994f277fb0ef2cf2065dd810f21e2a7ca9aa6fd60114b17d55aa53996208ae.
New dedicated test firmware built from firmware/isobus_tracked and pinned MIT
AgIsoStack++ 02f3fe910de5fdc73e2014394c2e80900490b9cc. ELF programming Verified OK,
all 5 Flash load segments match independent readback, last 64 KiB unchanged.
First raw BIN programming used the wrong origin, was corrected using ELF, and
is explicitly NOT successful evidence. SDK BIN starts at 0x80000400, not base0.
Latest correct flash log flash-isobus-02-elf.log; no need to reflash again.
New test image does not implement the old VCFG diagnostic UI; rollback backup
and original firmware are retained. Private application PGNs only, not UT/TC or
AEF-certified/registered-manufacturer interoperability.

USER CORRECTION: the USB disconnect after the successful short tests was a
manual unplug by the user, not evidence of an adapter/hub/power fault. The prior
hardware-instability inference from kernel "disabled by hub (EMI?)" is withdrawn.
can-mux-02 is clean; can-mux-03 retains the manual disconnect at wall119 s, outside
completed functional test windows. No additional long-duration test was run.
Adapter is enumerated again,
but all experimental PC/X5 processes have been stopped. HPM sends only leased
zero work frames without a client. Keep raw disconnect evidence and its corrected
manual-operation explanation; do not diagnose a hardware defect from this event.
Critical endpoint exit handlers now stop the whole isolated X5 experiment;
reconnect must not automatically restart nonzero commands. PC bridge exit also
terminates its simulation, avoiding a held Gazebo velocity after process death.

Detailed reproduction/limits/rollback are in README.md first section. The old
Ackermann 1 Mbps experiment below is HISTORICAL and needs its old firmware/bitrate
restored before reuse. Do not run old/new CAN owners together.

## Historical Ackermann state

Run18 has met the continuous original-start/two-fruit-pick/return HIL acceptance.
All16 independent cross-checks passed. This is a known-object-pose simulation
manipulation baseline, not visual detection or real-arm force-control validation.

- PC workspace: `/home/cgz/agribot_ws/ros2_ws`.
- PC evidence: `~/.local/state/agribot-hil-20261105/`.
- X5: `sunrise@192.168.1.102`, private workspace `~/agribot_hil_ws`.
- Domain 73, wired DDS only. Existing native Ackermann CDC/CAN chain only.
- HPM6750 is now physically in the native CAN path (previous bypass is historical).
- Source changes remain solely in the untracked `src/agribot_hil_apple/` package.

## Authoritative Latest State

**Run32 TERMINAL SUCCEEDED through actual6750 CAN bridge, independently VERIFIED.**
No mission/core/recorder/mirror stillrunning. X5mission32MainPID0/active-exited/
exit0, corestopped. PCauthfalseconfirmed, recorderSIGINTflushed. Bothstopcommands
drainedBEFOREfullrsync, nonemptyfullmission/corejournals/rawPC/provenance retained.
PCGazebo+fullmapRVizremainatfinish3, manualGazebocamera restoredafterbriefRViz
capture. RVizfulloriginalmap/nativepaths/recordedfusedtrace retained; RobotModel
showsTFerrorafterX5corestop (not an active-localization view). No fakestaticTF.

All18physicalaudit+10fluency+7gripperchecksPASSED. Original1/2/3 inorder, stop3
independenterror.213912931m. Twofruit5/7truebilateralcontacts/detach/deposit,
bothstillphysicaltrayatfinish, batchstowed. Bothnavphasesattempt1,0failedplans,
0reverseCAN/plannedsegments,0>=2simsecunplannedstops. NativeCANerrors0,
nonzerocommands9158/feedback57132bylastrecord, IO/checksum/replayerrors0.
Actual20Hzfingerfeedbackcomplete: outbound3686samplesmax4.48164um, completion
1872samplesmax.784761um; holdreleasedduringactualpicking. Wall586.7746s from
missionstart tofinish; samealgorithms/route/tolerancesas31, topologyonlymetadata.
102frozeninput/source/plugin/config/OSprofileSHAallunchanged. Originalsource/
6750firmware/Unity/Ubuntu untouched; noGitcommit. Allfiniteexecsdrained.

TrueRVizforegroundcapture rviz-final.png VERIFIEDfulloriginalmap/paths+Global
StatusOK; Car0TFerroraftercorestop noted. Earliercapture-obscured-gazebo.png is
actuallyoccludingGazebowindow, NOTRVizevidence; retainedhonestlyrenamed. Foreground
wmctrl+xwdcapturethenrestorepreviousGazeboactivewindowfixedthatcaptureissue.
FullreportsinPCstate/full-mission-32; noonlinecodechangesduringmission.

PersistentnetworkrequestdoneBOTHPC/X5: root0644profileidenticalSHA
6a280343d23968dc169734ebb8a3831b6b2aa33156fe9d1d2778eec36a78fc74,
systemd-sysctlactual--cat-configincluded/2keysonly, sysinit.targetdependsservice.
Actualbootloaderreapply--prefix=/net/core/rmem_max +/net/core/wmem_max passed
andreadback16MiBboth; nootherkeyschanged, noactualPC/X5reboottest. Standardboot
loadingispersisted; prepare_network.shtemporaryfallbackstillavailable.
FireDAPUARTtimeoutstillseparateknownissue, noflashorclaimtargetmemoryverified.

The following Run32-active section is HISTORICAL:

**LATEST USER: run the whole HIL through6750; then persist network buffers; continue.**
**Run32 ACTIVE**, X5unit agribot-hil-mission32.service PID23477 parent1284/systemd,
actualactive-running; outboundattempt1 following570points, physicalcarfirstrow
at24.79/36.03 confirmed. No newstart on usercontinue. PCauthtrue successful;
PCmain/record32/fullmapview(report32)/readonlymirror32 andX5core active.
Original1/2/3+same5/7picking+finish3, nohome/4 oralgorithm/tolerance changes.
Metadataonly bench.yaml+manifestnow hpm6750_native_bridge/hpmtrue; CANnative
1MbpsUSBX5->CAN1HPM->CAN0HPM->PC, zero/byteexact/directionproof copied32.
128pytest1.19s/privatebuild1.30s pass; exactexecutedsource/scenario/models/plugin
and102inputSHA frozen PCstate/full-mission-32. No online runtimecodechanges.

AfterX5reboot firststartupfailed ROScreation because rmem_max425984<8MiB.
Initialfailedunitwasactuallyinactive; journal+startup-before-network.jsonsaved,
not mistakenmissionfailure orCANfailure. Restored16MiBceiling thenrestartedcore;
startup+preexecuteconfirmed image/IMU/lidar/FASTLIVO+RTKready4/CANzerofaultfresh.
No motionwasauthorized untilhealthpassed. PCworldnotresetduringthisstationary
startupwait; actualstartremainedwithin.11m originalspawn. Firstrecordcoversit.

UserexplicitlyrequestedPERSIST: addedconfig/99-agribot-hil-network.conf withONLY
net.core.rmem_max=16777216 andwmem_max=16777216. Installedroot0644at/etc/sysctl.d
onBOTHPC/X5 andsysctl-p applied. Bothactualvalues16MiB, cmpmatches; actual
systemd-sysctl--cat-config showsprofileincluded/onlyoccurrenceofbothkeys; no
conflictingoverride. Serviceactive/static,before sysinit.target/userstacks.
DidNOTrebootPC/X5tosimulateboot; no routing/defaultbuffers/Ubuntuversion changes.
prepare_network.shstilltemporary-onlyfallback; READMEupdatedinstallprocess.

At32terminal preservemission/corejournals+allrawPC BEFOREreset, authfalse,
SIGINTstoprecord32thenstopX5core (drainbothbeforefullrsync). Mustpass audit_mission
18 +fluency10 +gripper_audit7, realfinish3/2fruitstilltray/stow/nativeCANerror0/
no reverse/failedplans/>=2simsecstops/fingers<.25mm+actualcontactsunlocked.
Verify102SHAunchanged andcaptureactualRVizwindow xwd+ffmpeg; do notcapturewhole
desktoporforceGazeboview. NeedreportHPMfullpassonlyafterallactualaudits. UART0
querytimeoutstillunresolved/doesnotblockCANtest; nofirmwareflash ornewgoal.
Allsetupfiniteexecsdrainedexceptthecurrentdriving-runtime5secobserver(handle
willbeprintedbyexec), notanothermission. PCGazebo/viewmayremainafterterminal.

The following reconnect-only check is HISTORICAL:

**LATEST: HPM6750 reconnected; physical zero-motion bridge checks PASSED.**
PCUSB-CAN003000224856511520343739/can0 returned, explicitly initialized1Mbps
ERROR-ACTIVE/restart0 withserialguard. WiredPC192.168.1.1/X5192.168.1.102 restored;
X5rebooted, coreinactive andZQWL966960660237 stillby-idttyACM0. FireDAP recognized
USB1fc9:5601, serial1d4a21d76150fec8, CDC/dev/ttyACM0 andHID. ProbeUSBrecognition
does not prove targetJTAG/debug/flashcontents readback or UARTwiring.

Userprovidedvcu_configurator_src_20261106.zip extractedunchanged privately to
PCstate/hpm6750-firmware-20261106. Readsource+BUILD_AND_FLASH: CAN1computePE31/
PE30external3.3Vtransceiver, CAN0machineonboard; nativeAckermann1Mbps0x181->
0x101/102/103, tracked250kbps0x514->0x532/533/534, classic8Bwhitelist, noreplay.
ELFhasactualruntimebridge_poll andtelemetrysymbols; BINsha9d4428055cfb2b33d69053f4ee327216e1a7971c173f8fe4592f8338a435f17b.
Configuration/A-Bstore, motion/profile, nativebridgehosttestscompiledfromthis
exactarchiveandpassed; notallFGMP/wholefirmwarebuild or targetmemoryverification.

Bidirectionalphysicaltest hpm-reconnect-20261106-can-{pc,rdk}-01.json PASSED:
X5sent3allzero0x181/PCreceived3; PCsent9stoppedfeedback/X5received9 exactordered
0x101/102/103andbytes. PCerrors0. Additional SYNCHRONIZED directiontest03 passed:
waitactualX5READYthenPCsent3zero0x181wrongdirection+9validfeedback; X5gotONLY9
feedback, no0x181, norepeatsin8sec. ConsistentwithrealHPMwhitelist/ports andnot
onecommonbypassbus. Evidencehpm-reconnect-20261106-direction-synchronized-03.json.
Earlierdirection01hadnoproducer;02readinessnotconfirmed; emptyresultsareNOT
acceptedhardwarefailureevidence.03isvalidreplacement, oldrawretained.

Remainingdiagnosticissue: FireDAP1152008N1UART0identity0x21/status0x20alltimeout
withzeroreceivedbytes (serial-01.json). NoVCFGwrite/configchange/flash/reset/
JTAGhalt ornonzeroactuatorcommand. CanconfirmCANpathworks, NOTtargetfirmware
version/hashormonitortelemetry. Useronlyrequestedinspection; noautoextrafix.
Allfiniteprocessesdrained, ZQWLchannelclosed, X5corestopped, PCauthpreviouslyfalse.
PCGazebo/viewremainat31; oldPCbridgeUSBsocketstaleafterunplugrequiresrecreation
beforefutureHILmotion. bench.yamlstilldirect/hpmfalseuntilnextauthorizedworkflow
adaptation; nofullmissionwith6750yet. Nooriginalfirmware/repo/Unitychanges.

The following offline-reconnect state is HISTORICAL:

**LATEST REQUEST: user is reconnecting HPM6750 and asks whether feedback arrives.**
No firmware/source changes, no motion or new mission. At15:24:06 PCkernel logged
USB1-10 disconnect; can0 disappeared and noCANadapter/6750diagnosticserial is
enumerated onPC. PC-X5wired NIC enx207bd2b013d1 isNO-CARRIER/unavailable andlost
192.168.1.1. X5SSH192.168.1.102 timeout andping3/3lost; receive-only check could
not connect (no remote actuatortest ran). Need PCUSB-CAN+PC-X5Ethernet restored
before real byte-exact bidirectional6750 test. Do not infer6750firmwarefault or
claimfeedback delivered. Current bench.yaml stilldirect/unvalidated onpurpose.
PC Gazebo/view retainfinal31scene butDDS logs missingwiredpeers; original bridge
socket will need explicitrecreation afterUSBreturns, not reuseitsstale socket.

**Run31 TERMINAL succeeded, user-amended1->2->3 scope independently VERIFIED.**
MissionunitMainPID0/active-exited/exit0. PCauthfalse confirmed; recorder31SIGINT
flushed, mirror31autoexited; X5corestop fullydrained before fullrsyncreports.
All18physicalaudit+10fluency+7gripperchecksPASSED; final3actualerror.230719m,
twofruit5/7stillinsideactualtray, batchstowed, bothlegsattempt1, CAN0errors,
no reverse/failedplans/>=2simsecunplannedstops. Actual20Hzfingerfeedback complete:
outbound3704samplesmax6.5577um, completion1874samplesmax.20074um; physicalhold
releasedbeforebothactualstemcontacts.96input/source/plugin/mapSHAallunchanged.
Bothjournalsnonempty andallrawreportsPCfull-mission-31. TrueRVizxwdconverted
rviz-completion.png showsoriginalfullmap/GlobalStatusOK/paths; usermanualzoom
andimageuncheckedpreserved, noforcedcamera. No claimcancelledhomelegpassed.

The following Run31-active section is HISTORICAL:

**LATEST USER AMENDMENT: "那就到3点算了". Home-return requirement withdrawn.**
Current route1->2->3 ONLY, finish original3[3,32.5,pi]; do not continue trying
the cancelled home turn. Keep the requested stablegripper and fulloriginalRVizmap.
Old persistentgoal text stillmentionshome and reads paused after userinterrupt;
that historical text does not override the latest explicit routeamendment.

**Run31 ACTIVE X5 agribot-hil-mission31.service PID722446, parent1237/systemd.**
PCmain/record31/view(report31)/mirror31+X5core active, PCauthtrue. Originalfresh
spawn/world/apples. StartupCAN+localization healthy; actualholdtrue ack and
outboundattempt1 following568points confirmed via liveunit/journal/physicalmotion.
Initial immediate observer read had no mission.json yet (startup); it was not a
missionfailure and no duplicate restart occurred. All finiteexecs drained.
96input/source/pluginbinary/fullmap/RViz hashes and exactpackage/scenario/models
frozenPCfull-mission-31. No online code/parameter changes planned.

Newdefault completion_via=[original3], finish3, finish_is_spawn=false, noStartgoal
or original4.127tests1.20s/build1.15s pass. No MPPI tuning/trackingprofile changes
were applied for the cancelled home return; previous tested baseline retained.
At31terminal preservejournals/rawPC, authfalse, stoprecordSIGINT andX5core,
copyreports, audit_mission+fluency+gripper_audit allrequired; fullRViztruewindow
capture via xwd+ffmpeg (GNOME fallback previously captureddesktop, not evidence).
Verifyactualfinish3,2fruitintray/stow, fulltravel fingererror<.25mm and realpicking
unlocked, no reverse/failures/stops,96hashunchanged. FullmapPCview mayremain.

30 terminalFAILED to returnhome near3 (MPPIoptimizer failed, later recovery
Smac no path). Allreports/raw/journals/96inputintegrity savedfull-mission-30,
authfalse/core/recorder/mirror stopped before31. Not a CANfault. Twofruit picking
passed; actual fingerfeedback proved stable beforefailure: outboundmax12.29um,
completion-to-firstfailuremax0.178um, complete20Hzdata; wholegripperaudit correctly
fails becausecompletionunfinished. Do not call30fullpass or claim homefixed.

The following Run30-active / home-return sections are HISTORICAL:

**Run30 ACTIVE, full new goal verification. X5userunit agribot-hil-mission30.service
PID708911,parent1237/systemd.** PCmain+record30+view.launch/report30+mirror30 and
X5core active, PCauthtrue. All finite setup execs drained; no online code changes.
96 hashes (source/config/CPPpluginbinary/fulloriginalPGM+YAML/RViz) + exactpackage/
scenario/models/pluginbinary frozen PCfull-mission-30. Fresh originalspawn/world.
Startup sensor/nativeCAN/FASTLIVOready healthy. Holdtrue physicalack recorded
beforeoutbound attempt1 following570points. Current car along firstrow.

Route source originals1/2/3 thenSPAWN XY(2,36); nooriginal4. Position-only final
remains; home APPROACHyaw pi/2 (not forcedoriginal0). Exacteast-arrival prospective
Smac failedmaxiterations and is preserved; northapproach passed31.890m,0reverse,
0changes in return-home-north-preflight-30.json. Natural northapproach is not a
changed final yaw requirement (nonewasrequired). Bothcompletiongoals3andhome
will be planned/followed asonecontinuouspath.

Actual physicalemptygripper probe PASSED lock/refuseclose, release/canclose,
rehold; maxerror1.2894e-7m vsold4mm. gripper-restraint-probe-30.json. Not full
drive proof.127pytest1.16s/privatebuild1.15s pass. Added privateC++holdplugin only,
original Gazebo/nativehardware/Unity unchanged. Fullmap actualRVizstartup image
rviz-full-map-startup.png VERIFIED original4000x4000 fullknownfield fits, camera
image and GlobalStatusOK. No Gazebocamera following. mirror30 readonlyrsync
feeds X5mission/harvestJSON toPCliveview, terminalresources autoexits; no controls.

At30 terminal: savejournals, SIGINTstoprecordflush, authfalse, copyreports; must
pass audit_mission,fluency ANDgripper_audit; verifyactualfinishhome after3, two
fruitsstillintray, stow, no reverse/failedplans/stops, drivefingers<.25mm and
released duringactualcontacts. SnapshotfullRVizwith30plans+Start. Check96hashes
unchanged. KeepgoalACTIVE until allverified. StopX5promptly afterfinitework;
RVizPCfullmap/savedpaths/camera canremain. No newgoal/commit/subagent/Ubuntuupdate.

The following pre-30 implementation/commissioning section is HISTORICAL:

**NEW ACTIVE GOAL:1->2->3->originalSPAWN, stable fingers during travel, RViz fullmap.**
29 already finished successfully at3; independent18/18 andfluency10/10 passed,
reports/rawPC/journals savedPCfull-mission-29. Recorder29 stopped, authfalse.
Do not wait old29PID or confuse its finish3 with the NEW goal's returnhome.

Current implementation ONLY independent HIL package: completion=[original3,
SPAWN(2,36,0)], finish_is_spawn, sequence[1,2,3,'start']; no original4 inserted.
New private PC C++plugin gripper_hold uses prismatic lower/upper=originalopen
during transit, restores original limits before picking, physical-state ack Bool.
Mission explicitly holdtrue beforedrive, false after parkingfixture, true after
stow; never force open heldfruit. Original physical contact/IK gates retained.
Old29 observed finger drift3.45..4.46mm, arm joint excursions much smaller.
Recorder now samples actual joint stamps20Hz and holdstatus; gripper_audit.py
requires eachdrive's physical error<.25mm + released during realstemcontacts.

127pytest1.20s and privatebuild1.32s passed. Native plugin built standalone via
scripts/build_sim_plugins.sh to privateplugins/libagribot_hil_gripper_hold.so.
Originalworkspace/hardware/Unity notchanged. Full original4000x4000 orchard
map is visualization-only; planning crop unchanged. view.launch.py will show
fullmap+live/saved nativepaths andfullwideview, Start marker after3, Topicfixed.

NOW PC main and X5core restarted for commissioning, PCauthfalse, NOmission30
running. Onlyfiniteexec99024 is physical finger lock/release/rehold probe
gripper-restraint-probe-30.json (emptygripper, no chassis motion). Need drain.
Then native read-only path_preflight completion3->home, verifyforward/noobstacle.
Ifpassed freshresetoriginalspawn/world/core, freeze all new sources/plugin
binary+fullmap/config, startfull30+independentrecord+view.launch/report30.
Full30 must pass audit_mission+fluency+gripper_audit AND realRViz fullmap screenshot
beforecompletegoal. Preservefailurefirst. Currentgoalactive/incomplete.

The following Run29-active section is HISTORICAL:

**Run29 ACTIVE: user changed route to1->2->3, then explicitly said reach3 FIRST.**
Stop at original point3[3,32.5,3.14159], do NOT automatically go4 or original
spawn; returning to spawn is a later user step, not included in this run.
Same exterior5/7 doublepick, pickupXY.10/yaw.15, lowRTK and forwardonly remain.
scene generator now uses the original firstTHREE points; completion_via=[3],
finish=3. Native algorithms and safety checks unchanged. 120pytest1.31s and
privatebuild1.47s passed; source+generatedscenario synced and verified onX5.

X5 userunit agribot-hil-mission29.service PID685923,parent1237/systemd.
PC main+record29 and X5core active, PCauthtrue, original scene/spawn reset and
freshfruit. NativeCAN/sensors/FASTLIVOready startupchecks passed. 83 input/source/
asset hashes + exact source/launch/scripts/scenario/models frozenPCfull-mission-29.
All finite setup execs drained. No online changes planned; no persistentgoal.
Atterminal preservejournals/rawPC/reports, stoprecorder SIGINT/core promptly,
authfalse, copyreports and audit_mission+fluency for FINISH3. If failed preserve
before any reset. Keep manualcamera, no reverse/home/point4 inserted.

The following Run28-terminal section is HISTORICAL:

**Run28 TERMINAL succeeded and independently VERIFIED. No mission/core/recorder
running; PC Gazebo remains open at the final scene, motion_authorized=false.**

The approved pickup XY0.10m/yaw0.15rad is now fully exercised with the deck-
mounted RTK model. Native outbound ANDcompletion succeeded FIRST attempt, no
failed/recovery plans, original1/2/3/4 in order, finish4 independent error0.164462m.
Final fused pickup error to the actual Smac endpoint0.098456m. No reverse plans
or native CAN wheel velocity, no >=2sim-sec unintended navigation stops.
Fruit5/7 true bilateral stem contact (TCP0.0861/0.1886mm), detached, deposited,
still in the physical tray at finish. One batchstow, parkingreleased. CANerrors0.
Actual picking span224.7->295.9=71.2simsec (different pose/plan from26).

Full rawPC/mission/harvest/journals/startup/runtimeprovenance and frozen exact
code/config/assets remainPCfull-mission-28. Independent audit18/18 and fluency
10/10 PASSED. All83 source/config/asset hashes checkedunchanged after the run.
No online modifications. This is still Gazebo known-object-pose manipulation
and kinematic arm actuation, not real-arm force/vision picking verification.
Local119pytest/build passed before28. No originalhardware/Unity/Ubuntu changes,
gitcommit or subagents. OldUSB physical disconnect cause remainsunknown.

The following Run28-active section is HISTORICAL:

**Run28 ACTIVE: user requested validation with pickup XY0.10m/yaw0.15rad.**
X5 unit agribot-hil-mission28.service PID664860, parent1237/systemd. PC main+
record28 and X5core active, PCauthtrue. Fresh original world/spawn/fruit; actual
following outbound attempt1 (570points), runtime profile confirms0.10/.15.
CAN0errors, sensor data+FASTLIVO/RTK ready healthy. Original1/2/3/4 stop4, exterior
5/7 single pickup, low RTK, no reverse/home, manual camera unchanged.
83 hashes and exactsource/launch/scripts/scenario/models/RTKassets frozen PC
full-mission-28. No online code/parameter changes planned. All finite setup
execs drained. No new persistentgoal; old goal remainscomplete from26.

At28 terminal preservejournals/rawPC/reports before any reset; stoprecord28
withSIGINT, stopX5core promptly, PCauthfalse. CopyX5reports then audit_mission+
fluency must bothpass to claim full validation. If failure preserve first,
diagnose actualpose vsrealnativepath endpoint, do not silently relax safety.

The following approved-but-not-yet-run tolerance section is HISTORICAL:

**User approved relaxing ONLY pickup XY tolerance: now0.10m (was0.06m).**
scene.py generates0.10m in mission.json; Mission.run fallback is also0.10m.
Pickup yaw remains0.15rad; final-point half-wheelbase tolerance, route, motion
limits and arm collision/contact checks unchanged. Local119pytest passed1.24s,
privatebuild1.25s. HIL source and generated mission synced to X5 and the actually
imported source/manifest verified there. No full rerun of this new tolerance yet.
Mission/core/record remain stopped; PCauthfalse, Gazebo retains stopped27 scene.
Do not restart or claim complete validation without the next run.

The following Run27 diagnosis before the approved tolerance change is HISTORICAL:

**Run27 TERMINAL cancelled after outbound pickup convergence failure. No mission,
core or recorder running; PCauthfalse and Gazebo retains the stopped scene.**
All27 reports/rawPC/journals saved locally. First outbound failed path progress
watchdog, second plan had started before user reported the problem and we stopped.
No fruit picked; do not claim27 full success. CANerrors0, native feedback healthy.

Exact diagnosis: pickup XY tolerance0.06m, yaw0.15rad. Native Smac's discretized
path endpoint[30.57500047,31.82500006] differs from requested[30.57,31.83].
Closest recorded fused pose to the ACTUAL endpoint was atsim239.6:
[30.56412847,31.88522168], distance0.061195m, yawerror0.06448rad. Thus it missed
the6cm XY gate by1.195mm, despite sufficient yaw, then overshot/circled as an
Ackermann forward-only vehicle cannot correct laterally in place. This is not
a proven CAN/RTK sensor fault. No parameters changed yet. User asks whether the
pickup precision is too strict; propose10cm XY, retain0.15rad yaw and actual-pose
IK/collision/contact checks before any follow-up implementation/validation.
115tests/build and26 prior success remain prior-version evidence.

The following Run27-active section is HISTORICAL:

**Run27 ACTIVE after the user's request to rerun the accepted deck-mounted RTK
model. X5 unit agribot-hil-mission27.service PID655502, parent1237/systemd.**
PC main+record27 and X5 core active; PCauthtrue. Original scene/spawn reset,
fresh fruit, actual CAN feedback and FASTLIVO/RTK ready checked before starting.
Current route fixed1/2/3/4, exterior single stop5/7, finish4, no reverse/home.
RTK leverarm Z0.264759521484375 on PC GNSS and X5 fusion, both TF frames lowered.
83 updated source/config/assets hashes + executed source/launch/scripts/scenario
snapshots frozen PCfull-mission-27. No online code changes planned. Prior goal
complete from26; 27 is a new normal user rerun, not a new persistent goal.

Monitor27 to terminal; preserve journals/reports, stop recorder using SIGINT,
copy X5 reports and run audit_mission+fluency. Stop X5 core promptly after the
finite mission; PCauthfalse for display. Do not claim success from26 for27.
All finite setup execs drained; mission itself is independent user-systemd.
Current USB by-id resolves ttyACM0, not assumed ttyACM1; driver uses by-id.

The following Run26 terminal / pre-27 display section is HISTORICAL:

**Run26 TERMINAL succeeded. No mission/core/recorder running. PC Gazebo is
reopened at the original spawn with motion_authorized=false for display.**

26 outbound/completion both succeeded on attempt1; original1/2/3/4 observed in
order, finish4 independent error0.222162m. Both5/7 physically detached by bilateral
stem contact (TCP0.6184/0.4700mm), deposited and retained in the real tray; one
batch stow. CANerrors0, no reverse planned/measured, no >=2sim-sec unintended
navigation stops. Independent audit18/18 and fluency10/10 passed. Reports,
journals, raw PC recording and executed source snapshots savedfull-mission-26.
All77 source/input hashes matched before any subsequent editing. Actual picking
span327.0->386.2=59.3simsec versus old21 214.7; targets/strategy also changed.

Audit initially compared the mission pose at509.2 with recorder tail620.943,
which included later idle physics. The original negative audit is preserved as
independent-audit-before-time-alignment.json. Audit now compares the nearest
same-time sample, requires <=0.5s gap, and rejects a mismatch at that time even
if later samples agree. Actual sample509.151 agrees within0.000091m; 3 new
regressions cover late idle, missing end telemetry and wrong same-time pose.
The recorded mission data has not been edited or cropped.

After26 user asked to remove both RTK tall posts, then lower the floating
antennas. New independent HIL rtk_visual.py removes Cube1/Cube2 primitives,
lowers the complete antenna frames520mm, compensates the retained bottom rail
so it stays on the chassis; short antenna feet meet rail aroundbaseZ0.135m.
Master/secondary URDF frames, SDF visual, fusion leverarm, simulatedGNSS manifest
and georeference are consistent; originalCAD and truehardware untouched.
Current115pytest pass1.19s, privatebuild1.16s; source+newscenario synced toX5
but core remains stopped. Static display only; new lowered version has NOT had
a full mission rerun, do not attribute Run26 to this later geometry version.
No commit/subagent/Unity/Ubuntu changes. Previous USB physical cause stillunknown.

The following Run26-active section is HISTORICAL:

**Run26 ACTIVE: X5 user-systemd agribot-hil-mission26.service PID639311,
parent1237/systemd. PC main+record26 and X5core active, PCauthtrue.** All finite
exec handles drained. 77 updated inputs/source hashes + exact snapshots frozen
PCfull-mission-26. No online changes planned. Goalactive/incomplete.

User rejects dense canopy31.8firststop; source now ONE known exterior stop
[30.57,31.83,pi], targets5 thenNEW7 on adjacent OUTSIDE branch, no extra stop
under canopy. New7actualBVHhit[30.3899994,31.4200001,.7292885]; proof
exterior-easy-fruit-bvh-candidates.json replaces abandoned31.57proof in inputs.
Two apples same stop -> stowonce, phasesoutbound/completion, original1/2/3/4
and finish4 preserved, no reverse/return1/home. StrictXY.06, pickupyaw.15,
finalstopyawignored. Same physically supported tray and faster arm remain.

Native new preflight passed BOTHfruit fullIK/OMPL/approach/carriedretreat AND
initial stow geometry for THREE poses: nominal(.04baseheight), +(.04x,.03y,.15yaw)
and -(.04x,.03y,.15yaw) at.03baseheight. These explicitly model observed stopping
error, not just nominal pose. Allshort/no winding, completecollisionchecks.
Reports exterior-pick-{nominal,positive,negative}-01 copiedPC. Native routes
54.384/32.612m passed allforward0changes. Source108pytest.64s/build1.17s.
Freshoriginalspawn/world/apples after25 failure, no physical preflight movement.

26 terminal: stoprecordflush/copyreports/audit_mission ANDfluency bothmustpass,
actualfinish4/no reverse/2fruit5+7intray/one batchstow, actual manipulation span
vs21baseline214.7simsec (strategy/targets also changed, not singlevariable).
Tray actualsupport screenshot22 alreadyverifiedsamegeometry; capture actual
exterior picking view if possible without locking camera. StopX5promptly,
PCauthfalse for display. Onlycompletegoal after ALL requirements directly
verified; anyfailurepreservefirstthencontinue. No originalhardware/Unity edits,
gitcommit, subagent orUbuntu changes. USBphysicalcauseunknown, currentlinkworks.

The following25terminal/preparation section is HISTORICAL:

**25 TERMINAL failed before picking; NO mission running.** Actual first stop
31.8/31.9 had branch collision with stowed link4 (depth.000165m), despite nominal
preflight. User explicitly rejects this dense-canopy place. All25reports/journal/
PC record saved, authfalse; X5core stopped. PCstop70411/build7766 may need drain.
Do not resume25PID627436 or claim success. Goalactive/incomplete.

Now choosing ONE exterior stop around the actually successful24 pickup location,
PARK[30.57,31.83,pi]. New7anchor moved to actual wood[30.3899994,31.4200001,
.7292885], near5 on the outside branch; not the abandoned31.57dense site.
Proof exterior-easy-fruit-bvh-candidates.json. Source pickup targets5/7 in same
stop, expectedphases outbound/completion, stowonce. #6edge-of-reach notselected.
Strict pickupXY.06 retained; yaw tightened.15 (final goal still positiononly).
Helper now supports x/y/yaw offsets to preflight measured parking deviations.
Next after build/reset: native route + full approach/carriedretreat for nominal
AND +/-observed pose variation before full26. No full26/preflight yet. Current
108tests lastpassed before the latest PARK/yaw adjustment, rerun before26.
Do not keep testing nominal-only poses under thick canopy.

The following Run25-active section is HISTORICAL:

**Run25 ACTIVE: X5 user-systemd agribot-hil-mission25.service PID627436,
parent1237/systemd.** PC main+record25 and X5 core active, PCauthtrue. All finite
exec handles are drained. 77 source/config/input hashes + exact source/launch/
scripts/nav2 frozen in PCfull-mission-25. No online changes planned. Goalactive.

24 TERMINAL failed ONLY minimum fruit count: outbound AND completion each
succeeded FIRST attempt, finish4 actualerror.221165m, only5picked (6unreachable
at actual pose). Records/journals/PC recording copied; no claim fullacceptance.
Actual pose changed6target to[.408,.455,.635], too near workspace edge. User
allows easier fruit, so replace6 rather than force weak paths. Do not wait24PID.

New25 picks7 then5 at TWO easy stops, both along forward direction between fixed
2/3: first[31.8,31.9,pi], second[30.53,31.83,3.19665505]. New7anchor
[31.5699997,31.5499992,.77120227] is actual original wood underside BVH ray hit;
generated proof fruit7-bvh-candidates.json includes sourceDAE SHA. Config registry
has easy_pickup_candidates; seven dynamic fruit. #6 stillhangs, notselected.
Vehicle moves about1.25m forward between stops, so arm MUST stow before that
movement. Two stows are necessary, not redundant batch retraction. Preserve
original1/2/3/4 coordinates and finish4; nohome/loopback/reverse.

Native preflight passed ALL five checks: outbound53.181m, reposition1.251m,
completion32.563m, zero reverse/changes; fullIK/OMPL/approach+carriedfruitretreat
for7 and5 at own stops. Approach.844/.865simsec, retreat1.677/1.684simsec.
No physical preflight movement. Source108pytest passed.70s/privatebuild1.15s.
Mission breaks an exhausted stop's target set without redoing selection. The
expected phases now outbound/pickup_reposition/completion; fluency/audit read
manifest phases dynamically. Helper --stop-index, path_preflight --pickup-index.

At25 terminal stoprecordflush/copyreports/audit_mission THENfluency (BOTHpass),
verifyactualfinish4/zero reverse/2fruit7+5intray/eachstowed before moving. Measure
actual picking span (including shortrelocation) vs21 baseline214.7simsec; note
targets/strategychanged, not singlevariableablation. Tray support actual22
screenshotalreadyviewed and samegeometry. StopX5 promptly/PCauthfalse display.
Only complete goal after all requested direct evidence; preservefailures first.
No originalhardware/Unity/Ubuntu changes, no git commit/subagents.

The following Run24-active section is HISTORICAL:

**Run24 ACTIVE: X5 user-systemd agribot-hil-mission24.service PID615312,
parent1237/systemd. PC main+record24 and X5 core active, PCauthtrue.**
No finite exec sessions remain. 75 updated source/config/input hashes frozen
with exact source/launch/scripts/nav2 in PCfull-mission-24. Includes new
pickup_preflight.py. No online changes planned. Goal ACTIVE/incomplete.

Latest108tests passed.67s/privatebuild1.16s. Fresh world/original spawn for24,
no reused fruit. Target5/6 on actual lower branches; PARK now[30.53,31.83,pi]
using23 measured stopping lane. Strict.06 pickup tolerance retained. Source
rejects shortCartesian joint winding>pi as well as incomplete fraction/jumps.
Sequential native prospective preflight (5removed to tray before6) PASSED BOTH,
approach durations.888/1.567simsec, retreat1.752/2.743simsec. Fully restored
actual scene afterward; no physical commands. Simultaneous-fruit preflight02
was negative for6 and remains saved, not labelled passed. Sequential helper
correctly models5removed,6now has17approachpoints instead of322winding.
Native fresh routes passed:outbound54.445m/completion32.562m,allforward/0changes.

24 at terminal: flushrecord/copyreports/run audit_mission AND fluency (bothpass)
then verify actual fixedpoint4[3,29], no reverse, exactly1stow after twofruit5/6,
both physically intray, actual arm elapsed vs21 baseline214.7simsec. Note targets
and strategy also changed, not a single-variable speed ablation. Tray support
direct actual screenshot22 already viewed and geometry unchanged. StopX5
promptly after finite work/PCauthfalse for display. Goalcomplete ONLY with all
requested evidence, otherwise preserve failure and continue. No original
hardware/Unity/Ubuntu changes, no git commit/subagent; priorUSBcauseunknown.

The following Run23/preflight62799 section is HISTORICAL:

**Run23 TERMINAL failed before picking; no mission currently running.**
PCauthfalse, record23 flushed, X5reports copied. First follow stalled at the new
precision park: actual/fused Y stayed31.825..31.83, requested31.68; minimum
fused XY error.14019m, then forward-only recovery circled and ultimately failed
startlethal near[32.55,31.96]. This was NOT a CAN disconnect. Preserve23 negative
evidence; do not claim smoothness or resume its old mission service603785.

Current source PARK changed north to[30.53,31.83,pi], keeping strict.06m pickup
position tolerance. This follows measured unobstructed stopping lane, not a
relaxed collision or contact gate. Fruit5/6 remain real secondary anchors.
Current ONLY finite exec62799 is native READ-ONLY pickup_preflight at this new
park, X5 core active for services; PC remains authfalse at old physical stopped
pose. New reusable pickup_preflight.py uses prospective geometry/virtual carried
fruit ONLY and restores actual planning scene; no arm/Gazebo/CAN movement.
Wait/drain62799 before deciding next action. All other finite handles drained.

If both fruit feasible at new north park: fresh original-world reset and native
route preflight, freeze ALL updated sources including new preflight module,
then next full24. No full24 started yet. Current106tests lastpassed before
PARK/new module; rerun tests/build before24. Goal remains active/incomplete.
Tray support actual screenshot22 valid; arm speed actual end-to-end still not
verified. Need full1/2/3/4 finish4, bothfruit/stow/speed/audit+fluency for complete.

The following Run23-active section is HISTORICAL:

**Run23 ACTIVE: X5 user-systemd agribot-hil-mission23.service PID603785,
parent1237/systemd.** PC main+recorder23, X5 core active; PCauthtrue. No finite
exec handles remain. Monitor unit/journal and reports/full-mission-23/mission.json.
74 inputs+exact executed-source/launch/scripts/nav2 frozen in PCfull-mission-23.
No online changes planned. Active GOAL remains unfinished: stop at fixed4,
physically supported tray, faster arm. User additionally allows easier fruit.

22 TERMINAL failed after first fruit detached: strict retreat fraction.941 was
rejected, not executed. Logs/reports/PC record retained. Its real Gazebo screenshot
full-mission-22/gazebo-supported-tray.png was viewed: real tray posts connect
to chassis, no floating gap. New full mission must still pass, not splice22.

New easier fruit5/6 are on actual secondary branch BVH anchors x30.30/30.22,
y31.44/31.36,z.7263/.7183, both70mm stems. Old1..4 stillhang. New PARK
[30.53,31.68,pi], still between fixed2/3. Both target stems are lower than old1/2.
Retreat tries complete collision-checked offsets(.05,.03),(.04,.02),(.04,.04),
(.035,-.007); partialfraction remains rejected. Added3retreatfixtures:106tests
passed.63s, privatebuild1.15s. Native route preflights passed:outbound54.529m,
completion32.594m,0reverse/0direction changes. Source only independent HIL.

Native prospective manipulation preflight BOTH5/6 passed fullIK+approach+OMPL
and retreat WITH virtual attached-fruit collision geometry. No physical arm or
Gazebo attachment requests were sent; executed=false. 5uses5cm/3cm retreat;
6rejects5cmthen passes4cm/2cm. Planning scene restored to real original-spawn
geometry after preflight. Report easy-fruit-approach-retreat-preflight-01.json
copiedPC, along with route preflights. This is not full execution acceptance.

At23 terminal stoprecordflush/copyreports/audit_mission+fluency BOTHrequired;
actual finish4/no reverse/one stow/fruit5+6 in tray, compare actual manipulation
elapsed to21 baseline214.7simsec. Supports already directly rendered in22; same
support geometry retained. StopX5 promptly/PCauthfalse for display. Goalcomplete
ONLY after all requested parts directly verified. Preserve any failure before
reset. Old USBdisconnect physicalcause unknown;180s zero probepassed, noROSbypass.

The following Run22-active section is HISTORICAL, not currently running:

**Current active GOAL: original fixed1->2->3->4 ONLY, stop at4; physically
supported tray; faster arm planning/execution. Goal is not complete.**
Run22 has started in X5 user-systemd agribot-hil-mission22.service (get PID from
unit/provenance), PC main+record22 and X5 core active, PCauthtrue. 74 new hashes
and exact source/launch/scripts/nav2 frozen in full-mission-22. No online source
changes planned. All build/stop/preflight finite sessions drained; only latest
readback/provenance exec49822 may need draining. Monitor service/reports, not an
attached mission SSH. Original spawn/world/apples reset for22.

New103tests passed0.66s; privatebuild1.15s. Native cold outbound56.638m and
prospective completion30.362m passed, zero reverse segments/direction changes.
Manifest completion_via=original points3/4, finish=point4[3,29,0], phases
outbound/completion. Mission and independent audit now validate requested finish,
not original spawn; no loop-back1/home. Static DUBIN + MPPI vx_min0 retained.

Four tray supports, centers x=-.10/.015,y=+/- .105,z=.155, size.024/.024/.09,
attach below floorz.2. Original actual OBJ Blender global-BVH raycast surfaces:
rear mounts z.13479036, frontz.11275208, supportbottom.11 intersects both. Same
visual/physics/MoveIt boxes on base_link, no floating fixture/fruit teleport.
Arm joint limit.9rad/s,acc1.5; OMPLscales.8/.8, near-stem Cartesian.35/.5,
2mm collision sampling and .7mm contact gate retained. Remove forced2s padding
per motion, use .15s scheduling; collect real plan/execution timings. Mission
stows ONCE after batch rather than after each fruit; all paths still collision
checked. Need actual fruit/deposit/stow timing improvement and support screenshot.

USB re-enumeration probe52249 drained:180.022s,3222zero sends,3242 valid24B/XOR
feedback cycles,maxgap.46848s,0errors, passed. USBcurrently by-id->ttyACM1;
native driver uses correctby-id. Physical root of prior disconnect unknown;
don't claim hardware permanentfix. Goal blocker count is not3; meaningful new
software/preflight progress made. Original hardware/Unity untouched, no commit.

At22 terminal: stoprecordflush/copyreports/audit_mission THENfluency (bothmust
pass), check actual finish4/no reverse/one stow/2fruit still intray, compare arm
elapsed to21 baseline329.8->544.5=214.7simsec. Inspect actual rendered tray
support. Stop X5 promptly/PCauthfalse after finite work. Update goal complete
ONLY after all three requested parts have direct evidence; otherwise continue.

The following cancelled21/probe-active section is HISTORICAL:

**Run21 is TERMINAL cancelled because the X5 USB-CAN physically disconnected
and re-enumerated. No mission or X5 core is running. PC authfalse; recorder21
flushed and X5 reports copied.** Native kernel proof: usb1-1.4 device66 disconnect
at uptime64188.856753, new device67 at64189.215952, ttyACM1 instead of ttyACM0.
Native chassis reported write(ZQWL CDC): Input/output error. USB power/control=on,
so do not blame autosuspend. Exact physical cause (cable/power/manual unplug)
not yet known; asked user whether the cable was touched. by-id path remains
correct. Do NOT add ROS chassis-command bypass or silently resume the mission.

Before the disconnect,21 outbound passed on attempt1, both apples1/2 physically
picked/deposited, return first follow reached near actual[4.994,32.630]. Both
actual native plans have zero reverse segments:56.605m/113.862m. This does NOT
certify a complete return or fluency. All interrupted/failed evidence retained.

Current ONLY finite diagnostic is SSH exec52249:180-second zero-speed native CDC
to PC CAN continuity probe using the re-enumerated by-id device, PC authorization
still false. Output X5reports/full-mission-21/zero-can-continuity-after-reenum.json.
Wait/drain it before ending. Other finite sessions are drained. Current source
still98 tests, static forward circuit, no online changes. Next decide using
actual probe result + user cable response; do not relabel cancelled21 as passed.

The following Run21-active paragraph is HISTORICAL:

**Run21 is ACTIVE: X5 systemd unit agribot-hil-mission21.service, PID575295,
parent1237/systemd.** PC main+recorder21 and X5 core active; PC authtrue for this
simulation. All preceding finite tool sessions are drained. Do not wait on an
old SSH session; monitor mission.json/journal. New98tests passed0.68s, private
build1.29s. 74 input/source hashes + exact executed-source/launch/scripts/nav2
are frozen in full-mission-21. No online changes planned.

User explicitly requests NO REVERSE and original fixed1->2->3->4->1. Original
first four XY loaded directly from existing Ackermann waypoint YAML. Pickup
between2/3; return continues3/4/1 then original spawn facing west. Native Smac
DUBIN, MPPI vx_min0/forward preference. Native read-only preflights BOTH passed:
outbound56.646m and return113.657m; zero reverse segments/direction changes.
This is only prospective evidence; not yet full-run acceptance.

At terminal stop recorder/flush, copy reports, audit_mission then fluency; both
must pass. New checks include ordered return circuit, zero planned reverse and
no negative native CAN wheel velocity. Stop X5 promptly after finite work due
old long-idle LIVO memory growth. PCauthfalse for safe display. If failure,
preserve reports/journal before reset. No original hardware/Unity edits/commit.

The following stopped/preflight-pending paragraph describes the phase before21:

**User rejected reverse driving and requested the original fixed 1->2->3->4->1
circuit. No mission is running now. X5 core stopped, PC motion_authorized=false.**
Run20 is TERMINAL failed: outbound first attempt and both fruit picks passed;
return first follow failed near actual[6.608,36.329], then four native Smac
requests rejected the start as lethal. Reports, independent recorder and terminal
cloud/IMU/global-grid evidence retained in full-mission-20. No claim of fluency.
Its all-reverse return is explicitly rejected, not the next baseline.

Only HIL source changed: load original first four points from
agribot_ackermann_mppi/config/orchard_waypoints_ackermann_smac.yaml, keep pickup
between points2/3, return via3/4/1 then original spawn facing west. Static Smac
DUBIN and MPPI vx_min0, forward preference enabled. No original vehicle/Unity
files modified. New98tests passed0.68s. Native forward-circuit preflight and full
run are still pending. Need regenerate/sync/restart safely, preflight before
motion; do not treat Run20 as active or restore its reverse route.

The following Run20-active paragraph is HISTORICAL, not currently running:

**Fluency validation Run20 is now ACTIVE as X5 user-systemd unit
`agribot-hil-mission20.service`, MainPID558733 (parent1237, comm=systemd).**
It is NOT owned by an attached SSH session. Launch SSH exited normally and a
separate connection verified the same live PID/cgroup/domain73; provenance saved
in X5 `reports/full-mission-20/runtime-provenance.json`. No finite mission exec
handle should be waited on. Monitor unit/journal and mission.json instead.
PC main/recorder20 and X5 core active; PC motion_authorized=true for this run.
World reset, original spawn retained, no fruit reused. 73 new input/source hashes
and exact source/launch/scripts/nav2 frozen in PC `full-mission-20/`.
Latest93 tests passed1.01s, private build1.49s; no online changes planned.

Read-only actual cold-spawn outbound now PASSED:56.6448m,0 direction changes,
all forward. Earlier virtual outbound failed in the old parked scene; it remains
negative evidence, not mislabelled success. Prospective return passed56.6654m,
0 changes/all reverse. Two-fruit MoveIt check at new west-facing PARK passed.
No change to original sensors/map pixels, aisle via XY or vehicle collision bounds.
Run20 first outbound native path597poses followed after .73s from Mission start
(native startup/preflight already complete). This is not yet actual fluency pass.

At terminal: stop recorder to flush, copy X5 reports, run audit_mission THEN
fluency. Require BOTH passed; zero failure replans and no>=2sim-second interior
stop. Keep old18 physical pass separate from new smoothness requirement. Stop
X5 core promptly after finite work, PCauthfalse for safe display. If a service
failure occurs, do not restart/reset before preserving report/journal.

The following describes the earlier diagnostic phase, no longer live:

**Latest user request is smooth continuous operation, not just eventually finishing.**
Run19 is TERMINAL failed because the Codex daemon/server restart closed the
attached SSH stdout: Mission.event print raised BrokenPipeError. Old finite27936
is gone. It had reached near the old parking pose but picked no fruit. Recorder19
flushed and reports copied. No automatic repeat has been started since failure.

Current work is an explicitly requested fluency fix, not a reopened old goal.
PC scene and X5 core are active, **motion_authorized=false**. World freshly reset.
Current finite **read-only cold outbound preflight SSH exec34254** is active.
All preceding finite diagnostic/build/stop sessions are drained.

New source (HIL package only, not original hardware/Unity): PARK now
`[28.35,31.85,pi]`, allowing forward west-facing approach after the unchanged
original aisle vias. Previous east-facing park forced an extra180-degree turn
inside the narrow row. Native MoveIt prospective check for the new pose passed
both apple1/2 with full branch/fruit/arm collision checks; report
`west-parking-fluency-preflight-01.json`. Prospective return Smac passed with
zero direction changes,56.665m. Its earlier virtual outbound check failed10s;
do not claim all routes passed. Now checking from an actual fresh original spawn.

Smac reverse1.05/change2 are STATIC for both phases. Return no longer rebuilds
its heuristic tables through an online parameter update. Broken stdout no longer
aborts Mission.event; storage errors still propagate. New run_mission.sh will be
launched as an X5 user-systemd service, independent of SSH/Codex stdout.
New independent fluency checker requires full HIL audit, zero failed navigation
attempts, one FollowPath per phase, complete actual velocity telemetry and no
interior stationary interval>=2sim seconds; pickup and1.5s endpoint ramps excluded.
Applied to accepted18 it correctly FAILS fluency because of3 replans, despite
the physical acceptance being passed. Latest93 tests passed1.10s; private build
passed1.49s. Bash syntax passed. No full fluency run20 has started yet.

After cold preflight passes, freeze new source/config for20, start independent
recorder, authorize PC and launch the mission in an X5 user unit (not attached
SSH exec). Do not call it smooth unless BOTH independent-audit and fluency pass.
Stop X5 after finite tests to avoid old long-idle LIVO memory growth.

The following run19-active paragraph is HISTORICAL, not currently running:

**User-requested repeat Run19 is ACTIVE, finite SSH exec27936.**
This is a new explicit rerun after the earlier persistent goal was completed;
do not resume/recreate that goal or reuse the old run18 session. PC main,
recorder19 and X5 core are active. Vehicle/world/apples reset to original spawn.
All70 source/config/input hashes match the accepted18 baseline exactly, frozen
with source/launch in `full-mission-19/`. No code or parameter tuning planned.
Recorder timeout3600s. Mission requires minimum2 / maximum2 fruit and return.
Do not mark this repeat passed merely because18 passed. Monitor27936 to terminal,
then stop recorder to flush, copy X5 reports, run independent audit, stop X5
promptly (known long-idle LIVO growth) and set PC motion_authorized=false.
Keep Gazebo for viewing after completion. No other finite HIL sessions are live.

The following safe display/result state describes18 BEFORE the requested reset:

**Run18 succeeded; finite exec51144 exited0 and is drained.**
`full-mission-18/independent-audit.json` passed all16 checks against the actual
mission, harvest and independent PC recording. Source/input SHA are recorded.
Actual mission return error .2047796800m; independent final error .2164531608m.
Two distinct apples1/2 remain physically inside the tray; arm is stowed and
parking released. Both finger pads contacted each stem before its actual tree
detachment, gravity deposit and return. TCP errors9.6029e-6 /4.2042e-5m.

Original spawn[2,36,.1325,0] was retained, original aisle vias were observed in
order (nearest actual distances .514/.511/.819m), no truth-pose localization or
CAN bypass. Continuous actual trace max observed step .076503m. CAN errors0,
18681 nonzero commands,72879 feedback. LIVO peak2346664KiB, minimum available
RAM3830152KiB; mission sampler duration about1312.5wall seconds. This does not
prove the old long-idle memory-growth problem is solved.

Autonomous recoveries are explicit: outbound attempt1 failed then2 succeeded;
return attempts1/2 failed then3 succeeded. Thus the full mission passed with
three automatic failure replans, not zero-failure tracking. No online source,
controller or scenario changes were made during18. Tests83 passed1.09s.

Current safe display state: PC Gazebo main remains active, CAN bridge parameter
**motion_authorized=false** confirmed through its native parameter response.
Recorder18 stopped/flushed; X5 core stopped/inactive, RAM available6330MiB.
No navigation/harvest task or finite exec is live. Actual home screenshot
`full-mission-18/gazebo-home-two-fruit.png` was viewed and shows both fruit;
`gazebo-picking.png` is valid. `invalid-window-capture.png` is a mistaken desktop
capture, NOT evidence. Do not reset this display scene unless a new run is wanted.
Source remains only the independent untracked HIL package; no original hardware
or Unity code edited and no git commit. HPM6750 remains bypassed as requested.

The following describes18's historical startup, not live processes:

Run18 started as finite SSH exec **51144**, original-spawn fresh start.
PC main/recorder-18 and X5 core were active. 70 input hashes, exact source and launch
are frozen in `full-mission-18/`. Latest **83 tests passed, 1.09s**. No online
changes planned. All previous finite sessions, including return2719, are drained.
Native return penalty profile reverse1.05/change2 is restored in Mission and
reverts through atomic parameter API after return. Original pi/0/0 return headings
remain independent of east-facing PARK. New IMU-level height filter, continuous
stationary confirmation and fresh-clock settling are retained.

`return-commissioning-17` succeeded on its FIRST actual follow, no failure replan.
Physical final XY error **.1322669208m**, both fruit centers remain in the tray
[-.07994,-.105,.245] and[-.00513,-.105,.24499]. Real home screenshot is retained,
viewed and shows two fruit. This is only segmented return debugging, NOT full
original-start acceptance. It substantiates restoring the turn-change penalty
now that the separate false-ground problem is fixed. X5 was promptly stopped,
then whole PC world reset for18; no picked fruit was reused for acceptance.

The following describes terminal17 and its earlier commissioning startup:

**Run 17 is terminal failed**, finite exec79511 drained. It completed original
outbound on attempt7 and physically picked/deposited apples1/2, with bilateral
stem contacts and TCP errors3.1975e-6/7.4407e-6m. Both settling checks passed and
parking was released. Return passed the formerly false-ground-blocked section
and both return vias, but failed after8 attempts near `[40.3494,35.0896]`.
Native CAN errors0 / 32076 nonzero /119982 feedback. PeakLIVO3614036KiB. Independent
record duration2222.77s flushed and retained; real two-fruit screenshot available.
This is not a full mission pass and may not be spliced with a separate return.

The finite **return commissioning SSH exec2719** started from this failed
scene, with PC main/record-return-17 and X5 core active. Output
`return-commissioning-17/`, explicitly not original-spawn full acceptance.
Return uses native Smac reverse_penalty1.05/change_penalty2.0 temporarily through
the ordinary atomic parameter API; goal is original `[2,36,0]`, no velocity bypass.
Read-only `run17/return-penalty-preflight.json` found39.098m/one direction change
in1.776s. Prior baseline path's first40cm had several11cm direction switches;
MPPI stalled on these, rather than another demonstrated false-ground barrier.
That was commissioning; the current18 default now includes the validated profile.
Stop X5 promptly after finite end to avoid known idle LIVO memory growth.

The following run17 startup/state is historical, not a live full mission:

Run 17 started as finite SSH exec **79511**, original-spawn fresh start.
PC main/recorder-17 and X5 core were active. 70 input hashes, exact source and launch
are frozen in `full-mission-17/`. Latest **83 tests passed, 1.03 s**. No online
changes planned. Run16 and all diagnostic sessions are drained and terminal.

Run16 reached the precise parking pose on outbound attempt8. It selected apple1,
confirmed bilateral stem contacts (TCP error .000103186m), detached it, moved over
the tray and physically released it. The immediate post-drop single-frame
stopped check raised `Vehicle must be stopped before manipulating`; mission is
failed, NOT a two-fruit pass. The report still says held_fruit because failure
occurred after detach inside deposit validation; this flag alone does not prove
the fruit is still attached. World was fully reset for17 after stopping core.

Eight seconds of later fixed-base observation showed speed .00108..00175m/s,
base XYZ ranges [.00000019,.00000122,.00001484]m, no over-threshold samples.
This supports a bounded settling check, not removing the safety threshold.
`wait_observation` now requires .3 advancing sim seconds below the unchanged
.035m/s and .05rad/s limits, with fresh joint/object feedback and a 10wall-second
deadline; persistent motion/invalid values/stale or frozen stamps still fail.
Tests cover each rejection and clock rewind. It records explicit settle timeout
witnesses instead of hiding transient motion.

Also fixed node-local simulated clocks after manipulation: Mission.wait_sim first
consumes a new /clock update before starting its interval. Mission.event uses the
latest of its two subsystem clocks. Run16's old failure sim timestamp515.3 was
stale (its arm had already reached537.7); do not align the failure to515.3 in the
independent trace. New tests prove a10->100 stale-clock jump still waits2 full
sim seconds and event timestamps use the active subsystem. Original two-fruit,
original-spawn and physical return acceptance requirements are unchanged.

The following run16 startup/state is historical, not live:

Run 16 started as finite SSH exec **99660**, original-spawn fresh start.
PC main/recorder-16 and X5 core active. The new 100 Hz IMU-derived level frame is
used only for native obstacle-height filtering. Navigation still uses LIVO+RTK;
no truth localization or CAN bypass. 69 input hashes, source and launch snapshots
are frozen in `full-mission-16/`. Latest **75 tests passed, 1.05 s**; private build
passed 1.37 s. No online changes planned. Recorder timeout is 3600 wall seconds.

Run 15 is terminal failed, finite exec40550 drained. It completed outbound on
attempt8, physically picked apples1/2 with bilateral stem contacts and gravity
deposit (TCP errors 4.0296e-6 / 1.7447e-5 m), stowed and released parking. Return
failed after moving 1.27 m; fresh Smac attempts then failed. Native CAN errors0,
18835 nonzero commands, 89712 feedback; LIVO peak2650520KiB. The real screenshot
`full-mission-15/gazebo-two-fruit-tray.png` shows both apples in the physical tray.
This is not a full mission pass. Its flushed independent record is retained.

Actual failed-scene cloud/grid evidence exposed a ground-projection problem,
not a demonstrated arm self-return: obstacle-band stripe X31..35/Y28..39 contained
945 points, 857 at/below the original terrain maximum7cm; world-Z P50=-.096m.
The body pitch .13rad made distant ground appear .08..55m high in the body frame,
forming false barriers across rows. Offline truth was used for this diagnosis
only, not as a runtime input. `gravity_reference` derives base-to-level rotation
from actual IMU orientation plus original mounting, publishing only zero-offset
TF `base_link -> hil_base_level`; native height filter now references that frame.
The .08..55m band is unchanged. New tests retain real trunk points and reject
tilted-ground points, validate IMU mounting and invalid quaternion handling.
Same-time IMU/mount vs actual body roll/pitch comparison agreed before change.

Return headings are now decoupled from PARK and restored to original pi/0/0.
The ad hoc return penalty2 override is removed; native baseline is retained.
Early old-heading read-only success was NOT repeatable while the false ground
barrier existed; do not claim heading alone fixed return. The new level-frame
full run remains necessary. Do not splice segmented commissioning into acceptance.

The following run15 startup/state is historical, not live:

Run 15 started as finite SSH exec **40550**, original-spawn fresh start.
PC main/recorder-15 and X5 core were active. Exact source and 67 input hashes are
frozen in `full-mission-15/`. Latest 68 regressions passed (0.67 s).
No online controller changes are planned for this run.

Run 14 is terminal failed with no fruit picked: the position-only old parking
goal left the vehicle facing east with fruit behind its arm. Full reachability
checking correctly rejected those candidates. The old apple-5 backup also failed
native planning. No position or fruit was teleported to report success.

Joint preflight from the actual scene established a new east-facing parking pose
`[27.86,31.90,0]`: native Smac planning and full MoveIt checks for apples 1/2
passed. A separate apple-5 east pose failed native navigation and was rejected.
Stationary `east-commissioning-04/harvest.json` is terminal
`commissioning_succeeded`: apples 1 and 2 both had real bilateral stem contacts,
tree detachment and gravity deposit into the physical tray; TCP errors were
6.7202e-6 and 5.5323e-6 m. Parking constraint released. This is NOT full mission
acceptance. Navigation commissioning 03 reached the pose after two attempts;
its immediate manipulation was safely rejected while coasting. Commissioning 04
started after settling. All finite commissioning/test sessions are drained.
The attempted XWD screenshot failed with BadColor and is NOT visual evidence.

Run 15 uses this east parking pose with the unchanged original spawn and aisle
via XY coordinates. Pickup-scoped goal tolerance is XY .06 m / yaw .3 rad,
GoalAngleCritic and path orientation guidance enabled; restored before return.
Canonical two-second settling remains. Home completion is still position-only,
half-wheelbase tolerance. Default pickup has one stop; the unreachable apple-5
backup is no longer configured. Five dynamic fruit assets remain in the world.
Return waypoint XY coordinates are unchanged, with headings matching the new
east-facing pose rather than forcing a west-facing parking assumption.

The following describes run 14's historical startup, not a live session:

Run 14 started as finite SSH exec **66875**, now drained.
PC main/recorder-14 and X5 core were active. Inputs/source frozen in full-mission-14.
MPPI time_steps is 80 (8 s) rather than 40 (4 s): with max yaw rate .2302 rad/s,
4 s covers only 53 degrees; 8 s covers a right-angle turn while staying inside
the unchanged 8 m local map. Velocity/collision bounds are unchanged. Runtime
control rate and actual full mission remain to be checked; do not assume passed.

Run 13 is terminal cancelled, exec13715 drained, after repeated mid-aisle
oscillation at the original 4 s horizon. SIGINT preserved cancelled status and
recorded deferred restoration, followed by stopping core. No fruit was picked.
Actual native X5 binary/ROS-domain provenance is in run13/runtime-provenance.json.

Run 12 is terminal failed and exec6080 drained. It reached near PARK on the
first path, but proximity-only via tracking had not retired the rounded second
corner. Recovery wrongly went back toward [41,32.5], then repeatedly failed to
plan through the now locally observed alignment point [32,31.95]. A separate
read-only request to PARK alone from the same live pose succeeded (113 poses).
`route-progress-replay.json` reconstructs native Nav2 nearest-path feedback from
the recorded fused poses/exact path: old index1, corrected index3, final goal NOT
retired. This is replay, not a second live execution.

The middleware now locates intermediate vias on the actual Smac path, rejects
missing vias, and uses native FollowPath remaining-path feedback to retain passed
via progress. Original requested goals remain unchanged. The final pose still
requires native goal-checker success. Six new regressions cover rounded corners,
absolute resume indices, missing via rejection and invalid feedback. Latest
67 regressions pass (1.00 s). This fix is statically loaded in run14.

The following describes the earlier run12 startup and its fidelity evidence:

Run 12 started as finite SSH exec **6080**, original-spawn fresh start.
PC main/recorder-12 and X5 HIL core are active. Inputs/source frozen in private
`full-mission-12/`. Static config includes both run11 fixes and the conditional
apple5 backup. No online controller changes have been made in this run.
First outbound path 604 poses reached near PARK, terminal watchdog cancelled it;
recovery is following the 337-pose path through remaining via index 1. Native CAN
only. Do not wait on run11 or treat the currently live process as stopped.

Read-only fidelity report `full-mission-12/source-fidelity.json` proves actual
map pixels match the original map crop exactly, PC/X5 PGM SHA match, and all five
configured sensor fixed-joint origins plus three SDF sensor poses are preserved.
Initial incomplete inspection looked for sensor links already collapsed in SDF;
that method/result is retained separately and superseded by the URDF-origin plus
actual-SDF-sensor check. Latest 61 regressions pass (including map pixel test).

The following run11 state is historical, not the currently live services:

`full-mission-11` is terminal failed: only one fruit returned, minimum is two.
Finite SSH exec **34833** is exit 1 and drained. X5 HIL core is stopped, available
RAM recovered to 6297 MiB. PC recorder-11 stopped and flushed; PC Gazebo remains
active for display with **motion_authorized=false**. No navigation task is live.
The private input hash manifest and exact executed Python source are retained
in PC `full-mission-11/`. Do not wait on any old mission session.

Run 11 returned with ONE physically picked apple (1). It reached
pickup on outbound attempt 4 after further recovery, then native goal completion.
Apple 1 had bilateral stem contacts, TCP error 0.0000135083 m, detached and fell
into the real tray. Apple 2 failed the complete collision-checked approach from
the actual parked pose; 3 and 4 had no IK. They were properly skipped. Minimum
two fruit failed at final validation. Actual return error at mission check was
0.173979 m; independent record ended at 0.185376 m. Fruit remained in the tray,
arm was stowed, CAN errors 0 / nonzero commands 18301 / feedback frames 74868.
Peak LIVO RSS 2485432 KiB, minimum available RAM 3654292 KiB. Do not relabel this
a two-fruit success. Valid screenshot: run11/picking-gazebo.png; the file named
invalid-window-capture.png is an unrelated window capture, NOT picking evidence.

Two explicit in-flight diagnostic files in run 11 record:
`PathAlignCritic.use_path_orientations: true -> false`, then both PathAlign and
PathFollow `threshold_to_consider: .15 -> .03`. The first change was followed by
progress out of the mid-aisle oscillation. The second keeps feasible-path guidance
inside the 0.06 m pickup tolerance instead of disabling it early. These changes
are in current source, but run 11 is NOT unchanged cold-start validation.

Current source changes (not part of the already completed run 11):
the original PARK remains first, and the previously physically demonstrated
apple-5 parking pose is a backup stop. It is skipped once maximum count is met.
Two new run-level mocks cover both branches. Latest **60 regressions passed
1.01 s**, including corrected audit time scoping: outbound waypoints must only
be matched within outbound, not to a closer return pass through the same point.
The corrected run11 audit passes all physical/trace/CAN checks and fails the
mission-success and two-distinct-fruit requirements. Its source/input SHA are
recorded and exact audit source copied beside the report.
This fallback still needs actual cold-start full validation. Scene was regenerated
and new source/scenario synced to X5 after stopping its core. Start fresh run 12
with these static settings. Never reuse picked fruit or spawn at the parking
pose to claim a full run. Goal remains active; this turn made real progress and
is not blocked, but has not met complete two-fruit acceptance.

Run 10 is terminal failed, exec 98487 drained. Its idle native parameter
roundtrip passed: apply 22.2171 s, restore 22.1845 s, both native servers stayed
active without lifecycle reset. During outbound it repeatedly stalled on short
Smac reversing cusps. An explicitly recorded in-flight diagnostic disabled
`FollowPath.enforce_path_inversion`, after which the car promptly advanced from
X about 15 to 20, then reached the original row. However final precise parking
still stalled and bounded recovery ended the mission failed, without picking.
All reports and the flushed independent PC record remain in `full-mission-10/`.

Run 11 statically uses enforce_path_inversion=false. It retains the original
aisle via points [41,36,0] and [41,32.5,pi], and adds an aligned approach via
[32,31.95,pi] before the same PARK. This makes the final 3.7 m straight instead
of attempting the remaining lateral adjustment at the precise parking point.
No vehicle or fruit was teleported. The two later online changes are recorded
above; the earlier plan to make no online experiment was not maintained.

Run 08 completed outbound on attempt 1 and physically picked apples 1 and 2 from
one PARK stop. Its mission failed before return on a 15 s parameter reply timeout.
Readback confirmed the requested parameters had applied. A separately executed
return then succeeded through native CAN: actual return error 0.20688 m, both
apples still inside the physical tray, maximum arm stow error 0.001974 rad.
This is segmented debugging evidence, NOT one accepted continuous full run.
Reports and the independent PC trace are retained in `full-mission-08/`.

Exact journal evidence exposed the timeout cause: native Smac heuristic rebuild
started at X5 wall 20:54:36, starved planner heartbeat, and the 10 s lifecycle bond
expired at 20:54:46. The callback finished around 20:54:58, then Nav2 reset and
auto-reconfigured. HIL now uses a 60 s lifecycle bond and a 120 s parameter reply
deadline, with no blind resubmission. This does not change physical-vehicle
configuration or disable the CAN command freshness stop.

Run 09 is terminal failed after deliberate cancellation to fix the proven bond
issue. It had one optimizer failure and then oscillated on a short reversing cusp
far from its destination. New route-length watchdog requires path progress within
45 sim seconds, supplementing the 12 s terminal watchdog, then cancels the action
and requests a new native Smac plan. It never forces success. SIGINT cleanup now
records deferred parameter restoration if ROS context is already shut down,
rather than masking cancellation with a bogus service timeout; restart required.

MPPI velocity feedback is now `/wheel/odometry`, decoded by the actual X5 CAN
driver from actual PC CAN feedback. FAST-LIVO2+RTK remains the sole pose/TF source.
08 moving-speed comparison: wheel vs physics RMSE vx/wz 0.00512/0.00705;
fused vx/wz 0.01919/0.01628. No ground-truth localization bypass was introduced.

57 targeted regressions passed in 0.55 s. Private package build passed. An offline
`audit_mission` cross-check now rejects failed/segmented runs, missing bilateral
stem contact, lost cargo, discontinuous actual motion or absent independent data.
It does not by itself validate real-hardware grasping or all safety properties.

## Accepted Partial Evidence

`full-mission-03` drove from the original spawn, physically picked apples 5 and 1,
confirmed both finger-pad stem contacts, detached their real tree joints, deposited
both under gravity into the physical tray, stowed, and drove back near the start.
The final return failed; do not relabel it successful.

Final observed base XY was `[1.8118, 36.2657]`, about 0.326 m from the original
`[2, 36]`. Smac ultimately rejected the current footprint as lethal. CAN errors
were zero, 31,500 nonzero native CAN commands were observed. Peak FAST-LIVO RSS
was 4,014,052 KiB; minimum X5 available RAM was 2,066,796 KiB. No OOM in this run,
but the earlier long-idle memory-growth problem is not thereby solved.

Full failed reports, two independent PC recording parts, screenshots and exact
executed navigation config/source are retained in PC `full-mission-03/` and X5.

## Previous Tests (07 and Earlier)

**Latest 2026-11-06 update:** `full-mission-07` is now running from a fresh original
spawn, finite SSH exec **97839**. PC main/recorder and X5 core active; standalone
self-filter was stopped and is not duplicated. Do not wait on or restart run 05.
New cold-start inputs are hashed in PC `full-mission-07/inputs.json`.

Run 06 is terminal failed: it had no task-level retry until the 900 s FollowPath
timeout, while circling near PARK. The path itself had no direction changes and
ended only 0.035 m from PARK; a current local grid showed goal-center cost 0.
Thus this was not proved to be a blocked goal. Cancelling the next attempt during
profile cleanup produced a restoration timeout, preserved as failed, not success.

Run 07 additionally disables GoalAngleCritic (the checker was position-only but
the optimizer still penalized terminal yaw) and retains PathAlign/PathFollow until
0.15 m instead of dropping their guidance early. Navigator now monitors official
FollowPath remaining-path feedback plus actual fused endpoint distance. No 3 cm
improvement for 12 sim seconds in the terminal 2 m cancels the old action, waits
for its result, and requests a fresh Smac path through existing recovery. It never
forces a successful result. Four new watchdog regressions pass: circling,
cross-track improvement, legitimate longer detour, invalid feedback/clock rewind.

Apple 2's socket-to-branch gap is now 70 mm (apple 1 remains 40 mm). Read-only
`same-parking-two-fruit-01.json` proved both reachable from the original PARK with
full branch/arm collision checking. Current mission uses one PARK stop, not the
two-close-parking arrangement that caused run 05's precision-maneuver problem.
Original spawn, row and aisle via points are unchanged.

Goal checker is position-only: yaw tolerance pi, stateful=false. Normal final XY
tolerance is half the true wheelbase, 0.263293743 m. Pickup navigation temporarily
uses XY 0.06 m, followed by actual-pose manipulation validation. Final yaw is still
measured but no longer incorrectly required for return completion. Closest-first
online selection now stops once one reachable candidate is found, rather than
exhaustively testing every unreachable fruit before executing a known good pick.
47 regressions passed (0.54 s). Private overlay rebuilt and source/scenario synced.

The following paragraphs are retained historical context, not the live state.

`full-mission-04` failed during the near-tree outbound search after applying the
high direction-change cost globally. Its failure reports and independent PC
recording are retained; it did not pick fruit. The new profile is therefore
scoped to return only; outbound retains the already demonstrated original costs.

`full-mission-05` is terminal **cancelled**, not successful. It physically picked
apple 5 and stowed, but repeatedly circled near the second parking point while
trying to satisfy the strict heading goal. It was deliberately stopped after
logging this problem. Actual final XY `[28.2566, 31.9363]` was close to PARK but
heading was not settled. No fruit is held; apple 5 is physically in the tray.
All reports/independent PC recording are copied back to PC. Finite SSH session
27369 and all other finite sessions are drained.

PC `agribot-hil-pc` and `agribot-hil-self-filter` are intentionally active for
display, with **motion_authorized=false**. Recorder is stopped. All X5 HIL
units are stopped; available RAM returned to 6311 MiB. Do not wait on any old
mission PID/session or claim a background route is still running.

Before any fresh PC launch, stop the standalone `agribot-hil-self-filter` unit
as well as `agribot-hil-pc`, or it will duplicate the filter now inside pc.launch.

The only planning parameter changes versus run 03 are a scoped return-only Smac
profile: `reverse_penalty=1.05`, `change_penalty=2.0`. It is applied through the
native parameter service and restored in finally, including on failure. The
generated baseline and real-vehicle configs are untouched.
Read-only tests on the same return via points showed:

- Old settings: 63.807 m, 18 direction changes.
- New settings: 56.719 m, zero direction changes, continuous reverse path.

Evidence: `return-path-baseline-01.json` and `return-path-cusp-penalty-01.json`.
This is prospective planning evidence, not a successful execution claim.

## New Code This Session

- Five dynamic apples plus background fruit across 337 wood components.
  Apple 5 has a 70 mm socket-to-wood stem, with shared SDF/MoveIt dimensions;
  the other four retain 40 mm. All stems meet measured actual woody surfaces.
- `parking_preflight`: prospective collision checking only. Extended-stem apple 5
  passed and was later physically picked in run 03.
- `mission`: original-spawn check, ordered pickup stops, native Smac/MPPI actions,
  progress-aware bounded recovery, physical harvesting and return/cargo checks.
- `mission_record`: independent actual/fused traces, arm, CAN and near-body costmap.
- `path_preflight`: read-only prospective return path and direction-change metrics.
- Busy lifecycle startup services now retry within a bounded activation deadline.
- Latest narrow regression: 38 passed in 0.49 s, including profile restoration
  after success and failure. Private Python overlay built.

## New Actual Self-Return Evidence

During run 05, a 30 s read-only moving-cloud probe captured four near-front-wheel
returns with abs(Y)=0.371..0.374 m, just outside the old 0.37 m self mask. Evidence
`full-mission-05/moving-self-return-probe.json`. They are inside the front-wheel
steering envelope and may mark a costmap voxel as occupied near the vehicle.
The HIL launch self mask is now Y +/-0.40 m, same X[-0.19,0.69], Z[-0.20,0.36].
An exact regression removes all four measured points while retaining points just
outside X, Y and height bounds. No original vehicle/config code was modified.

During the first picking stop of run 05, only the exact old HIL filter PID was
terminated and replaced with the corrected standalone node. No model reset,
pose or motion commands were sent. `sensor-filter-change.json` explicitly labels
this in-flight change. It means run 05 is not an unchanged cold-start regression.
There is only one live filter, PID 3651262 at this handoff (revalidate).

## Historical Next Steps Before Run 08

1. Resolve pickup precision-docking behavior: run 05 followed a 32-point path for
   several minutes, reached close XY but continued moving to settle yaw. At sim
   419.9 actual base was `[28.25596,31.94868,.04404]`, yaw about 2.47 rad, speed
   .215 m/s. Current goal tolerances are XY .08/yaw .15, stateful=true. Do not
   simply force-stop and call a full run passed; plan a feasible aligned approach
   or otherwise validate the actual manipulation pose before picking.
2. Root user history favors position-only final navigation stopping, with half
   wheelbase positional tolerance. HIL's added strict heading checks should be
   reconsidered carefully; the arm still needs an actually reachable parked pose.
3. Run the full original-spawn/two-fruit/return test with the latest filter and
   return-only profile. Inspect independent trace and real contacts/cargo, not only
   action status. Stop core promptly after the finite test to avoid idle growth.
4. Preserve all failures. No full mission has yet passed; persistent goal active.

Known boundaries remain: Gazebo known-object poses are used for manipulation,
not visual AI detection; the arm uses a kinematic position actuator, not verified
real torque/force control. The MoveIt/image-transport SIGINT teardown crash and
earlier long-idle LIVO OOM are not claimed fixed. No Unity integration yet.
