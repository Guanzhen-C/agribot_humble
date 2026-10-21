# Simulation RViz

`scripts/run_rviz2_clean_env.sh` removes an inherited ROS 1 environment and starts
the standard Humble RViz executable. Upstream RViz is not modified.

The `scout_viz/Image` plugin inherits the official image display. Rendering,
transports, topics and QoS remain unchanged. In the installed Humble build,
`ImageTransportDisplay::unsubscribe()` disconnects its callback and destroys the
filter, but retains a `message_filters::Connection` pointing at that filter.
A second unsubscribe (disable followed by destruction) can access the freed
filter mutex. The local plugin clears the connection before releasing the filter
and makes repeated cleanup harmless. A core dump from the visible vehicle test
identified this call stack; the earlier general shutdown-order workaround was
removed because it did not address this defect.

The standard Navigation 2 panel has a separate unload-order issue on the tested
Humble build: RViz's shared ROS node can outlive the panel library and crash in
callback-group cleanup. The wrapper keeps the existing Nav2 library loaded until
process exit using a process-local `LD_PRELOAD`. Navigation 2 remains enabled;
no navigation actions, goals, algorithms or display topics are replaced.

## Verification

Build this package before using the wrapper:

```bash
colcon build --packages-select scout_viz --symlink-install
source install/setup.bash
AGRIBOT_TEST_RVIZ=1 python3 -m pytest -q src/scout_viz/test/test_rviz_shutdown.py
AGRIBOT_TEST_RVIZ=1 python3 -m pytest -q src/scout_viz/test/test_nav2_panel_shutdown.py
```

The opt-in integration test needs a working X11 `DISPLAY`/`XAUTHORITY`. It starts
RViz on isolated ROS domain 97, publishes images and simulation time, stops both, and
requires both SIGINT and SIGTERM to exit normally within five seconds. Full
vehicle validation additionally checks exit codes after the actual map, TF,
robot, point-cloud and image displays have run. Abnormal exits are not suppressed.
The additional Navigation 2 regression opens the real panel on isolated domain
90, checks that its library is mapped and requires normal SIGINT/SIGTERM exit.
