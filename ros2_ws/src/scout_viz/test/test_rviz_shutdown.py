"""Opt-in X11 integration checks: AGRIBOT_TEST_RVIZ=1 python3 -m pytest ..."""

import os
from pathlib import Path
import signal
import subprocess
import time

import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get("AGRIBOT_TEST_RVIZ") != "1" or not os.environ.get("DISPLAY"),
    reason="Requires explicit opt-in and a working X11 display",
)


@pytest.mark.parametrize("iteration", range(3))
@pytest.mark.parametrize("stop_signal", [signal.SIGINT, signal.SIGTERM])
def test_shutdown_after_simulation_clock_stops(tmp_path, stop_signal, iteration, monkeypatch):
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image

    binary = Path(get_package_prefix("rviz2")) / "lib/rviz2/rviz2"
    config = tmp_path / "clock.rviz"
    config.write_text("""Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ''
  Global Options:
    Fixed Frame: map
    Frame Rate: 30
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
    - Class: scout_viz/Image
      Name: Camera
      Enabled: true
      Value: true
      Topic:
        Value: /test_image
        Reliability Policy: Best Effort
        Depth: 1
  Enabled: true
  Tools: []
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
""")
    domain = 97
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    log_path = tmp_path / "rviz.log"
    context = rclpy.context.Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node("rviz_shutdown_clock", context=context)
    publisher = node.create_publisher(Clock, "/clock", 10)
    image_publisher = node.create_publisher(Image, "/test_image", 1)
    frame = Image()
    frame.header.frame_id = "camera"
    frame.height, frame.width, frame.step = 48, 64, 192
    frame.encoding = "rgb8"
    frame.data = bytes([30, 180, 70]) * 64 * 48
    try:
        with log_path.open("w") as log:
            process = subprocess.Popen(
                [str(binary), "-d", str(config), "--ros-args", "-p", "use_sim_time:=true"],
                env=env, stdout=log, stderr=subprocess.STDOUT,
            )
            try:
                deadline = time.monotonic() + 25
                sample = 0
                while time.monotonic() < deadline:
                    assert process.poll() is None, log_path.read_text()
                    sample += 1
                    stamp = Clock()
                    stamp.clock.sec, stamp.clock.nanosec = divmod(sample * 10000000, 1000000000)
                    publisher.publish(stamp)
                    frame.header.stamp = stamp.clock
                    if sample % 10 == 0:
                        image_publisher.publish(frame)
                    assert "failed to load" not in log_path.read_text().lower(), log_path.read_text()
                    if "Stereo is NOT SUPPORTED" in log_path.read_text() and sample > 800:
                        break
                    time.sleep(.01)
                else:
                    pytest.fail("RViz did not initialize: " + log_path.read_text())
                node.destroy_publisher(publisher)
                assert image_publisher.get_subscription_count() > 0, log_path.read_text()
                node.destroy_publisher(image_publisher)
                time.sleep(.3)
                process.send_signal(stop_signal)
                assert process.wait(timeout=5) == 0, log_path.read_text()
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=5)
    finally:
        node.destroy_node()
        context.shutdown()
