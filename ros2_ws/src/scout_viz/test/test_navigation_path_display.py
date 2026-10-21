"""Opt-in X11 rendering test for the display-only path cleanup."""
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get("AGRIBOT_TEST_RVIZ") != "1" or not os.environ.get("DISPLAY"),
    reason="Requires explicit opt-in and X11",
)


def test_completed_path_is_cleared_without_changing_native_topics(tmp_path, monkeypatch):
    import numpy as np
    from PIL import ImageGrab
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as RosPath

    config = tmp_path / "navigation_path.rviz"
    config.write_text("""Panels: []
Visualization Manager:
  Class: ''
  Global Options:
    Fixed Frame: map
    Frame Rate: 30
    Background Color: 32; 32; 32
  Displays:
    - Class: scout_viz/NavigationPath
      Name: Navigation Path
      Enabled: true
      Value: true
      Buffer Length: 1
      Color: 25; 255; 0
      Line Style: Lines
      Pose Style: None
      Topic:
        Value: /test_path
        Reliability Policy: Reliable
        Durability Policy: Volatile
        Depth: 1
  Enabled: true
  Tools: []
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Scale: 50
      X: 0
      Y: 0
      Angle: 0
Window Geometry:
  Width: 900
  Height: 700
""")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    domain = 96
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    context = rclpy.context.Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node("navigation_path_render_test", context=context)
    publisher = node.create_publisher(RosPath, "/test_path", 1)
    status_publisher = node.create_publisher(GoalStatusArray, "/navigate_through_poses/_action/status",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    child_publisher = node.create_publisher(GoalStatusArray, "/follow_path/_action/status",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    path = RosPath()
    path.header.frame_id = "map"
    for x, y in [(-4., -3.), (-4., 3.), (4., 3.)]:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.w = 1.
        path.poses.append(pose)

    def goal_status(seconds, value):
        array = GoalStatusArray()
        goal = GoalStatus()
        goal.goal_info.stamp.sec = seconds
        goal.status = value
        array.status_list.append(goal)
        return array

    def publish_for(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:
            path.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(path)
            time.sleep(.03)

    def green_pixels(window, name):
        subprocess.run(["xdotool", "windowactivate", "--sync", window], check=True, env=env)
        geometry = subprocess.check_output(["xdotool", "getwindowgeometry", "--shell", window], text=True, env=env)
        bounds = dict(line.split("=", 1) for line in geometry.splitlines())
        x, y, width, height = [int(bounds[key]) for key in ("X", "Y", "WIDTH", "HEIGHT")]
        time.sleep(.3)
        screenshot = ImageGrab.grab(bbox=(x, y, x+width, y+height), xdisplay=env["DISPLAY"])
        screenshot.save(tmp_path / f"{name}.png")
        pixels = np.asarray(screenshot)[70:-70, 60:-60, :3].astype(np.int32)
        return int(np.sum((pixels[:, :, 1]>80) & (pixels[:, :, 1]>2*pixels[:, :, 0]) &
                          (pixels[:, :, 1]>2*pixels[:, :, 2])))

    process = None
    try:
        with (tmp_path/"rviz.log").open("w") as log:
            process = subprocess.Popen([str(Path(get_package_prefix("rviz2"))/"lib/rviz2/rviz2"),
                                       "-d", str(config)], env=env, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+20
            window = None
            while time.monotonic()<deadline:
                assert process.poll() is None, (tmp_path/"rviz.log").read_text()
                lines = subprocess.check_output(["wmctrl", "-lp"], text=True, env=env).splitlines()
                window = next((line.split()[0] for line in lines if len(line.split())>2 and
                               line.split()[2] == str(process.pid)), None)
                if window and publisher.get_subscription_count():
                    break
                time.sleep(.2)
            assert window and publisher.get_subscription_count(), (tmp_path/"rviz.log").read_text()
            status_publisher.publish(goal_status(10, 2))
            publish_for(1.5)
            active = green_pixels(window, "active")
            assert active > 100
            child_publisher.publish(goal_status(11, 4))
            publish_for(.5)
            assert green_pixels(window, "intermediate") > 100
            status_publisher.publish(goal_status(10, 4))
            child_publisher.publish(goal_status(12, 2))
            publish_for(1.)
            assert green_pixels(window, "completed_late_paths") == 0
            status_publisher.publish(goal_status(20, 2))
            publish_for(1.)
            assert green_pixels(window, "next_task") > 100
            assert node.count_publishers("/test_path") == 1
            assert node.count_publishers("/nav2/cmd_vel") == 0
            assert node.count_publishers("/cmd_vel") == 0
    finally:
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        node.destroy_node()
        context.shutdown()
