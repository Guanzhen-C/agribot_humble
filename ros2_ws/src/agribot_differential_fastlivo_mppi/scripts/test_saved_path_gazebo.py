#!/usr/bin/env python3
"""Run the saved-path controller test against Gazebo wheel/contact physics.

Gazebo world odometry deliberately isolates navigation from localization error.
The obstacle is a real Gazebo collision box and is inserted into the test map;
this fixture does not validate LiDAR/STVL detection or FAST-LIVO2 localization.
"""
import importlib.util
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.msg import ContactsState
from nav_msgs.msg import Odometry

source = Path(get_package_share_directory("agribot_mobile_app")) / "tools/test_saved_path_navigation.py"
spec = importlib.util.spec_from_file_location("saved_test", source)
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


class GazeboSimulation(fixture.Simulation):
    def __init__(self):
        self.spawned = False
        self.actual_v, self.actual_w = 0.0, 0.0
        self.spawn_status = None
        self.contact_count = 0
        super().__init__()
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.create_subscription(Odometry, "/gazebo_test/odom", self.physics_pose, 10)
        self.create_subscription(ContactsState, "/gazebo_test/contacts", self.contacts, 10)

    def advance(self):
        pass

    def physics_pose(self, message):
        import math
        p = message.pose.pose
        self.x, self.y = p.position.x, p.position.y
        self.yaw = math.atan2(2*p.orientation.w*p.orientation.z, 1-2*p.orientation.z**2)
        self.actual_v, self.actual_w = message.twist.twist.linear.x, message.twist.twist.angular.z

    def measured_velocity(self):
        return self.actual_v, self.actual_w

    def contacts(self, message):
        self.contact_count += sum("navigation_test_obstacle" in (state.collision1_name+state.collision2_name) for state in message.states)

    def validation_details(self):
        return {"gazebo_obstacle_spawn_success": self.spawn_status,
                "obstacle_body_contact_messages": self.contact_count,
                "final_angular_velocity": self.actual_w}

    def publish_map(self):
        if self.obstacle and not self.spawned and hasattr(self, "spawn_client"):
            request = SpawnEntity.Request()
            request.name = "navigation_test_obstacle"
            request.xml = '<sdf version="1.6"><model name="obstacle"><static>true</static><link name="box"><collision name="collision"><geometry><box><size>0.8 0.8 1.0</size></box></geometry></collision><visual name="visual"><geometry><box><size>0.8 0.8 1.0</size></box></geometry></visual></link></model></sdf>'
            request.initial_pose.position.x, request.initial_pose.position.z = 4.0, 0.5
            request.initial_pose.orientation.w = 1.0
            future = self.spawn_client.call_async(request)
            future.add_done_callback(lambda f: setattr(self, "spawn_status", f.result().success))
            self.spawned = True
        super().publish_map()


if __name__ == "__main__":
    os.environ["SAVED_PATH_TEST_PREFIX"] = "saved_path_gazebo"
    os.environ["SAVED_PATH_TEST_TIMEOUT"] = "300"
    os.environ["SAVED_PATH_TEST_SCOPE"] = "Gazebo four-wheel contact physics + real Nav2; ground-truth odometry and injected obstacle map; not LiDAR/STVL or FAST-LIVO2 acceptance"
    fixture.Simulation = GazeboSimulation
    fixture.main()
