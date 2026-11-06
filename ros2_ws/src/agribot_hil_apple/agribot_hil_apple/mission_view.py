"""Display saved mission paths; never publish control, localization or TF."""
import argparse
import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray


def planned_path(rows):
    message = PathMessage()
    message.header.frame_id = 'map'
    for x, y, qz, qw in rows:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(x), float(y), .12
        pose.pose.orientation.z, pose.pose.orientation.w = float(qz), float(qw)
        message.poses.append(pose)
    return message


def observed_path(rows):
    message = planned_path([[row[1], row[2], row[6], row[7]] for row in rows])
    for pose in message.poses:
        pose.pose.position.z = .16
    return message


def waypoint_markers(manifest):
    message = MarkerArray()
    for index, xyz in zip(manifest['fixed_route_order'], manifest['fixed_route_points']):
        marker = Marker()
        marker.header.frame_id, marker.ns, marker.id = 'map', 'fixed_waypoints', index
        marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xyz[0], xyz[1], .65
        marker.pose.orientation.w = 1.
        marker.scale.z = .65
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .8, .15, 1.
        marker.text = str(index)
        message.markers.append(marker)
    if manifest.get('finish_is_spawn'):
        marker = Marker()
        marker.header.frame_id, marker.ns, marker.id = 'map', 'fixed_waypoints', 0
        marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = manifest['spawn'][0], manifest['spawn'][1], .65
        marker.pose.orientation.w, marker.scale.z = 1., .65
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = .3, .8, 1., 1.
        marker.text = 'Start'
        message.markers.append(marker)
    return message


class MissionView(Node):
    def __init__(self, directory):
        super().__init__('hil_mission_view')
        self.directory, self.signatures = directory, {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.outputs = {name: self.create_publisher(kind, '/hil/visualization/'+name, qos)
                        for name, kind in [('outbound_path', PathMessage), ('completion_path', PathMessage),
                                           ('observed_path', PathMessage), ('waypoints', MarkerArray)]}
        self.live = PathMessage()
        self.live.header.frame_id = 'map'
        self.last_live_stamp = -1.
        self.create_subscription(Odometry, '/fastlivo_rtk/odometry', self.observe, qos_profile_sensor_data)
        self.create_timer(1., self.refresh)
        self.refresh()

    def observe(self, message):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if stamp-self.last_live_stamp < .2:
            return
        self.last_live_stamp = stamp
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose = message.pose.pose
        pose.pose.position.z = .16
        self.live.poses.append(pose)
        self.outputs['observed_path'].publish(self.live)

    def refresh(self):
        for filename in ('mission.json', 'independent-pc.json'):
            path = self.directory/filename
            try:
                signature = path.stat().st_mtime_ns
                if signature == self.signatures.get(filename):
                    continue
                data = json.loads(path.read_text())
            except (FileNotFoundError, json.JSONDecodeError):
                continue
            self.signatures[filename] = signature
            if filename == 'mission.json':
                for phase in ('outbound', 'completion'):
                    entries = [entry for entry in data['navigation'] if entry['phase'] == phase and entry.get('path')]
                    if entries:
                        self.outputs[phase+'_path'].publish(planned_path(entries[-1]['path']))
                self.outputs['waypoints'].publish(waypoint_markers(data['manifest']))
            else:
                self.live = observed_path(data['traces'].get('/fastlivo_rtk/odometry', []))
                self.outputs['observed_path'].publish(self.live)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init()
    node = MissionView(args.directory)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
