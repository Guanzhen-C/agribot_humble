"""IMU-derived level frame for obstacle-height filtering, not localization."""
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def base_to_level_quaternion(imu_quaternion, base_to_imu_rpy):
    quaternion = np.asarray(imu_quaternion, dtype=float)
    mounting = np.asarray(base_to_imu_rpy, dtype=float)
    if (quaternion.shape != (4,) or mounting.shape != (3,) or
            not np.isfinite(quaternion).all() or not np.isfinite(mounting).all() or
            np.linalg.norm(quaternion) < 1e-8):
        raise ValueError('Finite IMU orientation and mounting rotation are required')
    world_base = Rotation.from_quat(quaternion)*Rotation.from_euler('xyz', mounting).inv()
    heading = world_base.as_matrix()[:, 0].copy()
    heading[2] = 0.
    if np.linalg.norm(heading) < 1e-6:
        raise ValueError('A vertical vehicle heading cannot define the level frame')
    heading /= np.linalg.norm(heading)
    up = np.array([0., 0., 1.])
    world_level = Rotation.from_matrix(np.column_stack((heading, np.cross(up, heading), up)))
    return (world_base.inv()*world_level).as_quat()


class GravityReference(Node):
    def __init__(self):
        super().__init__('hil_gravity_reference')
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.level_frame = self.declare_parameter('level_frame', 'hil_base_level').value
        self.imu_frame = self.declare_parameter('imu_frame', 'imu_link').value
        self.mounting = self.declare_parameter('base_to_imu_rpy', [0., 0., 0.]).value
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Imu, '/imu/data', self.receive, qos_profile_sensor_data)

    def receive(self, message):
        if message.header.frame_id != self.imu_frame or message.orientation_covariance[0] < 0.:
            return
        q = message.orientation
        try:
            rotation = base_to_level_quaternion([q.x, q.y, q.z, q.w], self.mounting)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.)
            return
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.level_frame
        (transform.transform.rotation.x, transform.transform.rotation.y,
         transform.transform.rotation.z, transform.transform.rotation.w) = map(float, rotation)
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = GravityReference()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
