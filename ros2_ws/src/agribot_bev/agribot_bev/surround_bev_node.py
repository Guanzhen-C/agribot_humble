"""ROS 2 node that converts four simulated fisheye images into a local BEV."""

import math
import time

import cv2
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

from .projection import (
    CAMERA_NAMES,
    make_bev_ground_grid,
    make_occupancy_valid_mask,
    make_stereographic_map,
    quaternion_matrix,
)


class SurroundBevNode(Node):
    def __init__(self):
        super().__init__("surround_bev")
        self._declare_parameters()
        self._read_parameters()

        self._bridge = CvBridge()
        self._maps = None
        self._map_image_shape = None
        self._frame_count = 0
        self._last_obstacle_mask = None
        self._last_cloud_stamp_ns = None

        self._image_publisher = self.create_publisher(
            Image, self._bev_image_topic, qos_profile_sensor_data
        )
        self._mask_publisher = self.create_publisher(
            Image, self._valid_mask_topic, qos_profile_sensor_data
        )
        self._grid_publisher = self.create_publisher(
            OccupancyGrid, self._occupancy_grid_topic, 1
        )

        self._image_subscribers = [
            Subscriber(
                self,
                Image,
                topic,
                qos_profile=qos_profile_sensor_data,
            )
            for topic in self._camera_topics
        ]
        self._synchronizer = ApproximateTimeSynchronizer(
            self._image_subscribers,
            queue_size=self._sync_queue_size,
            slop=self._sync_slop_sec,
        )
        self._synchronizer.registerCallback(self._on_images)

        self._tf_buffer = None
        self._tf_listener = None
        self._pointcloud_subscription = None
        if self._fuse_pointcloud:
            self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._pointcloud_subscription = self.create_subscription(
                PointCloud2,
                self._pointcloud_topic,
                self._on_pointcloud,
                qos_profile_sensor_data,
            )

        self._grid_size = int(round(2.0 * self._extent_m / self._resolution_m))
        self._occupancy_valid = make_occupancy_valid_mask(
            self._extent_m, self._resolution_m, self._min_range_m
        )
        self.get_logger().info(
            f"surround BEV ready: {self._grid_size}x{self._grid_size} "
            f"at {self._resolution_m:.3f} m/cell, "
            f"pointcloud_fusion={self._fuse_pointcloud}"
        )

    def _declare_parameters(self):
        self.declare_parameter("front_topic", "/bev/cameras/front/image_raw")
        self.declare_parameter("left_topic", "/bev/cameras/left/image_raw")
        self.declare_parameter("rear_topic", "/bev/cameras/rear/image_raw")
        self.declare_parameter("right_topic", "/bev/cameras/right/image_raw")
        self.declare_parameter("bev_image_topic", "/bev/image")
        self.declare_parameter("valid_mask_topic", "/bev/valid_mask")
        self.declare_parameter("occupancy_grid_topic", "/bev/ground_grid")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("extent_m", 10.0)
        self.declare_parameter("resolution_m", 0.1)
        self.declare_parameter("min_range_m", 0.3)
        self.declare_parameter("camera_height_m", 1.0)
        self.declare_parameter("camera_radius_m", 0.09)
        self.declare_parameter("camera_pitch_deg", 25.0)
        self.declare_parameter("horizontal_fov_deg", 180.0)
        self.declare_parameter("vehicle_length_m", 2.4)
        self.declare_parameter("vehicle_width_m", 1.35)
        self.declare_parameter("sync_queue_size", 12)
        self.declare_parameter("sync_slop_sec", 0.12)
        self.declare_parameter("fuse_pointcloud", False)
        self.declare_parameter("pointcloud_topic", "/points")
        self.declare_parameter("pointcloud_timeout_sec", 0.5)
        self.declare_parameter("obstacle_min_height_m", 0.15)
        self.declare_parameter("obstacle_max_height_m", 2.5)
        self.declare_parameter("pointcloud_stride", 1)

    def _read_parameters(self):
        value = lambda name: self.get_parameter(name).value
        self._camera_topics = tuple(value(f"{name}_topic") for name in CAMERA_NAMES)
        self._bev_image_topic = value("bev_image_topic")
        self._valid_mask_topic = value("valid_mask_topic")
        self._occupancy_grid_topic = value("occupancy_grid_topic")
        self._base_frame = value("base_frame")
        self._extent_m = float(value("extent_m"))
        self._resolution_m = float(value("resolution_m"))
        self._min_range_m = float(value("min_range_m"))
        self._camera_height_m = float(value("camera_height_m"))
        self._camera_radius_m = float(value("camera_radius_m"))
        self._camera_pitch_rad = math.radians(float(value("camera_pitch_deg")))
        self._horizontal_fov_rad = math.radians(float(value("horizontal_fov_deg")))
        self._vehicle_length_m = float(value("vehicle_length_m"))
        self._vehicle_width_m = float(value("vehicle_width_m"))
        self._sync_queue_size = int(value("sync_queue_size"))
        self._sync_slop_sec = float(value("sync_slop_sec"))
        self._fuse_pointcloud = bool(value("fuse_pointcloud"))
        self._pointcloud_topic = value("pointcloud_topic")
        self._pointcloud_timeout_sec = float(value("pointcloud_timeout_sec"))
        self._obstacle_min_height_m = float(value("obstacle_min_height_m"))
        self._obstacle_max_height_m = float(value("obstacle_max_height_m"))
        self._pointcloud_stride = max(1, int(value("pointcloud_stride")))
        if self._extent_m <= 0.0 or self._resolution_m <= 0.0:
            raise ValueError("extent_m and resolution_m must be positive")
        if self._min_range_m < 0.0 or self._min_range_m >= self._extent_m:
            raise ValueError("min_range_m must be in [0, extent_m)")
        if self._sync_queue_size <= 0 or self._sync_slop_sec <= 0.0:
            raise ValueError("synchronizer queue size and slop must be positive")

    def _build_maps(self, image_shape):
        image_height, image_width = image_shape
        grid_x, grid_y = make_bev_ground_grid(
            self._extent_m, self._resolution_m
        )
        ground_range = np.hypot(grid_x, grid_y)
        range_valid = (ground_range >= self._min_range_m) & (
            ground_range <= self._extent_m
        )
        maps = []
        total_weight = np.zeros_like(grid_x, dtype=np.float64)
        for camera_name in CAMERA_NAMES:
            map_x, map_y, weight = make_stereographic_map(
                camera_name,
                image_width,
                image_height,
                grid_x,
                grid_y,
                self._camera_height_m,
                self._camera_radius_m,
                self._camera_pitch_rad,
                self._horizontal_fov_rad,
            )
            weight = np.where(range_valid, weight, 0.0)
            maps.append((map_x, map_y, weight))
            total_weight += weight
        self._maps = maps
        self._image_valid = total_weight > 0.0
        self._map_image_shape = image_shape
        coverage = float(np.mean(self._image_valid))
        self.get_logger().info(
            f"precomputed BEV LUTs for {image_width}x{image_height} camera "
            f"images, coverage={coverage:.3f}"
        )

    def _on_images(self, front, left, rear, right):
        started = time.perf_counter()
        messages = (front, left, rear, right)
        try:
            images = tuple(
                self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                for message in messages
            )
        except Exception as error:
            self.get_logger().error(f"failed to convert camera image: {error}")
            return
        shapes = {image.shape[:2] for image in images}
        if len(shapes) != 1:
            self.get_logger().error(f"camera image shapes differ: {sorted(shapes)}")
            return
        image_shape = next(iter(shapes))
        if self._maps is None or image_shape != self._map_image_shape:
            self._build_maps(image_shape)

        size = self._grid_size
        color_sum = np.zeros((size, size, 3), dtype=np.float64)
        weight_sum = np.zeros((size, size), dtype=np.float64)
        for image, (map_x, map_y, weight) in zip(images, self._maps):
            projected = cv2.remap(
                image,
                map_x,
                map_y,
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
            )
            color_sum += projected.astype(np.float64) * weight[..., None]
            weight_sum += weight

        bev = np.zeros((size, size, 3), dtype=np.uint8)
        bev[self._image_valid] = np.clip(
            color_sum[self._image_valid]
            / weight_sum[self._image_valid, None],
            0.0,
            255.0,
        ).astype(np.uint8)
        self._mask_vehicle(bev)

        stamp_source = max(messages, key=self._stamp_nanoseconds)
        stamp = stamp_source.header.stamp
        self._publish_image(bev, "bgr8", stamp, self._image_publisher)
        valid_mask = (self._image_valid.astype(np.uint8) * 255)
        self._publish_image(valid_mask, "mono8", stamp, self._mask_publisher)
        self._publish_occupancy_grid(stamp)

        self._frame_count += 1
        if self._frame_count == 1 or self._frame_count % 30 == 0:
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            stamps = [self._stamp_nanoseconds(message) for message in messages]
            sync_delta_ms = (max(stamps) - min(stamps)) / 1.0e6
            self.get_logger().info(
                f"published BEV frame {self._frame_count} in "
                f"{elapsed_ms:.1f} ms, sync span {sync_delta_ms:.1f} ms"
            )

    def _mask_vehicle(self, image):
        front_row = int(
            round(
                (self._extent_m - self._vehicle_length_m / 2.0)
                / self._resolution_m
            )
        )
        rear_row = int(
            round(
                (self._extent_m + self._vehicle_length_m / 2.0)
                / self._resolution_m
            )
        )
        left_column = int(
            round(
                (self._extent_m - self._vehicle_width_m / 2.0)
                / self._resolution_m
            )
        )
        right_column = int(
            round(
                (self._extent_m + self._vehicle_width_m / 2.0)
                / self._resolution_m
            )
        )
        cv2.rectangle(
            image,
            (left_column, front_row),
            (right_column, rear_row),
            (35, 35, 35),
            thickness=-1,
        )
        cv2.rectangle(
            image,
            (left_column, front_row),
            (right_column, rear_row),
            (220, 220, 220),
            thickness=1,
        )

    def _publish_image(self, image, encoding, stamp, publisher):
        message = self._bridge.cv2_to_imgmsg(image, encoding=encoding)
        message.header.stamp = stamp
        message.header.frame_id = self._base_frame
        publisher.publish(message)

    def _on_pointcloud(self, message):
        field_names = {field.name for field in message.fields}
        if not {"x", "y", "z"}.issubset(field_names):
            self.get_logger().warning(
                "PointCloud2 does not contain x/y/z fields",
                throttle_duration_sec=5.0,
            )
            return
        try:
            points = point_cloud2.read_points(
                message, field_names=["x", "y", "z"], skip_nans=True
            )
            xyz = np.column_stack((points["x"], points["y"], points["z"]))
            xyz = xyz[:: self._pointcloud_stride].astype(np.float64, copy=False)
            xyz = self._transform_points(xyz, message)
        except (AssertionError, TransformException, ValueError) as error:
            self.get_logger().warning(
                f"failed to transform point cloud: {error}",
                throttle_duration_sec=2.0,
            )
            return
        if xyz.size == 0:
            return

        finite = np.all(np.isfinite(xyz), axis=1)
        height_valid = (xyz[:, 2] >= self._obstacle_min_height_m) & (
            xyz[:, 2] <= self._obstacle_max_height_m
        )
        range_valid = np.hypot(xyz[:, 0], xyz[:, 1]) <= self._extent_m
        outside_vehicle = (
            (np.abs(xyz[:, 0]) > self._vehicle_length_m / 2.0)
            | (np.abs(xyz[:, 1]) > self._vehicle_width_m / 2.0)
        )
        xyz = xyz[finite & height_valid & range_valid & outside_vehicle]

        obstacle_mask = np.zeros(
            (self._grid_size, self._grid_size), dtype=bool
        )
        if xyz.size:
            x_indices = np.floor(
                (xyz[:, 0] + self._extent_m) / self._resolution_m
            ).astype(np.int64)
            y_indices = np.floor(
                (xyz[:, 1] + self._extent_m) / self._resolution_m
            ).astype(np.int64)
            inside = (
                (x_indices >= 0)
                & (x_indices < self._grid_size)
                & (y_indices >= 0)
                & (y_indices < self._grid_size)
            )
            obstacle_mask[y_indices[inside], x_indices[inside]] = True
        self._last_obstacle_mask = obstacle_mask
        self._last_cloud_stamp_ns = self._stamp_nanoseconds(message)

    def _transform_points(self, xyz, message):
        if not message.header.frame_id:
            raise ValueError("PointCloud2 frame_id is empty")
        if message.header.frame_id == self._base_frame:
            return xyz
        transform = self._tf_buffer.lookup_transform(
            self._base_frame,
            message.header.frame_id,
            Time.from_msg(message.header.stamp),
            timeout=Duration(seconds=0.05),
        ).transform
        rotation = transform.rotation
        matrix = quaternion_matrix(
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        translation = np.array(
            [
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ],
            dtype=np.float64,
        )
        return xyz @ matrix.T + translation

    def _publish_occupancy_grid(self, stamp):
        data = np.full(
            (self._grid_size, self._grid_size), -1, dtype=np.int8
        )
        data[self._occupancy_valid] = 0
        image_stamp_ns = self._stamp_nanoseconds_from_stamp(stamp)
        cloud_is_fresh = (
            self._last_cloud_stamp_ns is not None
            and abs(image_stamp_ns - self._last_cloud_stamp_ns)
            <= self._pointcloud_timeout_sec * 1.0e9
        )
        if self._fuse_pointcloud:
            if cloud_is_fresh and self._last_obstacle_mask is not None:
                data[self._last_obstacle_mask] = 100
            else:
                # A stale obstacle source must not clear navigation space.
                data[self._occupancy_valid] = -1

        message = OccupancyGrid()
        message.header.stamp = stamp
        message.header.frame_id = self._base_frame
        message.info.map_load_time = stamp
        message.info.resolution = self._resolution_m
        message.info.width = self._grid_size
        message.info.height = self._grid_size
        message.info.origin.position.x = -self._extent_m
        message.info.origin.position.y = -self._extent_m
        message.info.origin.orientation.w = 1.0
        message.data = data.ravel().tolist()
        self._grid_publisher.publish(message)

    @staticmethod
    def _stamp_nanoseconds(message):
        return SurroundBevNode._stamp_nanoseconds_from_stamp(message.header.stamp)

    @staticmethod
    def _stamp_nanoseconds_from_stamp(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main(args=None):
    rclpy.init(args=args)
    node = SurroundBevNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
