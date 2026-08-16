#!/usr/bin/env python3
"""ROS 2 node and dependency-free HTTP/SSE server for the mobile PWA."""

from __future__ import annotations

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
import json
import math
import mimetypes
import os
from pathlib import Path
import socket
import shutil
import threading
import time
from collections import defaultdict, deque
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlparse

from geometry_msgs.msg import (
    PolygonStamped,
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
)
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from scout_msgs.msg import ScoutStatus
from sensor_msgs.msg import CameraInfo, Imu, NavSatFix, PointCloud2
from std_msgs.msg import Bool, String, UInt8

from .catalog import (
    BagCatalog,
    CatalogError,
    GridData,
    MapCatalog,
    quaternion_yaw,
    validated_identifier,
)
from .processes import ProcessError, ProcessSlots
from .profiles import ProfileError, RuntimeProfiles


MAX_JSON_BODY = 256 * 1024
MAX_ROUTE_POSES = 100


class ApiError(RuntimeError):
    def __init__(self, status: int, message: str):
        super().__init__(message)
        self.status = status


class RateTracker:
    def __init__(self):
        self._lock = threading.Lock()
        self._arrivals: dict[str, deque[float]] = defaultdict(deque)

    def mark(self, topic: str) -> None:
        now = time.monotonic()
        with self._lock:
            values = self._arrivals[topic]
            values.append(now)
            while values and now - values[0] > 10.0:
                values.popleft()

    def snapshot(self) -> dict[str, dict]:
        now = time.monotonic()
        result = {}
        with self._lock:
            for topic, values in self._arrivals.items():
                while values and now - values[0] > 10.0:
                    values.popleft()
                if not values:
                    continue
                rate = 0.0
                if len(values) >= 2 and values[-1] > values[0]:
                    rate = (len(values) - 1) / (values[-1] - values[0])
                result[topic] = {
                    "hz": round(rate, 2),
                    "age_sec": round(now - values[-1], 2),
                }
        return result


def finite_number(value, description: str) -> float:
    if isinstance(value, bool):
        raise ApiError(HTTPStatus.BAD_REQUEST, f"{description}必须是有限数值")
    try:
        converted = float(value)
    except (TypeError, ValueError) as error:
        raise ApiError(HTTPStatus.BAD_REQUEST, f"{description}必须是有限数值") from error
    if not math.isfinite(converted) or abs(converted) > 100000.0:
        raise ApiError(HTTPStatus.BAD_REQUEST, f"{description}超出允许范围")
    return converted


def pose_document(value: object) -> dict:
    if not isinstance(value, dict):
        raise ApiError(HTTPStatus.BAD_REQUEST, "位姿必须是JSON对象")
    return {
        "x": finite_number(value.get("x"), "X坐标"),
        "y": finite_number(value.get("y"), "Y坐标"),
        "yaw": finite_number(value.get("yaw", 0.0), "航向角"),
    }


def action_status_name(status: int) -> str:
    return {
        GoalStatus.STATUS_UNKNOWN: "unknown",
        GoalStatus.STATUS_ACCEPTED: "accepted",
        GoalStatus.STATUS_EXECUTING: "executing",
        GoalStatus.STATUS_CANCELING: "canceling",
        GoalStatus.STATUS_SUCCEEDED: "succeeded",
        GoalStatus.STATUS_CANCELED: "canceled",
        GoalStatus.STATUS_ABORTED: "aborted",
    }.get(status, "unknown")


class GatewayHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, gateway, static_root):
        self.gateway = gateway
        self.static_root = static_root
        super().__init__(address, GatewayRequestHandler)


class GatewayRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "AgribotMobile/0.1"

    @property
    def gateway(self):
        return self.server.gateway

    def log_message(self, format_string, *args):
        self.gateway.get_logger().debug(format_string % args)

    def _headers(self, content_type: str, length: int | None = None, cache=False):
        self.send_header("Content-Type", content_type)
        if length is not None:
            self.send_header("Content-Length", str(length))
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; connect-src 'self'; img-src 'self' data:; "
            "style-src 'self' 'unsafe-inline'; script-src 'self'",
        )
        self.send_header(
            "Cache-Control",
            "public, max-age=31536000, immutable" if cache else "no-store",
        )

    def _json(self, status: int, document: object):
        payload = json.dumps(
            document, ensure_ascii=False, separators=(",", ":")
        ).encode("utf-8")
        self.send_response(status)
        self._headers("application/json; charset=utf-8", len(payload))
        self.end_headers()
        self.wfile.write(payload)

    def _error(self, status: int, message: str):
        self._json(status, {"ok": False, "error": message})

    def _body(self) -> dict:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as error:
            raise ApiError(HTTPStatus.BAD_REQUEST, "Content-Length无效") from error
        if length <= 0 or length > MAX_JSON_BODY:
            raise ApiError(HTTPStatus.BAD_REQUEST, "请求体为空或过大")
        try:
            document = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeError, json.JSONDecodeError) as error:
            raise ApiError(HTTPStatus.BAD_REQUEST, "请求体不是有效JSON") from error
        if not isinstance(document, dict):
            raise ApiError(HTTPStatus.BAD_REQUEST, "请求体必须是JSON对象")
        return document

    def _require_authorization(self):
        expected = self.gateway.api_token
        if expected and self.headers.get("X-Agribot-Token", "") != expected:
            raise ApiError(HTTPStatus.UNAUTHORIZED, "控制口令错误")

    def do_GET(self):
        parsed = urlparse(self.path)
        try:
            if parsed.path == "/api/v1/state":
                self._json(HTTPStatus.OK, self.gateway.state_snapshot())
                return
            if parsed.path == "/api/v1/events":
                self._events()
                return
            if parsed.path == "/api/v1/maps":
                self._json(HTTPStatus.OK, {"maps": self.gateway.map_catalog.list()})
                return
            if parsed.path == "/api/v1/bags":
                self._json(HTTPStatus.OK, {"bags": self.gateway.bag_catalog.list()})
                return
            if parsed.path == "/api/v1/profiles":
                self._json(
                    HTTPStatus.OK,
                    {
                        "profiles": self.gateway.runtime_profiles.public(),
                        "processing_enabled": self.gateway.processing_enabled,
                    },
                )
                return
            if parsed.path == "/api/v1/grid":
                query = parse_qs(parsed.query)
                layer = query.get("layer", ["map"])[0]
                self._json(HTTPStatus.OK, self.gateway.live_grid(layer).payload(layer))
                return
            prefix = "/api/v1/maps/"
            suffix = "/grid"
            if parsed.path.startswith(prefix) and parsed.path.endswith(suffix):
                map_id = unquote(parsed.path[len(prefix) : -len(suffix)]).strip("/")
                grid = self.gateway.map_catalog.grid(map_id)
                self._json(HTTPStatus.OK, grid.payload("map"))
                return
            self._static(parsed.path)
        except ApiError as error:
            self._error(error.status, str(error))
        except (CatalogError, ProfileError, ProcessError) as error:
            self._error(HTTPStatus.BAD_REQUEST, str(error))
        except BrokenPipeError:
            return
        except Exception as error:  # pragma: no cover - defensive HTTP boundary
            self.gateway.get_logger().error(f"GET请求处理失败: {error}")
            self._error(HTTPStatus.INTERNAL_SERVER_ERROR, str(error))

    def do_POST(self):
        parsed = urlparse(self.path)
        try:
            self._require_authorization()
            body = self._body()
            result = self.gateway.handle_command(parsed.path, body)
            self._json(HTTPStatus.OK, {"ok": True, **(result or {})})
        except ApiError as error:
            self._error(error.status, str(error))
        except (CatalogError, ProfileError, ProcessError) as error:
            self._error(HTTPStatus.BAD_REQUEST, str(error))
        except Exception as error:  # pragma: no cover - defensive HTTP boundary
            self.gateway.get_logger().error(f"POST请求处理失败: {error}")
            self._error(HTTPStatus.INTERNAL_SERVER_ERROR, str(error))

    def _events(self):
        try:
            self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            self.connection.settimeout(5.0)
            if hasattr(socket, "TCP_KEEPIDLE"):
                self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 15)
                self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 5)
                self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
        except OSError:
            pass
        self.send_response(HTTPStatus.OK)
        self._headers("text/event-stream; charset=utf-8")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        self.wfile.write(b"retry: 3000\n\n")
        self.wfile.flush()
        revision = -1
        while not self.gateway.stopping:
            revision, payload = self.gateway.wait_for_event(revision, 10.0)
            try:
                self.wfile.write(b"event: state\n")
                self.wfile.write(b"data: " + payload + b"\n\n")
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                return

    def _static(self, request_path: str):
        relative = "index.html" if request_path in ("", "/") else unquote(request_path[1:])
        relative_path = Path(relative)
        if relative_path.is_absolute() or ".." in relative_path.parts:
            raise ApiError(HTTPStatus.NOT_FOUND, "文件不存在")
        root = self.server.static_root
        candidate = root / relative_path
        if not candidate.is_file():
            candidate = root / "index.html"
        if not candidate.is_file():
            raise ApiError(HTTPStatus.SERVICE_UNAVAILABLE, "手机端静态文件尚未构建")
        payload = candidate.read_bytes()
        content_type = mimetypes.guess_type(candidate.name)[0] or "application/octet-stream"
        if content_type.startswith("text/") or content_type in (
            "application/javascript",
            "application/manifest+json",
        ):
            content_type += "; charset=utf-8"
        cache = "/assets/" in request_path or "/icons/" in request_path
        self.send_response(HTTPStatus.OK)
        self._headers(content_type, len(payload), cache=cache)
        self.end_headers()
        self.wfile.write(payload)


class MobileGateway(Node):
    def __init__(self):
        super().__init__("mobile_gateway")
        share = Path(get_package_share_directory("agribot_mobile_app"))
        self.http_host = str(self.declare_parameter("http_host", "0.0.0.0").value)
        self.http_port = int(self.declare_parameter("http_port", 8088).value)
        self.api_token = str(self.declare_parameter("api_token", "").value)
        self.map_root = Path(
            str(self.declare_parameter("map_root", str(Path.home() / "agribot_maps")).value)
        ).expanduser()
        self.bag_root = Path(
            str(self.declare_parameter("bag_root", str(Path.home() / "agribot_bags")).value)
        ).expanduser()
        self.required_collection_mount = str(
            self.declare_parameter("required_collection_mount", "").value
        )
        self.process_log_root = Path(
            str(
                self.declare_parameter(
                    "process_log_root", str(Path.home() / "agribot_mobile_logs")
                ).value
            )
        ).expanduser()
        self.processing_enabled = bool(
            self.declare_parameter("processing_enabled", False).value
        )
        self.processing_domain_id = int(
            self.declare_parameter("processing_domain_id", 71).value
        )
        self.processing_playback_rate = float(
            self.declare_parameter("processing_playback_rate", 0.5).value
        )
        self.static_root = Path(
            str(self.declare_parameter("static_root", str(share / "web")).value)
        )
        profile_path = Path(
            str(
                self.declare_parameter(
                    "runtime_profiles", str(share / "config" / "runtime_profiles.yaml")
                ).value
            )
        )

        self.pose_topic = str(
            self.declare_parameter("pose_topic", "/fastlivo_rtk/odometry").value
        )
        self.global_plan_topic = str(
            self.declare_parameter("global_plan_topic", "/plan").value
        )
        self.local_plan_topic = str(
            self.declare_parameter(
                "local_plan_topic", "/transformed_global_plan"
            ).value
        )
        self.trajectory_topic = str(
            self.declare_parameter(
                "trajectory_topic", "/fastlivo_rtk/path"
            ).value
        )
        self.footprint_topic = str(
            self.declare_parameter(
                "footprint_topic", "/local_costmap/published_footprint"
            ).value
        )
        self.camera_rate_topic = str(
            self.declare_parameter(
                "camera_rate_topic", "/camera/rgb/camera_info"
            ).value
        )
        flat_footprint = list(
            self.declare_parameter(
                "vehicle_footprint",
                [
                    0.754818,
                    0.485974,
                    0.754818,
                    -0.485974,
                    -0.227500,
                    -0.485974,
                    -0.227500,
                    0.485974,
                ],
            ).value
        )
        if len(flat_footprint) < 6 or len(flat_footprint) % 2:
            raise ValueError("vehicle_footprint必须包含至少三个二维顶点")
        self.vehicle_footprint = [
            [float(flat_footprint[index]), float(flat_footprint[index + 1])]
            for index in range(0, len(flat_footprint), 2)
        ]
        self.grid_topics = {
            "map": str(self.declare_parameter("map_topic", "/map").value),
            "global_costmap": str(
                self.declare_parameter(
                    "global_costmap_topic", "/global_costmap/costmap"
                ).value
            ),
            "local_costmap": str(
                self.declare_parameter(
                    "local_costmap_topic", "/local_costmap/costmap"
                ).value
            ),
        }

        self.map_catalog = MapCatalog(self.map_root)
        self.bag_catalog = BagCatalog(self.bag_root)
        self.runtime_profiles = RuntimeProfiles(profile_path)
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._revision = 0
        self._stopping = False
        self._http_server = None
        self._http_thread = None
        self._rates = RateTracker()
        self._snapshot_lock = threading.Lock()
        self._snapshot_cache_revision = -1
        self._snapshot_cache = None
        self._event_cache_revision = -1
        self._event_cache = None
        self._grids: dict[str, GridData] = {}
        self._grid_revisions = defaultdict(int)
        self._state = {
            "pose": None,
            "paths": {"history": [], "global": [], "local": []},
            "footprint": None,
            "vehicle": {"footprint": self.vehicle_footprint},
            "localization": {
                "ready": None,
                "lidar_ready": None,
                "fusion_ready": None,
                "fixed_active": None,
                "rtk_seed_ready": None,
                "status": "未收到",
                "rtk_initializer_status": "未收到",
                "fix_quality": None,
                "heading_solution": "未收到",
            },
            "chassis": None,
            "command": {"linear": 0.0, "angular": 0.0},
            "navigation": {
                "kind": None,
                "status": "idle",
                "feedback": {},
                "goal": None,
                "route": [],
            },
            "active_runtime": None,
            "active_collection": None,
            "active_processing": None,
        }
        self.processes = ProcessSlots(self._touch)
        self._goal_handle = None
        self._path_updates: dict[str, float] = {}

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        normal = QoSProfile(depth=20)
        self._create_counted_subscription(
            PointCloud2, "/lidar/points", qos_profile_sensor_data
        )
        self._create_counted_subscription(Imu, "/imu/data", qos_profile_sensor_data)
        self._create_counted_subscription(
            CameraInfo,
            self.camera_rate_topic,
            qos_profile_sensor_data,
            reported_topic="/camera/rgb/image_raw",
        )
        self._create_counted_subscription(
            NavSatFix, "/rtk/fix", qos_profile_sensor_data
        )
        self._create_counted_subscription(Odometry, "/wheel/odometry", normal)
        self.create_subscription(Odometry, self.pose_topic, self._pose_callback, normal)
        self.create_subscription(
            NavPath,
            self.global_plan_topic,
            self._path_callback("global", self.global_plan_topic, 1200),
            normal,
        )
        self.create_subscription(
            NavPath,
            self.local_plan_topic,
            self._path_callback("local", self.local_plan_topic, 400, 0.15),
            normal,
        )
        self.create_subscription(
            NavPath,
            self.trajectory_topic,
            self._path_callback("history", self.trajectory_topic, 600, 0.5),
            latched,
        )
        self.create_subscription(
            PolygonStamped,
            self.footprint_topic,
            self._footprint_callback,
            latched,
        )
        for layer, topic in self.grid_topics.items():
            self.create_subscription(
                OccupancyGrid, topic, self._grid_callback(layer, topic), latched
            )
        self.create_subscription(Bool, "/localization/ready", self._value_callback("ready"), latched)
        self.create_subscription(Bool, "/localization/lidar_ready", self._value_callback("lidar_ready"), latched)
        self.create_subscription(Bool, "/fastlivo_rtk/ready", self._value_callback("fusion_ready"), latched)
        self.create_subscription(Bool, "/fastlivo_rtk/fixed_active", self._value_callback("fixed_active"), latched)
        self.create_subscription(Bool, "/localization/rtk_seed_ready", self._value_callback("rtk_seed_ready"), latched)
        self.create_subscription(String, "/localization/status", self._value_callback("status"), latched)
        self.create_subscription(
            String,
            "/localization/rtk_initializer_status",
            self._value_callback("rtk_initializer_status"),
            latched,
        )
        self.create_subscription(UInt8, "/rtk/fix_quality", self._value_callback("fix_quality"), normal)
        self.create_subscription(String, "/rtk/heading_solution", self._value_callback("heading_solution"), normal)
        self.create_subscription(ScoutStatus, "/scout_status", self._chassis_callback, normal)
        self.create_subscription(Twist, "/nav2/cmd_vel", self._command_callback, normal)
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._external_goal_status,
            normal,
        )
        self.create_subscription(
            GoalStatusArray,
            "/navigate_through_poses/_action/status",
            self._external_goal_status,
            normal,
        )

        self._initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self._navigate_to_pose = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._navigate_through_poses = ActionClient(
            self, NavigateThroughPoses, "navigate_through_poses"
        )
        self._clear_global = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap"
        )
        self._clear_local = self.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap"
        )
        self._cancel_external_goal_clients = (
            self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal"),
            self.create_client(
                CancelGoal, "/navigate_through_poses/_action/cancel_goal"
            ),
        )
        # Coalesce high-rate ROS callbacks into one mobile state update. Five Hz is
        # responsive enough for the map while leaving localization CPU headroom.
        self.create_timer(0.2, self._touch)

    @property
    def stopping(self) -> bool:
        return self._stopping

    def _touch(self) -> None:
        with self._condition:
            self._revision += 1
            self._condition.notify_all()

    def _create_counted_subscription(
        self, message_type, topic, qos, reported_topic: str | None = None
    ):
        rate_topic = reported_topic or topic

        def callback(_message):
            self._rates.mark(rate_topic)

        # Raw subscriptions count serialized arrivals without constructing large
        # Python Image or PointCloud2 objects.
        self.create_subscription(message_type, topic, callback, qos, raw=True)

    def _value_callback(self, name):
        topic = {
            "fix_quality": "/rtk/fix_quality",
            "heading_solution": "/rtk/heading_solution",
        }.get(name)

        def callback(message):
            if topic:
                self._rates.mark(topic)
            with self._lock:
                self._state["localization"][name] = message.data

        return callback

    def _pose_callback(self, message: Odometry):
        self._rates.mark(self.pose_topic)
        pose = message.pose.pose
        twist = message.twist.twist
        with self._lock:
            self._state["pose"] = {
                "frame": message.header.frame_id,
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "yaw": quaternion_yaw(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
                "linear_speed": math.hypot(twist.linear.x, twist.linear.y),
                "angular_speed": twist.angular.z,
            }

    def _path_callback(
        self,
        name: str,
        topic: str,
        maximum_points: int,
        minimum_interval: float = 0.0,
    ):
        def callback(message: NavPath):
            self._rates.mark(topic)
            now = time.monotonic()
            if now - self._path_updates.get(name, 0.0) < minimum_interval:
                return
            self._path_updates[name] = now
            poses = message.poses
            step = max(1, math.ceil(len(poses) / maximum_points))
            points = [
                [pose.pose.position.x, pose.pose.position.y]
                for pose in poses[::step]
            ]
            if poses and (len(poses) - 1) % step:
                points.append(
                    [poses[-1].pose.position.x, poses[-1].pose.position.y]
                )
            with self._lock:
                self._state["paths"][name] = points

        return callback

    def _footprint_callback(self, message: PolygonStamped):
        self._rates.mark(self.footprint_topic)
        points = [[point.x, point.y] for point in message.polygon.points]
        if len(points) < 3:
            return
        with self._lock:
            self._state["footprint"] = {
                "frame": message.header.frame_id,
                "points": points,
            }

    def _grid_callback(self, layer: str, topic: str):
        def callback(message: OccupancyGrid):
            self._rates.mark(topic)
            orientation = message.info.origin.orientation
            self._grid_revisions[layer] += 1
            try:
                grid_data = memoryview(message.data).cast("B").tobytes()
            except TypeError:
                grid_data = bytes(value & 0xFF for value in message.data)
            grid = GridData(
                width=message.info.width,
                height=message.info.height,
                resolution=message.info.resolution,
                origin_x=message.info.origin.position.x,
                origin_y=message.info.origin.position.y,
                origin_yaw=quaternion_yaw(
                    orientation.x, orientation.y, orientation.z, orientation.w
                ),
                data=grid_data,
                revision=self._grid_revisions[layer],
            )
            with self._lock:
                self._grids[layer] = grid

        return callback

    def _chassis_callback(self, message: ScoutStatus):
        self._rates.mark("/scout_status")
        with self._lock:
            self._state["chassis"] = {
                "linear_velocity": message.linear_velocity,
                "angular_velocity": message.angular_velocity,
                "control_mode": int(message.control_mode),
                "base_state": int(message.base_state),
                "fault_code": int(message.fault_code),
                "battery_voltage": message.battery_voltage,
            }

    def _command_callback(self, message: Twist):
        self._rates.mark("/nav2/cmd_vel")
        with self._lock:
            self._state["command"] = {
                "linear": message.linear.x,
                "angular": message.angular.z,
            }

    def _external_goal_status(self, message: GoalStatusArray):
        active = [
            status.status
            for status in message.status_list
            if status.status
            in (
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_CANCELING,
            )
        ]
        if active and self._goal_handle is None:
            with self._lock:
                self._state["navigation"]["status"] = action_status_name(active[-1])
            self._touch()

    def live_grid(self, layer: str) -> GridData:
        if layer not in self.grid_topics:
            raise ApiError(HTTPStatus.BAD_REQUEST, "未知地图图层")
        with self._lock:
            grid = self._grids.get(layer)
        if grid is None:
            raise ApiError(HTTPStatus.NOT_FOUND, "该地图图层尚未发布")
        return grid

    def _grid_metadata(self) -> dict:
        with self._lock:
            return {
                layer: {
                    "revision": grid.revision,
                    "width": grid.width,
                    "height": grid.height,
                    "resolution": grid.resolution,
                }
                for layer, grid in self._grids.items()
            }

    def _disk_state(self) -> dict:
        result = {}
        for name, path in (("bags", self.bag_root), ("maps", self.map_root)):
            existing = path
            while not existing.exists() and existing != existing.parent:
                existing = existing.parent
            try:
                usage = shutil.disk_usage(existing)
                result[name] = {
                    "root": str(path),
                    "free_bytes": usage.free,
                    "total_bytes": usage.total,
                }
            except OSError:
                result[name] = {"root": str(path), "free_bytes": None, "total_bytes": None}
        return result

    def state_snapshot(self) -> dict:
        with self._snapshot_lock:
            with self._lock:
                revision = self._revision
                if self._snapshot_cache_revision == revision:
                    return self._snapshot_cache
                core = json.loads(json.dumps(self._state, ensure_ascii=False))
            document = {
                "revision": revision,
                "server_time": time.time(),
                "ros": {
                    "node": self.get_name(),
                    "domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
                },
                **core,
                "rates": self._rates.snapshot(),
                "grids": self._grid_metadata(),
                "processes": self.processes.snapshot(),
                "storage": self._disk_state(),
            }
            self._snapshot_cache_revision = revision
            self._snapshot_cache = document
            return document

    def event_payload(self) -> tuple[int, bytes]:
        document = self.state_snapshot()
        revision = int(document["revision"])
        with self._snapshot_lock:
            if self._event_cache_revision != revision:
                self._event_cache = json.dumps(
                    document, ensure_ascii=False, separators=(",", ":")
                ).encode("utf-8")
                self._event_cache_revision = revision
            return revision, self._event_cache

    def wait_for_state(self, previous_revision: int, timeout: float):
        with self._condition:
            if self._revision == previous_revision and not self._stopping:
                self._condition.wait(timeout=timeout)
        return self._revision, self.state_snapshot()

    def wait_for_event(self, previous_revision: int, timeout: float):
        with self._condition:
            if self._revision == previous_revision and not self._stopping:
                self._condition.wait(timeout=timeout)
        return self.event_payload()

    def start_http(self) -> None:
        self._http_server = GatewayHttpServer(
            (self.http_host, self.http_port), self, self.static_root
        )
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever,
            name="agribot-mobile-http",
            daemon=True,
        )
        self._http_thread.start()
        self.get_logger().info(
            f"手机端已启动: http://{self.http_host}:{self.http_port}"
        )
        if not self.api_token:
            self.get_logger().warning("未配置api_token，仅应在受控局域网内使用")

    def handle_command(self, path: str, body: dict) -> dict | None:
        handlers = {
            "/api/v1/localization/initial-pose": self.publish_initial_pose,
            "/api/v1/navigation/goal": self.send_navigation_goal,
            "/api/v1/navigation/route": self.send_navigation_route,
            "/api/v1/navigation/cancel": self.cancel_navigation,
            "/api/v1/navigation/clear-costmaps": self.clear_costmaps,
            "/api/v1/collection/start": self.start_collection,
            "/api/v1/collection/stop": self.stop_collection,
            "/api/v1/processing/start": self.start_processing,
            "/api/v1/processing/stop": self.stop_processing,
            "/api/v1/runtime/start": self.start_runtime,
            "/api/v1/runtime/stop": self.stop_runtime,
        }
        try:
            handler = handlers[path]
        except KeyError as error:
            raise ApiError(HTTPStatus.NOT_FOUND, "未知控制接口") from error
        return handler(body)

    @staticmethod
    def _stamped_pose(node: Node, pose: dict) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = node.get_clock().now().to_msg()
        message.pose.position.x = pose["x"]
        message.pose.position.y = pose["y"]
        message.pose.orientation.z = math.sin(pose["yaw"] / 2.0)
        message.pose.orientation.w = math.cos(pose["yaw"] / 2.0)
        return message

    def publish_initial_pose(self, body: dict) -> dict:
        pose = pose_document(body.get("pose"))
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose = self._stamped_pose(self, pose).pose
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = math.radians(15.0) ** 2
        self._initial_pose_publisher.publish(message)
        return {"pose": pose}

    def _assert_navigation_ready(self):
        with self._lock:
            ready = self._state["localization"].get("fusion_ready")
        if ready is not True:
            raise ApiError(HTTPStatus.CONFLICT, "融合定位尚未就绪，禁止下发导航任务")
        if self._goal_handle is not None:
            raise ApiError(HTTPStatus.CONFLICT, "已有导航任务正在执行")

    def send_navigation_goal(self, body: dict) -> dict:
        self._assert_navigation_ready()
        if not self._navigate_to_pose.wait_for_server(timeout_sec=0.2):
            raise ApiError(HTTPStatus.CONFLICT, "NavigateToPose服务器尚未就绪")
        pose = pose_document(body.get("pose"))
        goal = NavigateToPose.Goal()
        goal.pose = self._stamped_pose(self, pose)
        with self._lock:
            self._state["navigation"] = {
                "kind": "goal",
                "status": "sending",
                "feedback": {},
                "goal": pose,
                "route": [],
            }
        future = self._navigate_to_pose.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)
        self._touch()
        return {"goal": pose}

    def send_navigation_route(self, body: dict) -> dict:
        self._assert_navigation_ready()
        values = body.get("poses")
        if not isinstance(values, list) or not 2 <= len(values) <= MAX_ROUTE_POSES:
            raise ApiError(HTTPStatus.BAD_REQUEST, "连续路线必须包含2至100个位姿")
        route = [pose_document(value) for value in values]
        if not self._navigate_through_poses.wait_for_server(timeout_sec=0.2):
            raise ApiError(HTTPStatus.CONFLICT, "NavigateThroughPoses服务器尚未就绪")
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._stamped_pose(self, pose) for pose in route]
        with self._lock:
            self._state["navigation"] = {
                "kind": "route",
                "status": "sending",
                "feedback": {},
                "goal": route[-1],
                "route": route,
            }
        future = self._navigate_through_poses.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)
        self._touch()
        return {"poses": route}

    def _goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:  # pragma: no cover - rclpy boundary
            with self._lock:
                self._state["navigation"]["status"] = "failed"
                self._state["navigation"]["feedback"] = {"message": str(error)}
            self._touch()
            return
        if not goal_handle.accepted:
            with self._lock:
                self._state["navigation"]["status"] = "rejected"
            self._touch()
            return
        self._goal_handle = goal_handle
        with self._lock:
            self._state["navigation"]["status"] = "executing"
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)
        self._touch()

    def _feedback_callback(self, feedback_message):
        feedback = feedback_message.feedback
        document = {}
        for name in ("distance_remaining", "number_of_recoveries", "number_of_poses_remaining"):
            if hasattr(feedback, name):
                value = getattr(feedback, name)
                document[name] = float(value) if isinstance(value, float) else int(value)
        with self._lock:
            self._state["navigation"]["feedback"] = document
        self._touch()

    def _goal_result_callback(self, future):
        try:
            wrapped = future.result()
            status = action_status_name(wrapped.status)
        except Exception as error:  # pragma: no cover - rclpy boundary
            status = "failed"
            with self._lock:
                self._state["navigation"]["feedback"] = {"message": str(error)}
        self._goal_handle = None
        with self._lock:
            self._state["navigation"]["status"] = status
        self._touch()

    def cancel_navigation(self, _body: dict) -> dict:
        handle = self._goal_handle
        if handle is None:
            requested = False
            for client in self._cancel_external_goal_clients:
                if client.wait_for_service(timeout_sec=0.1):
                    client.call_async(CancelGoal.Request())
                    requested = True
            with self._lock:
                self._state["navigation"]["status"] = (
                    "canceling" if requested else "idle"
                )
            self._touch()
            return {"status": "canceling" if requested else "idle"}
        handle.cancel_goal_async()
        with self._lock:
            self._state["navigation"]["status"] = "canceling"
        self._touch()
        return {"status": "canceling"}

    def clear_costmaps(self, _body: dict) -> dict:
        called = []
        for name, client in (("global", self._clear_global), ("local", self._clear_local)):
            if client.wait_for_service(timeout_sec=0.2):
                client.call_async(ClearEntireCostmap.Request())
                called.append(name)
        if len(called) != 2:
            raise ApiError(HTTPStatus.CONFLICT, "全局或局部代价地图清除服务尚未就绪")
        return {"cleared": called}

    def _log_path(self, category: str, identifier: str) -> Path:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return self.process_log_root / category / f"{identifier}_{timestamp}.log"

    def start_collection(self, body: dict) -> dict:
        self.processes.assert_exclusive("collection")
        map_name = validated_identifier(str(body.get("map_name", "")), "地图名称")
        if self.required_collection_mount:
            mount = Path(self.required_collection_mount)
            if not mount.is_mount():
                raise ApiError(
                    HTTPStatus.CONFLICT,
                    f"采集固态硬盘未挂载: {mount}",
                )
        self.bag_root.mkdir(parents=True, exist_ok=True)
        bag_id = f"{map_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        bag_path = self.bag_root / bag_id
        command = [
            "ros2",
            "launch",
            "agribot_hardware_bringup",
            "ackermann_sensor_data_collection.launch.py",
            f"start_camera:={'true' if body.get('start_camera', True) else 'false'}",
            f"enable_ntrip:={'true' if body.get('enable_ntrip', False) else 'false'}",
            "record_bag:=true",
            f"bag_output:={bag_path}",
        ]
        self.processes.collection.start(
            command, self._log_path("collection", bag_id), os.environ.copy()
        )
        with self._lock:
            self._state["active_collection"] = {
                "bag_id": bag_id,
                "bag_path": str(bag_path),
                "map_name": map_name,
            }
        self._touch()
        return {"bag_id": bag_id, "bag_path": str(bag_path)}

    def stop_collection(self, _body: dict) -> dict:
        self.processes.collection.stop()
        with self._lock:
            collection = self._state["active_collection"]
            self._state["active_collection"] = None
        self._touch()
        if collection:
            metadata = Path(collection["bag_path"]) / "metadata.yaml"
            return {"bag_id": collection["bag_id"], "complete": metadata.is_file()}
        return {"complete": False}

    def start_processing(self, body: dict) -> dict:
        if not self.processing_enabled:
            raise ApiError(HTTPStatus.FORBIDDEN, "本机未启用Jetson离线处理")
        self.processes.assert_exclusive("processing")
        bag_id = str(body.get("bag_id", ""))
        bag_path = self.bag_catalog.path(bag_id)
        map_name = validated_identifier(str(body.get("map_name", "")), "地图名称")
        map_base = self.map_root / map_name
        command = [
            "ros2",
            "run",
            "agribot_offline_mapping",
            "run_rtk_mapping_pipeline.py",
            str(bag_path),
            str(map_base),
            "--domain-id",
            str(self.processing_domain_id),
            "--playback-rate",
            str(self.processing_playback_rate),
        ]
        without_rtk = bool(body.get("without_rtk", False))
        if without_rtk:
            command.append("--without-rtk")
        environment = os.environ.copy()
        self.processes.processing.start(
            command, self._log_path("processing", map_name), environment
        )
        with self._lock:
            self._state["active_processing"] = {
                "bag_id": bag_id,
                "map_name": map_name,
                "without_rtk": without_rtk,
            }
        self._touch()
        return {"map_name": map_name}

    def stop_processing(self, _body: dict) -> dict:
        self.processes.processing.stop()
        with self._lock:
            processing = self._state["active_processing"]
            self._state["active_processing"] = None
        self._touch()
        return {"processing": processing}

    def start_runtime(self, body: dict) -> dict:
        self.processes.assert_exclusive("runtime")
        profile_id = str(body.get("profile_id", ""))
        map_id = str(body.get("map_id", ""))
        motion = bool(body.get("motion", False))
        if motion and body.get("motion_confirmed") is not True:
            raise ApiError(HTTPStatus.BAD_REQUEST, "真车运动必须再次明确确认")
        map_base = self.map_catalog.map_base(map_id)
        command = self.runtime_profiles.command(profile_id, map_base, motion)
        self.processes.runtime.start(
            command, self._log_path("runtime", f"{profile_id}_{map_id}"), os.environ.copy()
        )
        with self._lock:
            self._state["pose"] = None
            self._state["paths"] = {"history": [], "global": [], "local": []}
            self._state["footprint"] = None
            self._state["navigation"] = {
                "kind": None,
                "status": "idle",
                "feedback": {},
                "goal": None,
                "route": [],
            }
            self._grids.clear()
            self._state["active_runtime"] = {
                "profile_id": profile_id,
                "map_id": map_id,
                "motion": motion,
            }
        self._touch()
        return {"profile_id": profile_id, "map_id": map_id, "motion": motion}

    def stop_runtime(self, _body: dict) -> dict:
        self.cancel_navigation({})
        self.processes.runtime.stop()
        with self._lock:
            runtime = self._state["active_runtime"]
            self._state["active_runtime"] = None
        self._touch()
        return {"runtime": runtime}

    def shutdown_gateway(self) -> None:
        self._stopping = True
        self._touch()
        if self._http_server is not None:
            self._http_server.shutdown()
            self._http_server.server_close()
            if self._http_thread is not None:
                self._http_thread.join(timeout=2.0)
        self.processes.stop_all()


def main(args=None):
    rclpy.init(args=args)
    node = MobileGateway()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        node.start_http()
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown_gateway()
        except KeyboardInterrupt:
            pass
        try:
            executor.shutdown(timeout_sec=2.0)
        except KeyboardInterrupt:
            pass
        try:
            node.destroy_node()
        finally:
            # Humble may already shut the context down while handling SIGINT.
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
