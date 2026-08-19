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
from collections import defaultdict
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
)
from scout_msgs.msg import ScoutStatus
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
MONITORED_TOPICS = (
    "/lidar/points",
    "/imu/data",
    "/camera/rgb/image_raw",
    "/rtk/fix",
    "/fastlivo_rtk/odometry",
    "/scout_status",
)


class ApiError(RuntimeError):
    def __init__(self, status: int, message: str):
        super().__init__(message)
        self.status = status


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
            "default-src 'self'; connect-src 'self' {}; img-src 'self' data:; "
            "style-src 'self' 'unsafe-inline'; script-src 'self'".format(
                self.gateway.semantic_service_url
            ),
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
                self._json(HTTPStatus.OK, {"maps": self.gateway.available_maps()})
                return
            if parsed.path == "/api/v1/semantic/config":
                self._json(
                    HTTPStatus.OK,
                    self.gateway.semantic_public_config(),
                )
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
        except (
            CatalogError,
            ProfileError,
            ProcessError,
        ) as error:
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
        except (
            CatalogError,
            ProfileError,
            ProcessError,
        ) as error:
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
        self.semantic_service_url = str(
            self.declare_parameter(
                "semantic_service_url", "http://172.18.80.26:8090"
            ).value
        ).rstrip("/")
        if not self.semantic_service_url.startswith(("http://", "https://")):
            raise ValueError("semantic_service_url必须是HTTP(S)地址")
        self.semantic_map_ids = set(
            str(value)
            for value in self.declare_parameter(
                "semantic_map_ids",
                [
                    "map_lio_sam_0811",
                    "map_lio_sam_zoulang_0813_indoor",
                ],
            ).value
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
        self._task_transition_lock = threading.RLock()
        self._revision = 0
        self._stopping = False
        self._http_server = None
        self._http_thread = None
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
                "initialization_stage": None,
                "initialization_source": "none",
                "initialization_status": "未收到",
                "visual_available": None,
                "visual_status": "未收到",
                "manual_required": None,
                "status": "未收到",
                "rtk_initializer_status": "未收到",
                "fix_quality": None,
                "heading_solution": "未收到",
            },
            "chassis": None,
            "command": {"linear": 0.0, "angular": 0.0},
            "topics": {
                topic: {"available": False} for topic in MONITORED_TOPICS
            },
            "navigation": {
                "kind": None,
                "status": "idle",
                "feedback": {},
                "goal": None,
                "route": [],
            },
            "semantic": {
                "available": False,
                "map_id": None,
                "status": "idle",
                "instruction": "",
                "route": [],
                "destinations": [],
                "avoid_node_ids": [],
                "execution_allowed": False,
                "statistics": {},
                "model": "",
                "error": "",
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
        self.create_subscription(String, "/localization/initialization_stage", self._value_callback("initialization_stage"), latched)
        self.create_subscription(String, "/localization/initialization_source", self._value_callback("initialization_source"), latched)
        self.create_subscription(String, "/localization/initialization_status", self._value_callback("initialization_status"), latched)
        self.create_subscription(Bool, "/localization/visual_available", self._value_callback("visual_available"), latched)
        self.create_subscription(String, "/localization/visual_status", self._value_callback("visual_status"), latched)
        self.create_subscription(Bool, "/localization/manual_required", self._value_callback("manual_required"), latched)
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
        self.create_timer(1.0, self._refresh_topic_availability)
        # Coalesce navigation callbacks into one mobile state update. Five Hz is
        # responsive enough for the map while leaving localization CPU headroom.
        self.create_timer(0.2, self._touch)

    @property
    def stopping(self) -> bool:
        return self._stopping

    def _touch(self) -> None:
        with self._condition:
            self._revision += 1
            self._condition.notify_all()

    def _refresh_topic_availability(self) -> None:
        topics = {
            topic: {"available": bool(self.get_publishers_info_by_topic(topic))}
            for topic in MONITORED_TOPICS
        }
        with self._lock:
            if topics == self._state["topics"]:
                return
            self._state["topics"] = topics
        self._touch()

    def _value_callback(self, name):
        def callback(message):
            with self._lock:
                self._state["localization"][name] = message.data

        return callback

    def _pose_callback(self, message: Odometry):
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

    def available_maps(self) -> list[dict]:
        maps = self.map_catalog.list()
        for item in maps:
            item["has_semantic"] = item["id"] in self.semantic_map_ids
        return maps

    def semantic_public_config(self) -> dict:
        return {
            "service_url": self.semantic_service_url,
            "map_ids": sorted(self.semantic_map_ids),
            "request_origin": "phone",
            "provider": "alibaba_cloud_bailian",
        }

    def _empty_semantic_state(self, map_id: str | None = None) -> dict:
        return {
            "available": bool(map_id and map_id in self.semantic_map_ids),
            "map_id": map_id,
            "status": "idle",
            "instruction": "",
            "route": [],
            "destinations": [],
            "avoid_node_ids": [],
            "execution_allowed": False,
            "statistics": {},
            "model": "",
            "error": "",
        }

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
                    "localhost_only": os.environ.get("ROS_LOCALHOST_ONLY", "0") == "1",
                },
                **core,
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
            "/api/v1/semantic/route": self.receive_semantic_route,
            "/api/v1/semantic/execute": self.execute_semantic_navigation,
            "/api/v1/semantic/clear": self.clear_semantic_navigation,
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
        with self._lock:
            stage = self._state["localization"].get("initialization_stage")
            manual_required = self._state["localization"].get("manual_required")
        if stage is not None and manual_required is not True:
            raise ApiError(
                HTTPStatus.CONFLICT,
                "自动初始定位仍在执行；只有RTK和视觉均失败后才接受手动位姿",
            )
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
            navigation_status = self._state["navigation"].get("status")
        if ready is not True:
            raise ApiError(HTTPStatus.CONFLICT, "融合定位尚未就绪，禁止下发导航任务")
        if self._goal_handle is not None or navigation_status in (
            "sending",
            "accepted",
            "executing",
            "canceling",
        ):
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
        values = body.get("poses")
        if not isinstance(values, list) or not 2 <= len(values) <= MAX_ROUTE_POSES:
            raise ApiError(HTTPStatus.BAD_REQUEST, "连续路线必须包含2至100个位姿")
        route = [pose_document(value) for value in values]
        return self._send_navigation_route(route, "route")

    def _send_navigation_route(self, route: list[dict], kind: str) -> dict:
        self._assert_navigation_ready()
        if not self._navigate_through_poses.wait_for_server(timeout_sec=0.2):
            raise ApiError(HTTPStatus.CONFLICT, "NavigateThroughPoses服务器尚未就绪")
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._stamped_pose(self, pose) for pose in route]
        with self._lock:
            self._state["navigation"] = {
                "kind": kind,
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

    def receive_semantic_route(self, body: dict) -> dict:
        semantic = body.get("semantic")
        if not isinstance(semantic, dict):
            raise ApiError(HTTPStatus.BAD_REQUEST, "缺少172服务器返回的语义路线")
        with self._lock:
            active_runtime = self._state.get("active_runtime")
        if not isinstance(active_runtime, dict):
            raise ApiError(HTTPStatus.CONFLICT, "请先启动一张导航地图")
        map_id = str(active_runtime.get("map_id", ""))
        if map_id not in self.semantic_map_ids:
            raise ApiError(HTTPStatus.CONFLICT, "当前地图没有对应的语义图谱")
        if semantic.get("map_id") != map_id:
            raise ApiError(HTTPStatus.CONFLICT, "语义路线与当前运行地图不一致")
        if semantic.get("provider") != "alibaba_cloud_bailian":
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义路线不是由阿里百炼生成")
        if semantic.get("model") != "qwen3.7-flash":
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义路线模型版本不受支持")
        graph_digest = semantic.get("graph_sha256")
        if (
            not isinstance(graph_digest, str)
            or len(graph_digest) != 64
            or any(value not in "0123456789abcdef" for value in graph_digest)
        ):
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义图谱摘要无效")
        values = semantic.get("route")
        if not isinstance(values, list) or not 2 <= len(values) <= MAX_ROUTE_POSES:
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义路线必须包含2至100个位姿")
        route = []
        for value in values:
            pose = pose_document(value)
            pose["place_id"] = str(value.get("place_id", ""))
            route.append(pose)
        instruction = semantic.get("instruction")
        if not isinstance(instruction, str) or not instruction.strip() or len(instruction) > 1000:
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义任务描述无效")
        avoid_node_ids = semantic.get("avoid_node_ids", [])
        if (
            not isinstance(avoid_node_ids, list)
            or len(avoid_node_ids) > 16
            or any(not isinstance(value, str) or not value for value in avoid_node_ids)
        ):
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义避让节点无效")
        execution_allowed = semantic.get("execution_allowed")
        if not isinstance(execution_allowed, bool):
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义路线执行标记无效")
        if avoid_node_ids and execution_allowed:
            raise ApiError(HTTPStatus.BAD_REQUEST, "包含避让区的路线不能直接执行")
        destinations = semantic.get("destinations", [])
        if not isinstance(destinations, list) or len(destinations) > 16:
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义目的地无效")
        validated_destinations = []
        for destination in destinations:
            if not isinstance(destination, dict):
                raise ApiError(HTTPStatus.BAD_REQUEST, "语义目的地无效")
            summary = destination.get("semantic_summary", [])
            if (
                not isinstance(summary, list)
                or any(not isinstance(value, str) for value in summary)
            ):
                raise ApiError(HTTPStatus.BAD_REQUEST, "语义目的地描述无效")
            validated_destinations.append(
                {
                    "place_id": str(destination.get("place_id", "")),
                    "name": str(destination.get("name", ""))[:100],
                    "semantic_summary": [value[:100] for value in summary[:5]],
                }
            )
        raw_statistics = semantic.get("statistics", {})
        if not isinstance(raw_statistics, dict):
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义路线统计无效")
        raw_place_count = finite_number(
            raw_statistics.get("route_navigation_places", len(route)),
            "拓扑点数量",
        )
        if not raw_place_count.is_integer() or not 2 <= raw_place_count <= MAX_ROUTE_POSES:
            raise ApiError(HTTPStatus.BAD_REQUEST, "拓扑点数量无效")
        route_length = finite_number(
            raw_statistics.get("drivable_route_length_m", 0.0), "路线长度"
        )
        if route_length < 0.0:
            raise ApiError(HTTPStatus.BAD_REQUEST, "路线长度无效")
        statistics = {
            "route_navigation_places": int(raw_place_count),
            "drivable_route_length_m": route_length,
        }
        self._assert_navigation_ready()
        semantic_state = {
            "available": True,
            "map_id": map_id,
            "status": "ready",
            "instruction": instruction.strip(),
            "route": route,
            "destinations": validated_destinations,
            "avoid_node_ids": avoid_node_ids,
            "execution_allowed": execution_allowed,
            "statistics": statistics,
            "provider": "alibaba_cloud_bailian",
            "model": "qwen3.7-flash",
            "graph_sha256": graph_digest,
            "request_id": str(body.get("request_id", ""))[:64],
            "error": "",
        }
        with self._lock:
            current_runtime = self._state.get("active_runtime")
            if not isinstance(current_runtime, dict) or current_runtime.get("map_id") != map_id:
                raise ApiError(HTTPStatus.CONFLICT, "接收路线期间运行地图已切换")
            self._state["semantic"] = semantic_state
        self._touch()
        return {"semantic": semantic_state}

    def execute_semantic_navigation(self, _body: dict) -> dict:
        with self._lock:
            semantic = json.loads(json.dumps(self._state["semantic"]))
            active_runtime = self._state.get("active_runtime")
        if semantic.get("status") != "ready" or len(semantic.get("route", [])) < 2:
            raise ApiError(HTTPStatus.CONFLICT, "请先生成有效的语义路线")
        if not isinstance(active_runtime, dict) or semantic.get("map_id") != active_runtime.get(
            "map_id"
        ):
            raise ApiError(HTTPStatus.CONFLICT, "语义路线与当前运行地图不一致")
        if semantic.get("execution_allowed") is not True:
            raise ApiError(
                HTTPStatus.CONFLICT,
                "该任务包含语义避让区，配置Nav2 Keepout Filter前禁止真车执行",
            )
        route = [pose_document(value) for value in semantic["route"]]
        result = self._send_navigation_route(route, "semantic")
        return {
            **result,
            "instruction": semantic["instruction"],
            "map_id": semantic["map_id"],
        }

    def clear_semantic_navigation(self, _body: dict) -> dict:
        with self._lock:
            active_runtime = self._state.get("active_runtime")
            map_id = active_runtime.get("map_id") if isinstance(active_runtime, dict) else None
            self._state["semantic"] = self._empty_semantic_state(map_id)
        self._touch()
        return {"map_id": map_id}

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

    def _stop_active_tasks(self) -> list[str]:
        """Fully stop the previous managed task before another one is started."""
        active = self.processes.running_names()
        if "runtime" in active or self._goal_handle is not None:
            self.cancel_navigation({})
        self.processes.stop_all()
        remaining = self.processes.running_names()
        if remaining:
            raise ProcessError("旧任务尚未完全退出，禁止启动新任务")
        with self._lock:
            self._goal_handle = None
            self._state["active_runtime"] = None
            self._state["active_collection"] = None
            self._state["active_processing"] = None
            self._state["semantic"] = self._empty_semantic_state()
            self._state["navigation"] = {
                "kind": None,
                "status": "idle",
                "feedback": {},
                "goal": None,
                "route": [],
            }
        self._touch()
        return active

    def start_collection(self, body: dict) -> dict:
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
            "start_camera:=true",
            "enable_ntrip:=false",
            "record_bag:=true",
            f"bag_output:={bag_path}",
        ]
        with self._task_transition_lock:
            stopped = self._stop_active_tasks()
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
        return {"bag_id": bag_id, "bag_path": str(bag_path), "stopped": stopped}

    def stop_collection(self, _body: dict) -> dict:
        with self._task_transition_lock:
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
        with self._task_transition_lock:
            stopped = self._stop_active_tasks()
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
        return {"map_name": map_name, "stopped": stopped}

    def stop_processing(self, _body: dict) -> dict:
        with self._task_transition_lock:
            self.processes.processing.stop()
            with self._lock:
                processing = self._state["active_processing"]
                self._state["active_processing"] = None
        self._touch()
        return {"processing": processing}

    def start_runtime(self, body: dict) -> dict:
        profile_id = str(body.get("profile_id", ""))
        map_id = str(body.get("map_id", ""))
        motion = bool(body.get("motion", False))
        if motion and body.get("motion_confirmed") is not True:
            raise ApiError(HTTPStatus.BAD_REQUEST, "真车运动必须再次明确确认")
        map_base = self.map_catalog.map_base(map_id)
        command = self.runtime_profiles.command(profile_id, map_base, motion)
        with self._task_transition_lock:
            stopped = self._stop_active_tasks()
            self.processes.runtime.start(
                command,
                self._log_path("runtime", f"{profile_id}_{map_id}"),
                os.environ.copy(),
            )
            with self._lock:
                self._state["pose"] = None
                self._state["paths"] = {"history": [], "global": [], "local": []}
                self._state["footprint"] = None
                self._grids.clear()
                self._state["active_runtime"] = {
                    "profile_id": profile_id,
                    "map_id": map_id,
                    "motion": motion,
                }
                self._state["semantic"] = self._empty_semantic_state(map_id)
        self._touch()
        return {
            "profile_id": profile_id,
            "map_id": map_id,
            "motion": motion,
            "stopped": stopped,
        }

    def stop_runtime(self, _body: dict) -> dict:
        with self._task_transition_lock:
            self.cancel_navigation({})
            self.processes.runtime.stop()
            with self._lock:
                runtime = self._state["active_runtime"]
                self._goal_handle = None
                self._state["active_runtime"] = None
                self._state["semantic"] = self._empty_semantic_state()
                self._state["navigation"] = {
                    "kind": None,
                    "status": "idle",
                    "feedback": {},
                    "goal": None,
                    "route": [],
                }
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
        with self._task_transition_lock:
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
