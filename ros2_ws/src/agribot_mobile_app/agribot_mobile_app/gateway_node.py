#!/usr/bin/env python3
"""ROS 2 node and dependency-free HTTP/SSE server for the mobile PWA."""

from __future__ import annotations

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from array import array
import json
import gzip
import math
import mimetypes
import os
from pathlib import Path
import socket
import shutil
import ssl
import threading
import time
from collections import defaultdict
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlparse

import numpy as np
import yaml

from diagnostic_msgs.msg import DiagnosticArray
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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
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
from std_srvs.srv import SetBool, Trigger

from .catalog import (
    BagCatalog,
    CatalogError,
    GridData,
    MapCatalog,
    quaternion_yaw,
    validated_identifier,
)
from .chassis_feedback import ChassisFeedback
from .processes import ProcessError, ProcessSlots
from .profiles import ProfileError, RuntimeProfiles
from .physical_config import PhysicalConfigError, PhysicalConfigStore
from .vcu_monitor import VcuMonitor
from .saved_path import SavedPathStore, numbered_markers, handover_path_index
from .saved_path_runner import SavedPathRunner
from .navigation_health import NavigationInputHealth
from .route_costmap import (
    RouteCorridorPolicy,
    RouteCostmapError,
    build_route_costmap,
    neutral_route_costmap,
    world_to_grid,
)


MAX_JSON_BODY = 256 * 1024
MAX_ROUTE_POSES = 110
MAX_ROUTE_CENTERLINE_POINTS = 10000
MONITORED_TOPICS = (
    "/lidar/points",
    "/imu/data",
    "/camera/rgb/image_raw",
    "/rtk/fix",
    "/fastlivo_rtk/odometry",
    "/scout_status",
)
TIME_SYNC_SENSORS = ("lidar", "imu", "camera", "rtk")
TIME_SYNC_PAIRS = ("lidar_imu", "lidar_camera", "lidar_rtk")
TIME_SYNC_CLOCKS = {
    "hikrobot_mvs/time_sync": "camera",
    "hipnuc_imu/time_sync": "imu",
    "rtk_nmea/time_sync": "rtk",
}


def empty_diagnostic(message: str = "未收到授时诊断") -> dict:
    return {
        "level": 3,
        "message": message,
        "hardware_id": "",
        "values": {},
        "updated_at": None,
    }


def empty_time_sync_state(message: str = "未收到授时诊断") -> dict:
    return {
        "summary": empty_diagnostic(message),
        "sensors": {
            name: empty_diagnostic(message) for name in TIME_SYNC_SENSORS
        },
        "pairs": {name: empty_diagnostic(message) for name in TIME_SYNC_PAIRS},
        "clocks": {
            name: empty_diagnostic(message) for name in TIME_SYNC_CLOCKS.values()
        },
    }


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

    def __init__(self, address, gateway, static_root, tls_context=None):
        self.gateway = gateway
        self.static_root = static_root
        super().__init__(address, GatewayRequestHandler)
        if tls_context is not None:
            # Handshake lazily in the request thread so one stalled client
            # cannot block accept() for everyone else.
            self.socket = tls_context.wrap_socket(
                self.socket, server_side=True, do_handshake_on_connect=False
            )

    def handle_error(self, request, client_address):
        # Browsers abort the TLS handshake until the self-signed certificate is
        # trusted; that is expected and must not flood the log with tracebacks.
        self.gateway.get_logger().debug(f"HTTP连接异常: {client_address}")


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
        compressed = False
        for item in self.headers.get("Accept-Encoding", "").split(","):
            name, *parameters = item.lower().strip().split(";")
            if name.strip() != "gzip":
                continue
            quality = dict(part.strip().partition("=")[::2] for part in parameters).get("q", "1")
            try:
                compressed = 0 < float(quality) <= 1 and len(payload) >= 65536
            except ValueError:
                pass
            break
        if compressed:
            payload = gzip.compress(payload, compresslevel=1, mtime=0)
        self.send_response(status)
        self._headers("application/json; charset=utf-8", len(payload))
        self.send_header("Vary", "Accept-Encoding")
        if compressed:
            self.send_header("Content-Encoding", "gzip")
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
                        "vehicles": self.gateway.runtime_profiles.public_vehicles(),
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
            if parsed.path.startswith(prefix) and parsed.path.endswith("/saved-paths"):
                map_id = unquote(parsed.path[len(prefix):-len("/saved-paths")]).strip("/")
                self._json(HTTPStatus.OK, self.gateway.saved_paths.public(map_id))
                return
            if parsed.path.startswith(prefix) and parsed.path.endswith("/routes"):
                map_id = unquote(parsed.path[len(prefix):-len("/routes")]).strip("/")
                self._json(HTTPStatus.OK, self.gateway.map_catalog.route_presets(map_id))
                return
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
            PhysicalConfigError,
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
            PhysicalConfigError,
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
            # Never cache the SPA HTML under a missing script or stylesheet URL.
            if relative_path.suffix or relative_path.parts[0] in ("assets", "icons", "downloads", "fieldgate"):
                raise ApiError(HTTPStatus.NOT_FOUND, "文件不存在")
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
        # HTTPS gives desktop browsers a secure context for Web Serial (6750 UART0
        # on the operator's computer). Plain HTTP stays for the Android app.
        self.https_port = int(self.declare_parameter("https_port", 8443).value)
        tls_root = Path.home() / ".agribot_mobile" / "tls"
        self.tls_cert_file = Path(str(self.declare_parameter(
            "tls_cert_file", str(tls_root / "cert.pem")).value)).expanduser()
        self.tls_key_file = Path(str(self.declare_parameter(
            "tls_key_file", str(tls_root / "key.pem")).value)).expanduser()
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
        self.trajectory_from_pose = bool(self.declare_parameter("trajectory_from_pose", True).value)
        self.footprint_topic = str(
            self.declare_parameter(
                "footprint_topic", "/local_costmap/published_footprint"
            ).value
        )
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
        self.semantic_proximity_costmap_topic = str(
            self.declare_parameter(
                "semantic_proximity_costmap_topic",
                "/semantic_navigation/proximity_costmap",
            ).value
        )
        self.map_catalog = MapCatalog(self.map_root)
        self.bag_catalog = BagCatalog(self.bag_root)
        self.runtime_profiles = RuntimeProfiles(profile_path)
        self.physical_config = PhysicalConfigStore(Path(str(self.declare_parameter(
            "physical_config_root",
            str(Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state")))
                / "agribot-mobile-app/physical-config"),
        ).value)))
        default_vehicle = self.runtime_profiles.public_vehicles()[0]
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._task_transition_lock = threading.RLock()
        self._headlight_lock = threading.Lock()
        self._revision = 0
        self._stopping = False
        self._http_server = None
        self._http_thread = None
        self._https_server = None
        self._https_thread = None
        self._snapshot_lock = threading.Lock()
        self._snapshot_cache_revision = -1
        self._snapshot_cache = None
        self._event_cache_revision = -1
        self._event_cache = None
        self._grids: dict[str, GridData] = {}
        self._grid_revisions = defaultdict(int)
        self._navigation_health = NavigationInputHealth()
        self._pose_callbacks = MutuallyExclusiveCallbackGroup()
        self._chassis_callbacks = MutuallyExclusiveCallbackGroup()
        self._navigation_callbacks = MutuallyExclusiveCallbackGroup()
        self._status_callbacks = MutuallyExclusiveCallbackGroup()
        self._housekeeping_callbacks = MutuallyExclusiveCallbackGroup()
        self._grid_callbacks = {layer: MutuallyExclusiveCallbackGroup() for layer in self.grid_topics}
        self._semantic_costmap_source: dict | None = None
        self._chassis_feedback = ChassisFeedback()
        self._state = {
            "pose": None,
            "paths": {"history": [], "global": [], "local": []},
            "footprint": None,
            "vehicle": default_vehicle,
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
            "time_sync": empty_time_sync_state(),
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
                "destination_poses": [],
                "destinations": [],
                "avoid_node_ids": [],
                "avoidance_zones": [],
                "execution_allowed": False,
                "costmap_ready": False,
                "statistics": {},
                "model": "",
                "error": "",
            },
            "active_runtime": None,
            "active_collection": None,
            "active_processing": None,
        }
        self.processes = ProcessSlots(self._touch)
        self.vcu_monitor = VcuMonitor(str(self.declare_parameter("vcu_diagnostic_port", "").value), self._touch)
        self._vehicle_installations = {}
        hardware_share = Path(get_package_share_directory("agribot_hardware_bringup"))
        for vehicle, relative in (("ackermann", "config/sensor_mounts.yaml"),
                                  ("differential", "differential/config/sensor_mounts.yaml")):
            try:
                mounts = yaml.safe_load((hardware_share / relative).read_text(encoding="utf-8"))
                self._vehicle_installations[vehicle] = {"parent_frame": "base_link", "mounts": mounts}
            except (OSError, ValueError, yaml.YAMLError):
                self._vehicle_installations[vehicle] = None
        self._goal_handle = None
        self._path_updates: dict[str, float] = {}
        self._history_received_at = 0.0

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._semantic_mask_publisher = self.create_publisher(
            OccupancyGrid, self.semantic_proximity_costmap_topic, latched
        )
        normal = QoSProfile(depth=20)
        latest = QoSProfile(depth=1)
        self.create_subscription(Odometry, self.pose_topic, self._pose_callback, latest,
                                 callback_group=self._pose_callbacks)
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
        if not self.trajectory_from_pose:
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
                OccupancyGrid, topic, self._grid_callback(layer, topic), latched,
                callback_group=self._grid_callbacks[layer],
            )
        self.create_subscription(Bool, "/localization/ready", self._value_callback("ready"), latched, callback_group=self._status_callbacks)
        self.create_subscription(Bool, "/localization/lidar_ready", self._value_callback("lidar_ready"), latched, callback_group=self._status_callbacks)
        self.create_subscription(Bool, "/fastlivo_rtk/ready", self._value_callback("fusion_ready"), latched, callback_group=self._status_callbacks)
        self.create_subscription(Bool, "/fastlivo_rtk/fixed_active", self._value_callback("fixed_active"), latched, callback_group=self._status_callbacks)
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
        self.create_subscription(ScoutStatus, "/scout_status", self._chassis_callback, latest,
                                 callback_group=self._chassis_callbacks)
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self._diagnostics_callback, normal,
            callback_group=self._status_callbacks,
        )
        self.create_subscription(Twist, "/nav2/cmd_vel", self._command_callback, latest,
                                 callback_group=self._navigation_callbacks)
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
        self._navigate_to_pose = ActionClient(self, NavigateToPose, "navigate_to_pose",
                                             callback_group=self._navigation_callbacks)
        self._initialization_start_client = self.create_client(Trigger, "/localization/start")
        self._headlight_client = self.create_client(SetBool, "/hardware/set_headlight",
                                                   callback_group=self._chassis_callbacks)
        self._handover_client = self.create_client(Trigger, "/localization/switch_to_fastlivo",
                                                   callback_group=self._navigation_callbacks)
        self.create_subscription(String, "/localization/active_method",
                                 self._value_callback("active_method"), latched,
                                 callback_group=self._status_callbacks)
        config_path = Path(get_package_share_directory("agribot_hardware_bringup")) / "differential/config/nav2_params_differential_fastlivo_mapped.yaml"
        self.saved_paths = SavedPathStore(self.map_catalog, config_path)
        footprint = json.loads(yaml.safe_load(config_path.read_text())["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"])
        self._saved_plan_publisher = self.create_publisher(NavPath, self.global_plan_topic, 1)
        self._saved_runner = SavedPathRunner(self, self._saved_observe, self._saved_update,
                                            self._saved_publish, footprint,
                                            yaml.safe_load(config_path.read_text())["controller_server"]["ros__parameters"]["goal_checker"]["xy_goal_tolerance"],
                                            callback_group=self._navigation_callbacks,
                                            recovery_ready=self._saved_recovery_ready)
        self._navigate_through_poses = ActionClient(
            self, NavigateThroughPoses, "navigate_through_poses",
            callback_group=self._navigation_callbacks,
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
        self.create_timer(2.0, self._refresh_topic_availability,
                          callback_group=self._housekeeping_callbacks)
        # Coalesce navigation callbacks into one mobile state update. Five Hz is
        # responsive enough for the map while leaving localization CPU headroom.
        self.create_timer(0.2, self._touch, callback_group=self._status_callbacks)

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
        changed = False
        with self._lock:
            if topics != self._state["topics"]:
                self._state["topics"] = topics
                changed = True
            now = time.time()
            time_sync = self._state["time_sync"]
            sections = [(time_sync, "summary")]
            sections.extend(
                (time_sync[group], name)
                for group in ("sensors", "pairs", "clocks")
                for name in time_sync[group]
            )
            for section, name in sections:
                entry = section[name]
                updated_at = entry.get("updated_at")
                if (
                    updated_at is not None
                    and now - updated_at > 3.0
                    and entry.get("level") != 3
                ):
                    section[name] = empty_diagnostic("授时诊断已停止")
                    changed = True
        if changed:
            self._touch()

    @staticmethod
    def _diagnostic_document(status) -> dict:
        raw_level = status.level
        level = (
            raw_level[0]
            if isinstance(raw_level, (bytes, bytearray)) and raw_level
            else int(raw_level)
        )
        return {
            "level": level,
            "message": status.message,
            "hardware_id": status.hardware_id,
            "values": {value.key: value.value for value in status.values},
            "updated_at": time.time(),
        }

    def _diagnostics_callback(self, message: DiagnosticArray) -> None:
        changed = False
        with self._lock:
            state = self._state["time_sync"]
            for status in message.status:
                if status.name.startswith(("agribot/chassis_can/", "agribot/chassis_serial/")):
                    self._chassis_feedback.update_diagnostic(status)
                    changed = True
                    continue
                if status.name == "sensor_time_sync/summary":
                    state["summary"] = self._diagnostic_document(status)
                    changed = True
                    continue
                if status.name.startswith("sensor_time_sync/"):
                    name = status.name.removeprefix("sensor_time_sync/")
                    if name in TIME_SYNC_SENSORS:
                        state["sensors"][name] = self._diagnostic_document(status)
                        changed = True
                    elif name in TIME_SYNC_PAIRS:
                        state["pairs"][name] = self._diagnostic_document(status)
                        changed = True
                    continue
                clock_name = TIME_SYNC_CLOCKS.get(status.name)
                if clock_name is not None:
                    state["clocks"][clock_name] = self._diagnostic_document(status)
                    changed = True
        if changed:
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
            self._navigation_health.record("pose", message.header.frame_id,
                                           self._message_age(message))
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
            if self.trajectory_from_pose:
                self._append_display_history(pose.position.x, pose.position.y)

    def _append_display_history(self, x, y):
        # Derive the phone-only trace from the already received odometry. Taking
        # a growing ROS Path allocates thousands of Python poses before callback
        # throttling can run, starving small critical subscriptions on the X5.
        now = time.monotonic()
        if now - self._history_received_at < 0.5:
            return
        points = self._state["paths"]["history"]
        if points and math.hypot(x-points[-1][0], y-points[-1][1]) < 0.02:
            return
        self._history_received_at = now
        if len(points) >= 600:
            points = points[::2]
        self._state["paths"]["history"] = points + [[x, y]]

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

    def _message_age(self, message):
        stamp = message.header.stamp
        return self.get_clock().now().nanoseconds * 1e-9 - (stamp.sec + stamp.nanosec * 1e-9)

    def _grid_callback(self, layer: str, topic: str):
        def callback(message: OccupancyGrid):
            received_at = time.monotonic()
            source_age = self._message_age(message) if layer == "local_costmap" else None
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
                if layer == "local_costmap":
                    self._navigation_health.record(layer, message.header.frame_id,
                                                   source_age, now=received_at)
                semantic = (
                    self._semantic_costmap_source
                    if layer == "map"
                    and self._semantic_costmap_source is not None
                    else None
                )
                active_costmap_source = (
                    self._semantic_costmap_source
                    if layer == "global_costmap"
                    and self._semantic_costmap_source is not None
                    and self._state["semantic"].get("status") == "ready"
                    and self._state["semantic"].get("costmap_ready") is not True
                    else None
                )
            if layer == "map":
                try:
                    self._publish_semantic_costmap(grid, semantic)
                except RouteCostmapError as error:
                    self.get_logger().error(f"无法更新语义避让代价层: {error}")
            if (
                layer == "global_costmap"
                and active_costmap_source is not None
                and self._semantic_costs_applied(
                    grid, active_costmap_source["verification_points"]
                )
            ):
                with self._lock:
                    if (
                        self._semantic_costmap_source is active_costmap_source
                        and self._state["semantic"].get("status") == "ready"
                    ):
                        self._state["semantic"]["costmap_ready"] = True
                self.get_logger().info("A*宽走廊和语义避让代价已写入全局代价地图")
                self._touch()

        return callback

    def _costmap_message(self, grid: GridData, mask) -> OccupancyGrid:
        message = OccupancyGrid()
        stamp = self.get_clock().now().to_msg()
        message.header.stamp = stamp
        message.header.frame_id = "map"
        message.info.map_load_time = stamp
        message.info.resolution = grid.resolution
        message.info.width = grid.width
        message.info.height = grid.height
        message.info.origin.position.x = grid.origin_x
        message.info.origin.position.y = grid.origin_y
        message.info.origin.orientation.z = math.sin(0.5 * grid.origin_yaw)
        message.info.origin.orientation.w = math.cos(0.5 * grid.origin_yaw)
        message.data = array("b", mask.reshape(-1).tobytes())
        return message

    def _publish_semantic_costmap(
        self, grid: GridData, semantic: dict | None
    ):
        if semantic is None:
            mask = neutral_route_costmap(grid)
        else:
            mask = build_route_costmap(
                grid,
                semantic["route_centerline"],
                semantic["avoidance_zones"],
                semantic["corridor_policy"],
            )
        self._semantic_mask_publisher.publish(self._costmap_message(grid, mask))
        return mask

    def _publish_neutral_semantic_costmap(self) -> None:
        with self._lock:
            grid = self._grids.get("map")
        if grid is not None:
            self._publish_semantic_costmap(grid, None)

    @staticmethod
    def _semantic_costs_applied(grid: GridData, points: list[dict]) -> bool:
        if not points:
            return False
        for point in points:
            column, row = world_to_grid(grid, point["x"], point["y"])
            if not 0 <= column < grid.width or not 0 <= row < grid.height:
                return False
            if grid.data[row * grid.width + column] < point["minimum_cost"]:
                return False
        return True

    def _chassis_callback(self, message: ScoutStatus):
        self._chassis_feedback.update_status(message)

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
            "destination_poses": [],
            "destinations": [],
            "route": [],
            "route_centerline": [],
            "avoid_node_ids": [],
            "avoidance_zones": [],
            "execution_allowed": False,
            "costmap_ready": False,
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
                core["navigation_inputs"] = self._navigation_health.snapshot()
            document = {
                "revision": revision,
                "server_time": time.time(),
                "ros": {
                    "node": self.get_name(),
                    "domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
                    "localhost_only": os.environ.get("ROS_LOCALHOST_ONLY", "0") == "1",
                },
                **core,
                **self._chassis_feedback.snapshot(core["vehicle"]["id"]),
                "grids": self._grid_metadata(),
                "processes": self.processes.snapshot(),
                "physical_config": self.physical_config.public(),
                "storage": self._disk_state(),
                "vcu": self.vcu_monitor.snapshot(),
                "console": {
                    "https_port": self.https_port if self._https_server is not None else None,
                },
                "vehicle_installation": self._vehicle_installations.get(core["vehicle"]["id"]),
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
        self.vcu_monitor.start()
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
        self._start_https()
        if not self.api_token:
            self.get_logger().warning("未配置api_token，仅应在受控局域网内使用")

    def _start_https(self) -> None:
        if self.https_port <= 0:
            return
        if not (self.tls_cert_file.is_file() and self.tls_key_file.is_file()):
            self.get_logger().warning(
                f"未找到HTTPS证书 {self.tls_cert_file}，电脑端6750诊断串口不可用"
            )
            return
        try:
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            context.minimum_version = ssl.TLSVersion.TLSv1_2
            context.load_cert_chain(str(self.tls_cert_file), str(self.tls_key_file))
            self._https_server = GatewayHttpServer(
                (self.http_host, self.https_port), self, self.static_root, context
            )
        except (OSError, ssl.SSLError) as error:
            self._https_server = None
            self.get_logger().error(f"HTTPS启动失败: {error}")
            return
        self._https_thread = threading.Thread(
            target=self._https_server.serve_forever,
            name="agribot-mobile-https",
            daemon=True,
        )
        self._https_thread.start()
        self._touch()
        self.get_logger().info(
            f"电脑端HTTPS已启动: https://{self.http_host}:{self.https_port}"
        )

    def handle_command(self, path: str, body: dict) -> dict | None:
        handlers = {
            "/api/v1/chassis/headlight": self.set_headlight,
            "/api/v1/localization/initial-pose": self.publish_initial_pose,
            "/api/v1/localization/start": self.start_initialization,
            "/api/v1/navigation/goal": self.send_navigation_goal,
            "/api/v1/navigation/route": self.send_navigation_route,
            "/api/v1/navigation/saved-path/preview": self.preview_saved_path,
            "/api/v1/navigation/saved-path/execute": self.execute_saved_path,
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
            "/api/v1/runtime/config/import": self.import_physical_config,
            "/api/v1/runtime/config/reset": self.reset_physical_config,
        }
        try:
            handler = handlers[path]
        except KeyError as error:
            raise ApiError(HTTPStatus.NOT_FOUND, "未知控制接口") from error
        return handler(body)

    def set_headlight(self, body: dict) -> dict:
        enabled = body.get("enabled")
        if type(enabled) is not bool:
            raise ApiError(HTTPStatus.BAD_REQUEST, "enabled必须为布尔值")
        with self._headlight_lock:
            with self._lock:
                vehicle = self._state["vehicle"]["id"]
            if body.get("vehicle_type") != vehicle:
                raise ApiError(HTTPStatus.CONFLICT, "灯光请求车型与当前底盘不一致")
            snapshot = self._chassis_feedback.snapshot(vehicle)
            feedback = snapshot["chassis_feedback"]
            if not feedback["fresh"]:
                raise ApiError(HTTPStatus.CONFLICT, "未收到有效底盘反馈，不能控制灯光")
            if not feedback["headlight_supported"]:
                raise ApiError(HTTPStatus.CONFLICT, "当前底盘协议不支持灯光控制")
            if feedback["read_only"]:
                raise ApiError(HTTPStatus.CONFLICT, "观察阶段CAN只读，请先启用底盘运行模式")
            client = self._headlight_client
            if not client.wait_for_service(timeout_sec=0.3):
                raise ApiError(HTTPStatus.SERVICE_UNAVAILABLE, "底盘灯光服务未就绪，请更新底盘程序")
            request = SetBool.Request()
            request.data = enabled
            sent_at = time.monotonic()
            future = client.call_async(request)
            finished = threading.Event()
            future.add_done_callback(lambda _: finished.set())
            if not finished.wait(2.0):
                raise ApiError(HTTPStatus.GATEWAY_TIMEOUT, "灯光应答超时，请检查实际灯光状态")
            result = future.result()
            if not result or not result.success:
                raise ApiError(HTTPStatus.CONFLICT, result.message if result else "灯光请求失败")
            confirmed = self._chassis_feedback.wait_for_headlight(vehicle, enabled, sent_at, timeout=0.3)
            self._touch()
            return {"headlight": enabled, "confirmed": confirmed,
                    "message": "开灯指令已发送" if enabled else "关灯指令已发送"}

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

    def start_initialization(self, _body: dict) -> dict:
        with self._task_transition_lock:
            with self._lock:
                runtime = self._state.get("active_runtime")
                ready = self._state["localization"].get("fusion_ready")
                nav = self._state["navigation"].get("status")
                stage = self._state["localization"].get("initialization_stage")
            if not runtime or not runtime.get("initialization_wait_for_request"):
                raise ApiError(HTTPStatus.CONFLICT, "当前运行栈不是按钮初始化模式，请停止后重新选择启动")
            if ready or nav in ("sending", "accepted", "executing", "canceling"):
                raise ApiError(HTTPStatus.CONFLICT, "定位已就绪或导航正在运行，禁止重新初始化")
            if stage not in ("wait_request", "manual_required"):
                raise ApiError(HTTPStatus.CONFLICT, "初始化正在进行或节点尚未就绪")
            client = self._initialization_start_client
            if not client.wait_for_service(timeout_sec=0.3):
                raise ApiError(HTTPStatus.CONFLICT, "初始化服务尚未就绪")
            future = client.call_async(Trigger.Request())
            finished = threading.Event()
            future.add_done_callback(lambda _: finished.set())
            if not finished.wait(3.0):
                raise ApiError(HTTPStatus.GATEWAY_TIMEOUT, "初始化应答超时，请查看初始化状态，勿重复点击")
            result = future.result()
            if not result or not result.success:
                raise ApiError(HTTPStatus.CONFLICT, result.message if result else "初始化请求失败")
            return {"status": "requested", "message": result.message}

    def _assert_navigation_ready(self):
        with self._lock:
            ready = self._state["localization"].get("fusion_ready")
            navigation_status = self._state["navigation"].get("status")
        if ready is not True:
            raise ApiError(HTTPStatus.CONFLICT, "融合定位尚未就绪，禁止下发导航任务")
        if (getattr(self, "_saved_runner", None) and self._saved_runner.active) or self._goal_handle is not None or navigation_status in (
            "sending",
            "accepted",
            "executing",
            "canceling",
        ):
            raise ApiError(HTTPStatus.CONFLICT, "已有导航任务正在执行")

    def _clear_semantic_for_manual_navigation(self) -> None:
        with self._lock:
            active_runtime = self._state.get("active_runtime")
            map_id = (
                active_runtime.get("map_id")
                if isinstance(active_runtime, dict)
                else None
            )
            self._state["semantic"] = self._empty_semantic_state(map_id)
            self._semantic_costmap_source = None
        self._publish_neutral_semantic_costmap()

    def send_navigation_goal(self, body: dict) -> dict:
        self._assert_navigation_ready()
        if not self._navigate_to_pose.wait_for_server(timeout_sec=0.2):
            raise ApiError(HTTPStatus.CONFLICT, "NavigateToPose服务器尚未就绪")
        self._clear_semantic_for_manual_navigation()
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
            raise ApiError(HTTPStatus.BAD_REQUEST, "连续路线必须包含2至110个位姿")
        route = [pose_document(value) for value in values]
        return self._send_navigation_route(route, "route")

    def _saved_selection(self, body):
        map_id = str(body.get("map_id", ""))
        route, path = self.saved_paths.select(map_id, str(body.get("route_id", "")), body.get("start_point"))
        if route["vehicle_type"] != "differential":
            raise ApiError(HTTPStatus.CONFLICT, "保存路径模式目前仅验证差速车")
        return map_id, route, path

    def preview_saved_path(self, body):
        if (self._saved_runner.active or self._goal_handle is not None
                or self._state["navigation"].get("status") in ("sending", "executing", "canceling")):
            raise ApiError(HTTPStatus.CONFLICT, "导航期间不能切换预览路径")
        map_id, route, path = self._saved_selection(body)
        preview = {"map_id": map_id, "route_id": route["id"],
                   "start_point": body["start_point"],
                   "markers": numbered_markers(route, body["start_point"])}
        with self._lock:
            self._state["saved_path_preview"] = preview
        self._saved_publish(path)
        return {**preview, "path_points": len(path)}

    def execute_saved_path(self, body):
        with self._task_transition_lock:
            self._assert_navigation_ready()
            map_id, route, path = self._saved_selection(body)
            runtime = self._state.get("active_runtime") or {}
            if runtime.get("map_id") != map_id or runtime.get("vehicle_type") != "differential":
                raise ApiError(HTTPStatus.CONFLICT, "运行地图或车型与保存路径不匹配")
            try:
                self._saved_observe()
            except RuntimeError as error:
                raise ApiError(HTTPStatus.CONFLICT, str(error)) from error
            self._clear_semantic_for_manual_navigation()
            with self._lock:
                self._state["saved_path_preview"] = {
                    "map_id": map_id, "route_id": route["id"], "start_point": body["start_point"],
                    "markers": numbered_markers(route, body["start_point"])}
                self._state["navigation"] = {"kind": "saved_path", "status": "sending",
                                             "feedback": {}, "goal": path[-1], "route": []}
            try:
                callback = None
                if runtime.get("localization_strategy") == "navsat_waypoint100":
                    callback = self._handover_progress_callback(route, body["start_point"])
                self._saved_runner.start(path, on_progress=callback)
            except RuntimeError as error:
                self._saved_update("failed", str(error))
                raise ApiError(HTTPStatus.CONFLICT, str(error)) from error
        return {"route_id": route["id"], "start_point": body["start_point"]}

    def _handover_progress_callback(self, route, start):
        threshold = handover_path_index(route, start)
        waypoint = route.get("localization_handover_waypoint", 100)
        task = self._state["navigation"]
        lock = threading.RLock()
        request = {"future": None, "done": False, "last_try": -math.inf}

        def current():
            return (self._state["navigation"] is task and not self._saved_runner.stop_event.is_set())

        def completed(future):
            with lock:
                if request["future"] is not future:
                    return
                request["future"] = None
                try:
                    result = future.result()
                    request["done"] = result is not None and result.success
                except Exception:
                    request["done"] = False
                with self._lock:
                    if current() and request["done"]:
                        self._saved_update("executing", f"已通过第{waypoint}点，KF-GINS继续定位，FAST-LIVO2＋RTK预热中")

        def progress(index):
            if threshold is None or index < threshold:
                return
            with lock:
                if request["done"] or not current():
                    return
                now = time.monotonic()
                future = request["future"]
                if future is not None:
                    if now-request["last_try"] <= 5.0:
                        return
                    request["future"] = None
                    future.cancel()
                if now-request["last_try"] < 1.0:
                    return
                request["last_try"] = now
                if not self._handover_client.service_is_ready():
                    self.get_logger().warning(f"第{waypoint}点定位切换服务未就绪，保持当前定位并等待服务")
                    return
                request["future"] = self._handover_client.call_async(Trigger.Request())
                request["future"].add_done_callback(completed)

        return progress

    def _saved_observe(self):
        with self._lock:
            self._navigation_health.require(self._state["localization"].get("fusion_ready") is True)
            pose, grid = self._state["pose"], self._grids.get("local_costmap")
            if pose is None or grid is None:
                raise RuntimeError("定位或实时局部代价地图尚未收到，停止保存路径任务")
            return dict(pose), grid

    def _saved_recovery_ready(self):
        with self._lock:
            # Freshness is required to resume a paused task, not used to abort
            # a running task. A latched true flag alone cannot prove recovery.
            return (self._state["localization"].get("fusion_ready") is True
                    and all(v["ready"] for v in self._navigation_health.snapshot().values()))

    def _saved_update(self, status, message):
        with self._lock:
            self._state["navigation"]["status"] = status
            self._state["navigation"]["feedback"] = {"message": message}
        if status == "failed":
            self.get_logger().error(f"保存路径任务失败：{message}")
        self._touch()

    def _saved_publish(self, path):
        step = max(1, math.ceil(len(path) / 1200))
        display = path[::step]
        if display[-1] is not path[-1]:
            display.append(path[-1])
        with self._lock:
            self._state["paths"]["global"] = [[p["x"], p["y"]] for p in display]
        # /plan is display-only here. FollowPath still receives the complete
        # original path, not this bounded visualization copy.
        self._saved_plan_publisher.publish(self._saved_runner.message(display))
        self._touch()

    def _send_navigation_route(self, route: list[dict], kind: str) -> dict:
        self._assert_navigation_ready()
        if not self._navigate_through_poses.wait_for_server(timeout_sec=0.2):
            raise ApiError(HTTPStatus.CONFLICT, "NavigateThroughPoses服务器尚未就绪")
        if kind != "semantic":
            self._clear_semantic_for_manual_navigation()
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
        raw_route = semantic.get("route")
        if (
            not isinstance(raw_route, list)
            or not 2 <= len(raw_route) <= MAX_ROUTE_POSES
        ):
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义A*路线位姿数量无效")
        route = []
        for value in raw_route:
            pose = pose_document(value)
            pose["place_id"] = str(value.get("place_id", ""))
            if not pose["place_id"]:
                raise ApiError(HTTPStatus.BAD_REQUEST, "语义A*路线地点编号无效")
            route.append(pose)
        raw_centerline = semantic.get("route_centerline")
        if (
            not isinstance(raw_centerline, list)
            or not 2 <= len(raw_centerline) <= MAX_ROUTE_CENTERLINE_POINTS
        ):
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义A*中心线点数无效")
        route_centerline = []
        for point in raw_centerline:
            if not isinstance(point, dict):
                raise ApiError(HTTPStatus.BAD_REQUEST, "语义A*中心线点无效")
            route_centerline.append(
                {
                    "x": finite_number(point.get("x"), "语义A*中心线X坐标"),
                    "y": finite_number(point.get("y"), "语义A*中心线Y坐标"),
                }
            )
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
        if execution_allowed is not True:
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义路线执行标记无效")
        raw_destination_poses = semantic.get("destination_poses")
        if (
            not isinstance(raw_destination_poses, list)
            or not 1 <= len(raw_destination_poses) <= 16
        ):
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义目的地位姿无效")
        destination_poses = []
        for value in raw_destination_poses:
            pose = pose_document(value)
            pose["place_id"] = str(value.get("place_id", ""))
            if not pose["place_id"]:
                raise ApiError(HTTPStatus.BAD_REQUEST, "语义目的地编号无效")
            destination_poses.append(pose)
        destinations = semantic.get("destinations", [])
        if (
            not isinstance(destinations, list)
            or len(destinations) != len(destination_poses)
        ):
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
        if [item["place_id"] for item in validated_destinations] != [
            item["place_id"] for item in destination_poses
        ]:
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义目的地顺序不一致")
        raw_zones = semantic.get("avoidance_zones", [])
        if not isinstance(raw_zones, list) or len(raw_zones) > 16:
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义避让区域无效")
        avoidance_zones = []
        for zone in raw_zones:
            if not isinstance(zone, dict):
                raise ApiError(HTTPStatus.BAD_REQUEST, "语义避让区域无效")
            selector = str(zone.get("selector", ""))
            influence_radius = finite_number(
                zone.get("influence_radius_m"), "语义避让影响半径"
            )
            decay_length = finite_number(
                zone.get("decay_length_m"), "语义避让衰减距离"
            )
            if not selector or influence_radius <= 0.0 or decay_length <= 0.0:
                raise ApiError(HTTPStatus.BAD_REQUEST, "语义避让区域无效")
            avoidance_zones.append(
                {
                    "selector": selector,
                    "x": finite_number(zone.get("x"), "语义避让X坐标"),
                    "y": finite_number(zone.get("y"), "语义避让Y坐标"),
                    "influence_radius_m": influence_radius,
                    "decay_length_m": decay_length,
                }
            )
        if sorted(item["selector"] for item in avoidance_zones) != sorted(
            avoid_node_ids
        ):
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义避让区域与避让节点不一致")
        raw_costmap_policy = semantic.get("costmap_policy")
        if not isinstance(raw_costmap_policy, dict):
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义禁行区策略无效")
        half_width = finite_number(
            raw_costmap_policy.get("route_corridor_half_width_m"),
            "语义路线走廊半宽",
        )
        transition_width = finite_number(
            raw_costmap_policy.get("route_corridor_transition_width_m"),
            "语义路线走廊过渡宽度",
        )
        outside_cost = raw_costmap_policy.get("route_corridor_outside_cost")
        if (
            raw_costmap_policy.get("semantic_route_preference_enabled") is not True
            or raw_costmap_policy.get("route_corridor_model")
            != "wide_free_core_additive_outside"
            or half_width <= 0.0
            or transition_width <= 0.0
            or isinstance(outside_cost, bool)
            or not isinstance(outside_cost, int)
            or not 1 <= outside_cost <= 100
            or raw_costmap_policy.get("semantic_avoidance_is_lethal") is not False
            or raw_costmap_policy.get("semantic_proximity_cost_model")
            != "exponential_additive"
            or raw_costmap_policy.get("requires_nav2_proximity_layer") is not True
        ):
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义路线走廊策略无效")
        corridor_policy = RouteCorridorPolicy(
            half_width_m=half_width,
            transition_width_m=transition_width,
            outside_cost=outside_cost,
        )
        corridor_policy.validate()
        raw_statistics = semantic.get("statistics", {})
        if not isinstance(raw_statistics, dict):
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义任务统计无效")
        destination_count = raw_statistics.get("destination_count")
        avoidance_zone_count = raw_statistics.get("avoidance_zone_count")
        if (
            destination_count != len(destination_poses)
            or avoidance_zone_count != len(avoidance_zones)
            or raw_statistics.get("path_planner") != "nav2_smac_hybrid"
        ):
            raise ApiError(HTTPStatus.BAD_REQUEST, "语义任务统计无效")
        statistics = {
            "destination_count": destination_count,
            "avoidance_zone_count": avoidance_zone_count,
            "path_planner": "nav2_smac_hybrid",
            "route_navigation_places": raw_statistics.get(
                "route_navigation_places"
            ),
            "drivable_route_length_m": raw_statistics.get(
                "drivable_route_length_m"
            ),
            "search_algorithm": raw_statistics.get("search_algorithm"),
            "astar_cost_m": raw_statistics.get("astar_cost_m"),
        }
        self._assert_navigation_ready()
        semantic_state = {
            "available": True,
            "map_id": map_id,
            "status": "ready",
            "instruction": instruction.strip(),
            "destination_poses": destination_poses,
            "destinations": validated_destinations,
            "route": route,
            "route_centerline": route_centerline,
            "avoid_node_ids": avoid_node_ids,
            "avoidance_zones": avoidance_zones,
            "execution_allowed": True,
            "costmap_ready": False,
            "statistics": statistics,
            "provider": "alibaba_cloud_bailian",
            "model": "qwen3.7-flash",
            "graph_sha256": graph_digest,
            "request_id": str(body.get("request_id", ""))[:64],
            "error": "",
        }
        costmap_source = {
            "route_centerline": route_centerline,
            "avoidance_zones": avoidance_zones,
            "corridor_policy": corridor_policy,
            "verification_points": [],
        }
        with self._lock:
            map_grid = self._grids.get("map")
        if map_grid is None:
            raise ApiError(HTTPStatus.CONFLICT, "二维地图尚未发布，无法生成语义代价层")
        try:
            mask = self._publish_semantic_costmap(map_grid, costmap_source)
        except RouteCostmapError as error:
            raise ApiError(
                HTTPStatus.BAD_REQUEST, f"无法生成语义避让代价层: {error}"
            ) from error
        row, column = np.unravel_index(int(np.argmax(mask)), mask.shape)
        local_x = (float(column) + 0.5) * map_grid.resolution
        local_y = (float(row) + 0.5) * map_grid.resolution
        cosine = math.cos(map_grid.origin_yaw)
        sine = math.sin(map_grid.origin_yaw)
        costmap_source["verification_points"] = [
            {
                "x": map_grid.origin_x + cosine * local_x - sine * local_y,
                "y": map_grid.origin_y + sine * local_x + cosine * local_y,
                "minimum_cost": 180,
            }
        ]
        with self._lock:
            current_runtime = self._state.get("active_runtime")
            if not isinstance(current_runtime, dict) or current_runtime.get("map_id") != map_id:
                raise ApiError(HTTPStatus.CONFLICT, "接收路线期间运行地图已切换")
            self._state["semantic"] = semantic_state
            self._semantic_costmap_source = costmap_source
        self._touch()
        return {"semantic": semantic_state}

    def execute_semantic_navigation(self, _body: dict) -> dict:
        with self._lock:
            semantic = json.loads(json.dumps(self._state["semantic"]))
            active_runtime = self._state.get("active_runtime")
        if semantic.get("status") != "ready" or not semantic.get(
            "destination_poses"
        ):
            raise ApiError(HTTPStatus.CONFLICT, "请先生成有效的语义目标")
        if not isinstance(active_runtime, dict) or semantic.get("map_id") != active_runtime.get(
            "map_id"
        ):
            raise ApiError(HTTPStatus.CONFLICT, "语义路线与当前运行地图不一致")
        if semantic.get("execution_allowed") is not True:
            raise ApiError(
                HTTPStatus.CONFLICT,
                "语义路线尚未通过执行策略校验",
            )
        if semantic.get("costmap_ready") is not True:
            raise ApiError(HTTPStatus.CONFLICT, "语义避让代价层尚未就绪")
        destination_poses = [
            pose_document(value) for value in semantic["destination_poses"]
        ]
        result = self._send_navigation_route(destination_poses, "semantic")
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
            self._semantic_costmap_source = None
        self._publish_neutral_semantic_costmap()
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

    def _cancel_response_callback(self, future, handle):
        try:
            response = future.result()
            accepted = bool(response.goals_canceling)
        except Exception as error:  # pragma: no cover - rclpy boundary
            accepted = False
            detail = str(error)
        else:
            detail = "Nav2拒绝取消当前导航任务"

        with self._lock:
            if self._goal_handle is not handle:
                return
            self._goal_handle = None
            self._state["navigation"]["status"] = (
                "canceled" if accepted else "failed"
            )
            if not accepted:
                self._state["navigation"]["feedback"] = {"message": detail}
        self._touch()

    def cancel_navigation(self, _body: dict) -> dict:
        if getattr(self, "_saved_runner", None) and self._saved_runner.active:
            self._saved_runner.stop()
            return {"status": "canceled"}
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
        with self._lock:
            self._state["navigation"]["status"] = "canceling"
        self._touch()
        future = handle.cancel_goal_async()
        future.add_done_callback(
            lambda response, active_handle=handle: self._cancel_response_callback(
                response, active_handle
            )
        )
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
            self._chassis_feedback.reset()
            self._state["active_runtime"] = None
            self._state["active_collection"] = None
            self._state["active_processing"] = None
            self._state["semantic"] = self._empty_semantic_state()
            self._semantic_costmap_source = None
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
        vehicle_type = validated_identifier(
            str(body.get("vehicle_type", "")), "车型名称"
        )
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
        command = self.runtime_profiles.collection_command(vehicle_type, bag_path)
        vehicle = self.runtime_profiles.vehicle(vehicle_type)
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
                    "vehicle_type": vehicle_type,
                }
                self._state["vehicle"] = {
                    key: value
                    for key, value in vehicle.items()
                    if key != "collection"
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
        vehicle_type = validated_identifier(
            str(body.get("vehicle_type", "")), "车型名称"
        )
        map_id = str(body.get("map_id", ""))
        motion = bool(body.get("motion", False))
        refinement = body.get("initialization_refinement")
        wait_for_request = body.get("initialization_wait_for_request", False)
        if motion and body.get("motion_confirmed") is not True:
            raise ApiError(HTTPStatus.BAD_REQUEST, "真车运动必须再次明确确认")
        map_base = self.map_catalog.map_base(map_id)
        vehicle = self.runtime_profiles.vehicle(vehicle_type)
        with self._task_transition_lock:
            command = self.runtime_profiles.command(
                profile_id, map_base, motion, vehicle_type,
                self.physical_config.launch_arguments(vehicle_type),
                initialization_refinement=refinement,
                initialization_wait_for_request=wait_for_request,
            )
            stopped = self._stop_active_tasks()
            self.processes.runtime.start(
                command,
                self._log_path("runtime", f"{profile_id}_{map_id}"),
                os.environ.copy(),
            )
            with self._lock:
                self._state["pose"] = None
                self._state["paths"] = {"history": [], "global": [], "local": []}
                self._state["saved_path_preview"] = None
                self._state["footprint"] = None
                # Readiness belongs to the new runtime, never the previous map/mode.
                self._state["localization"].update({
                    "ready": False, "lidar_ready": False, "fusion_ready": False,
                    "fixed_active": False, "rtk_seed_ready": False,
                    "initialization_stage": None, "initialization_source": "none",
                    "initialization_status": "等待初始化节点",
                    "manual_required": False, "status": "未收到",
                    "rtk_initializer_status": "未收到",
                    "fix_quality": None, "heading_solution": "未收到",
                    "active_method": None,
                })
                self._grids.clear()
                self._navigation_health = NavigationInputHealth()
                self._state["active_runtime"] = {
                    "initialization_refinement": ("rtk_only"
                        if self.runtime_profiles.get(profile_id)["launch_package"] == "agribot_localization_switch"
                        else refinement or "ndt_gicp"),
                    "initialization_wait_for_request": wait_for_request,
                    "profile_id": profile_id,
                    "vehicle_type": vehicle_type,
                    "map_id": map_id,
                    "motion": motion,
                    "localization_strategy": ("navsat_waypoint100"
                        if self.runtime_profiles.get(profile_id)["launch_package"] == "agribot_localization_switch"
                        else "fastlivo_rtk"),
                    "configuration_source": (
                        self.physical_config.public()["source"]
                        if vehicle_type == "ackermann" else "default"
                    ),
                }
                self._state["vehicle"] = {
                    key: value
                    for key, value in vehicle.items()
                    if key != "collection"
                }
                self._state["semantic"] = self._empty_semantic_state(map_id)
                self._semantic_costmap_source = None
        self._touch()
        return {
            "profile_id": profile_id,
            "vehicle_type": vehicle_type,
            "map_id": map_id,
            "motion": motion,
            "stopped": stopped,
        }

    def _change_physical_config(self, document=None, *, reset=False) -> dict:
        with self._task_transition_lock:
            if any(value.get("running") for value in self.processes.snapshot().values()):
                raise ApiError(HTTPStatus.CONFLICT, "请先停止运行栈和采集任务，再更换真机配置")
            result = self.physical_config.reset() if reset else self.physical_config.import_document(document)
            self._touch()
            return {"physical_config": result}

    def import_physical_config(self, body: dict) -> dict:
        if body.get("vehicle_type") != "ackermann":
            raise PhysicalConfigError("本次导出仅适用于已验证的阿克曼车，差速车保持原配置")
        return self._change_physical_config(body.get("configuration"))

    def reset_physical_config(self, _body: dict) -> dict:
        return self._change_physical_config(reset=True)

    def stop_runtime(self, _body: dict) -> dict:
        with self._task_transition_lock:
            self.cancel_navigation({})
            self.processes.runtime.stop()
            with self._lock:
                runtime = self._state["active_runtime"]
                self._goal_handle = None
                self._chassis_feedback.reset()
                self._navigation_health = NavigationInputHealth()
                self._state["active_runtime"] = None
                self._state["semantic"] = self._empty_semantic_state()
                self._semantic_costmap_source = None
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
        try:
            self._saved_runner.stop()
        except RuntimeError as error:
            self.get_logger().error(str(error))
        self.vcu_monitor.close()
        self._stopping = True
        self._touch()
        for server, thread in (
            (self._http_server, self._http_thread),
            (self._https_server, self._https_thread),
        ):
            if server is not None:
                server.shutdown()
                server.server_close()
                if thread is not None:
                    thread.join(timeout=2.0)
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
