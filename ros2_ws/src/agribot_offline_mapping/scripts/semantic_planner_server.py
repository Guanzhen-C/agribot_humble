#!/usr/bin/env python3

"""HTTP boundary for phone-initiated Bailian semantic route planning."""

import argparse
import hashlib
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time
import uuid


MAXIMUM_REQUEST_BYTES = 64 * 1024
MAXIMUM_ROUTE_POSES = 100
MAXIMUM_ROUTE_CENTERLINE_POINTS = 10000


class SemanticServiceError(RuntimeError):
    pass


def strict_json_loads(value, description):
    def reject_constant(constant):
        raise ValueError("non-finite JSON number: {}".format(constant))

    def reject_duplicate_keys(pairs):
        result = {}
        for key, item in pairs:
            if key in result:
                raise ValueError("duplicate JSON key: {}".format(key))
            result[key] = item
        return result

    try:
        return json.loads(
            value,
            parse_constant=reject_constant,
            object_pairs_hook=reject_duplicate_keys,
        )
    except (TypeError, ValueError, json.JSONDecodeError) as error:
        raise SemanticServiceError(
            "{}不是严格JSON: {}".format(description, error)
        ) from error


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite(value, description):
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise SemanticServiceError("{}不是有限数值".format(description)) from error
    if not math.isfinite(result) or abs(result) > 100000.0:
        raise SemanticServiceError("{}不是有限数值".format(description))
    return result


def validate_route_document(document, graph_path, map_id):
    if not isinstance(document, dict) or document.get("schema_version") != 3:
        raise SemanticServiceError("语义路线格式无效")
    if document.get("frame_id") != "map":
        raise SemanticServiceError("语义路线不在map坐标系")
    graph_digest = file_sha256(graph_path)
    if document.get("graph_sha256") != graph_digest:
        raise SemanticServiceError("语义路线引用了过期图谱")
    policy = document.get("execution_policy")
    if not isinstance(policy, dict) or policy.get("preview_only") is not True:
        raise SemanticServiceError("语义路线缺少预览安全标记")
    route = document.get("route")
    poses = route.get("poses") if isinstance(route, dict) else None
    if not isinstance(poses, list) or not 2 <= len(poses) <= MAXIMUM_ROUTE_POSES:
        raise SemanticServiceError("A*路线必须包含2至100个位姿")
    validated_poses = []
    for item in poses:
        position = item.get("position") if isinstance(item, dict) else None
        if not isinstance(position, dict):
            raise SemanticServiceError("A*路线位姿无效")
        validated_poses.append(
            {
                "x": finite(position.get("x"), "A* X坐标"),
                "y": finite(position.get("y"), "A* Y坐标"),
                "yaw": finite(item.get("yaw", 0.0), "A*航向角"),
                "place_id": str(item.get("place_id", "")),
            }
        )
    raw_centerline = route.get("centerline")
    if (
        not isinstance(raw_centerline, list)
        or not 2 <= len(raw_centerline) <= MAXIMUM_ROUTE_CENTERLINE_POINTS
    ):
        raise SemanticServiceError("A*中心线点数无效")
    centerline = []
    for item in raw_centerline:
        if not isinstance(item, dict):
            raise SemanticServiceError("A*中心线点无效")
        centerline.append(
            {
                "x": finite(item.get("x"), "A*中心线X坐标"),
                "y": finite(item.get("y"), "A*中心线Y坐标"),
            }
        )
    task_plan = document.get("task_plan")
    if not isinstance(task_plan, dict):
        raise SemanticServiceError("语义路线缺少已校验任务计划")
    avoid_nodes = task_plan.get("avoid_node_ids", [])
    if not isinstance(avoid_nodes, list):
        raise SemanticServiceError("语义避让节点格式无效")
    resolved_stops = document.get("resolved_stops")
    if not isinstance(resolved_stops, list) or len(resolved_stops) < 2:
        raise SemanticServiceError("语义路线缺少有序目的地")
    destinations = []
    destination_poses = []
    for stop in resolved_stops[1:]:
        if not isinstance(stop, dict):
            raise SemanticServiceError("语义目的地无效")
        summary = stop.get("semantic_summary", [])
        if not isinstance(summary, list) or any(
            not isinstance(value, str) for value in summary
        ):
            raise SemanticServiceError("语义目的地描述无效")
        route_index = stop.get("navigation_route_index")
        if (
            isinstance(route_index, bool)
            or not isinstance(route_index, int)
            or not 0 <= route_index < len(validated_poses)
        ):
            raise SemanticServiceError("语义目的地没有有效的A*锚点")
        route_pose = validated_poses[route_index]
        place_id = str(stop.get("place_id", stop.get("selector", "")))
        if not place_id or route_pose["place_id"] != place_id:
            raise SemanticServiceError("语义目的地与A*锚点不一致")
        destinations.append(
            {
                "place_id": place_id,
                "name": str(stop.get("name", stop.get("selector", ""))),
                "semantic_summary": summary[:5],
            }
        )
        destination_poses.append(dict(route_pose))
    avoidance = document.get("avoidance_constraints")
    if not isinstance(avoidance, dict):
        raise SemanticServiceError("语义避让约束无效")
    avoidance_radius = finite(avoidance.get("radius_m", 0.0), "语义避让半径")
    if avoidance_radius < 0.0:
        raise SemanticServiceError("语义避让半径不能为负数")
    avoidance_nodes = avoidance.get("nodes", [])
    if not isinstance(avoidance_nodes, list):
        raise SemanticServiceError("语义避让节点无效")
    avoidance_zones = []
    for node in avoidance_nodes:
        position = node.get("position") if isinstance(node, dict) else None
        selector = str(node.get("selector", "")) if isinstance(node, dict) else ""
        if not selector or not isinstance(position, dict):
            raise SemanticServiceError("语义避让节点无效")
        avoidance_zones.append(
            {
                "selector": selector,
                "x": finite(position.get("x"), "语义避让X坐标"),
                "y": finite(position.get("y"), "语义避让Y坐标"),
                "radius_m": avoidance_radius,
            }
        )
    if sorted(zone["selector"] for zone in avoidance_zones) != sorted(
        str(value) for value in avoid_nodes
    ):
        raise SemanticServiceError("语义避让圆区与任务计划不一致")
    statistics = document.get("statistics", {})
    if not isinstance(statistics, dict):
        raise SemanticServiceError("语义路线统计无效")
    place_count = finite(
        statistics.get("route_navigation_places", len(validated_poses)),
        "拓扑点数量",
    )
    if not place_count.is_integer() or not 2 <= place_count <= MAXIMUM_ROUTE_POSES:
        raise SemanticServiceError("拓扑点数量无效")
    route_length = finite(
        statistics.get("drivable_route_length_m", 0.0), "路线长度"
    )
    if route_length < 0.0:
        raise SemanticServiceError("路线长度无效")
    search_algorithm = statistics.get("search_algorithm")
    if search_algorithm != "astar_euclidean_admissible":
        raise SemanticServiceError("语义路线未使用A*搜索")
    astar_cost = finite(statistics.get("astar_cost_m"), "A*代价")
    if astar_cost < 0.0:
        raise SemanticServiceError("A*代价无效")
    provenance = document.get("model_provenance", {})
    if provenance.get("provider") != "alibaba_cloud_bailian":
        raise SemanticServiceError("语义路线不是由阿里百炼生成")
    return {
        "available": True,
        "map_id": map_id,
        "status": "ready",
        "route": validated_poses,
        "route_centerline": centerline,
        "destination_poses": destination_poses,
        "destinations": destinations,
        "avoid_node_ids": [str(value) for value in avoid_nodes],
        "avoidance_zones": avoidance_zones,
        "execution_allowed": True,
        "costmap_policy": {
            "astar_centerline_is_soft_preference": True,
            "semantic_avoidance_is_lethal": True,
            "requires_nav2_keepout_filter": True,
        },
        "statistics": {
            "route_navigation_places": int(place_count),
            "drivable_route_length_m": route_length,
            "search_algorithm": search_algorithm,
            "astar_cost_m": astar_cost,
        },
        "model": str(provenance.get("model", "")),
        "provider": "alibaba_cloud_bailian",
        "graph_sha256": graph_digest,
        "task_id": str(task_plan.get("task_id", "")),
        "route_id": str(document.get("route_id", "")),
        "error": "",
    }


class SemanticPlannerService:
    def __init__(self, config_path):
        self.config_path = Path(config_path).expanduser().resolve()
        document = strict_json_loads(
            self.config_path.read_text(encoding="utf-8"), "语义服务配置"
        )
        self.host = str(document.get("host", "0.0.0.0"))
        self.port = int(document.get("port", 8090))
        self.bailian_url = str(
            document.get(
                "bailian_url",
                "https://dashscope.aliyuncs.com/compatible-mode/v1",
            )
        )
        self.model = str(document.get("model", "qwen3.7-flash"))
        self.embedding_model = str(
            document.get("embedding_model", "text-embedding-v4")
        )
        self.embedding_dimensions = int(document.get("embedding_dimensions", 1024))
        self.timeout = float(document.get("planning_timeout", 240.0))
        self.work_root = Path(document.get("work_root", "./tasks")).expanduser().resolve()
        planner = document.get("planner_script")
        self.planner_script = Path(planner).expanduser().resolve()
        self.allowed_origins = set(str(value) for value in document.get("allowed_origins", ["*"]))
        raw_maps = document.get("maps")
        if not isinstance(raw_maps, dict) or not raw_maps:
            raise SemanticServiceError("语义服务必须配置至少一张地图")
        self.maps = {}
        for map_id, value in raw_maps.items():
            if not isinstance(value, dict):
                raise SemanticServiceError("地图配置无效: {}".format(map_id))
            graph = Path(value.get("graph", "")).expanduser().resolve()
            if not graph.is_file():
                raise SemanticServiceError("语义图谱不存在: {}".format(graph))
            password_env = str(value.get("neo4j_password_env", ""))
            if not password_env or not os.environ.get(password_env, ""):
                raise SemanticServiceError("未配置{}".format(password_env))
            self.maps[str(map_id)] = {
                "label": str(value.get("label", map_id)),
                "graph": graph,
                "graph_sha256": file_sha256(graph),
                "neo4j_http_uri": str(value.get("neo4j_http_uri", "")),
                "neo4j_user": str(value.get("neo4j_user", "neo4j")),
                "neo4j_database": str(value.get("neo4j_database", "neo4j")),
                "neo4j_password_env": password_env,
                "retrieval_top_k": int(value.get("retrieval_top_k", 5)),
                "maximum_start_distance": float(value.get("maximum_start_distance", 10.0)),
                "avoidance_radius": float(value.get("avoidance_radius", 2.0)),
            }
        if not self.planner_script.is_file():
            raise SemanticServiceError("规划脚本不存在: {}".format(self.planner_script))
        if not 1 <= self.port <= 65535 or not 10.0 <= self.timeout <= 600.0:
            raise SemanticServiceError("语义服务端口或超时无效")
        if self.model != "qwen3.7-flash":
            raise SemanticServiceError("语义服务必须使用qwen3.7-flash")
        if self.embedding_model != "text-embedding-v4" or self.embedding_dimensions != 1024:
            raise SemanticServiceError("百炼向量检索必须使用1024维text-embedding-v4")
        if not os.environ.get("DASHSCOPE_API_KEY", "").strip():
            raise SemanticServiceError("未配置DASHSCOPE_API_KEY")
        self.work_root.mkdir(parents=True, exist_ok=True)
        self.planning_lock = threading.Lock()
        self.started_at = time.time()

    def public_maps(self):
        return [
            {
                "map_id": map_id,
                "label": value["label"],
                "graph_sha256": value["graph_sha256"],
            }
            for map_id, value in sorted(self.maps.items())
        ]

    def plan(self, body):
        map_id = str(body.get("map_id", ""))
        profile = self.maps.get(map_id)
        if profile is None:
            raise SemanticServiceError("当前地图没有语义图谱")
        instruction = body.get("instruction")
        if not isinstance(instruction, str) or not instruction.strip():
            raise SemanticServiceError("语义任务不能为空")
        instruction = instruction.strip()
        if len(instruction) > 1000:
            raise SemanticServiceError("语义任务不能超过1000个字符")
        position = body.get("start_position")
        if not isinstance(position, dict):
            raise SemanticServiceError("缺少当前map坐标位姿")
        start_x = finite(position.get("x"), "当前位置X")
        start_y = finite(position.get("y"), "当前位置Y")
        expected_digest = body.get("graph_sha256")
        if expected_digest not in (None, "", profile["graph_sha256"]):
            raise SemanticServiceError("手机和服务器使用的语义图谱版本不一致")
        if not self.planning_lock.acquire(blocking=False):
            raise SemanticServiceError("已有语义任务正在规划")
        try:
            request_id = uuid.uuid4().hex[:16]
            directory = self.work_root / map_id / request_id
            directory.mkdir(parents=True, exist_ok=False)
            route_path = directory / "route.json"
            command = [
                "python3",
                str(self.planner_script),
                "--graph",
                str(profile["graph"]),
                "--map-id",
                map_id,
                "--instruction",
                instruction,
                "--task-id",
                "phone_{}".format(request_id),
                "--start-position",
                str(start_x),
                str(start_y),
                "--maximum-start-distance",
                str(profile["maximum_start_distance"]),
                "--avoidance-radius",
                str(profile["avoidance_radius"]),
                "--model",
                self.model,
                "--base-url",
                self.bailian_url,
                "--embedding-model",
                self.embedding_model,
                "--embedding-dimensions",
                str(self.embedding_dimensions),
                "--neo4j-http-uri",
                profile["neo4j_http_uri"],
                "--neo4j-user",
                profile["neo4j_user"],
                "--neo4j-database",
                profile["neo4j_database"],
                "--retrieval-top-k",
                str(profile["retrieval_top_k"]),
                "--intent-output",
                str(directory / "intent.json"),
                "--context-output",
                str(directory / "context.json"),
                "--task-plan-output",
                str(directory / "task_plan.json"),
                "--route-output",
                str(route_path),
            ]
            environment = os.environ.copy()
            environment["AGRIBOT_NEO4J_PASSWORD"] = environment[
                profile["neo4j_password_env"]
            ]
            completed = subprocess.run(
                command,
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=self.timeout,
                check=False,
            )
            (directory / "planner.log").write_text(
                completed.stdout or "", encoding="utf-8"
            )
            if completed.returncode != 0:
                tail = (completed.stdout or "语义规划失败").strip().splitlines()[-1]
                raise SemanticServiceError(tail[:500])
            document = strict_json_loads(
                route_path.read_text(encoding="utf-8"), "语义路线"
            )
            semantic = validate_route_document(document, profile["graph"], map_id)
            semantic["instruction"] = instruction
            return {"request_id": request_id, "semantic": semantic}
        except subprocess.TimeoutExpired as error:
            raise SemanticServiceError("阿里百炼语义规划超时") from error
        finally:
            self.planning_lock.release()


class SemanticRequestHandler(BaseHTTPRequestHandler):
    server_version = "AgribotSemantic/1"
    protocol_version = "HTTP/1.1"

    @property
    def service(self):
        return self.server.service

    def log_message(self, format_string, *args):
        print("{} - {}".format(self.address_string(), format_string % args), flush=True)

    def _origin(self):
        origin = self.headers.get("Origin", "")
        allowed = self.service.allowed_origins
        if "*" in allowed:
            return "*"
        return origin if origin in allowed else ""

    def _headers(self, status, content_type="application/json; charset=utf-8", length=0):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(length))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        origin = self._origin()
        if origin:
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Vary", "Origin")
        self.end_headers()

    def _json(self, status, document):
        payload = json.dumps(document, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self._headers(status, length=len(payload))
        self.wfile.write(payload)

    def _error(self, status, message):
        self._json(status, {"error": str(message)})

    def do_OPTIONS(self):
        self.send_response(HTTPStatus.NO_CONTENT)
        origin = self._origin()
        if origin:
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Vary", "Origin")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Max-Age", "600")
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_GET(self):
        if self.path == "/api/v1/health":
            self._json(
                HTTPStatus.OK,
                {
                    "status": "ready",
                    "provider": "alibaba_cloud_bailian",
                    "model": self.service.model,
                    "embedding_model": self.service.embedding_model,
                    "embedding_dimensions": self.service.embedding_dimensions,
                    "maps": self.service.public_maps(),
                    "uptime_s": int(time.time() - self.service.started_at),
                },
            )
            return
        if self.path == "/api/v1/semantic/maps":
            self._json(HTTPStatus.OK, {"semantic_maps": self.service.public_maps()})
            return
        self._error(HTTPStatus.NOT_FOUND, "未知接口")

    def do_POST(self):
        if self.path != "/api/v1/semantic/plan":
            self._error(HTTPStatus.NOT_FOUND, "未知接口")
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 1 <= length <= MAXIMUM_REQUEST_BYTES:
                raise SemanticServiceError("请求体大小无效")
            body = strict_json_loads(
                self.rfile.read(length).decode("utf-8"), "手机语义规划请求"
            )
            if not isinstance(body, dict):
                raise SemanticServiceError("请求体必须是JSON对象")
            self._json(HTTPStatus.OK, self.service.plan(body))
        except SemanticServiceError as error:
            status = (
                HTTPStatus.CONFLICT
                if str(error) == "已有语义任务正在规划"
                else HTTPStatus.BAD_REQUEST
            )
            self._error(status, str(error))
        except Exception as error:
            self._error(HTTPStatus.INTERNAL_SERVER_ERROR, str(error))


class SemanticHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, service):
        self.service = service
        super().__init__(address, SemanticRequestHandler)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path)
    return parser.parse_args()


def main():
    arguments = parse_args()
    service = SemanticPlannerService(arguments.config)
    server = SemanticHttpServer((service.host, service.port), service)
    print(
        "Agribot Bailian semantic service listening on {}:{} with {} and {}.".format(
            service.host, service.port, service.model, service.embedding_model
        ),
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.5)
    finally:
        server.server_close()


if __name__ == "__main__":
    try:
        main()
    except (OSError, SemanticServiceError, UnicodeDecodeError) as error:
        raise SystemExit("error: {}".format(error)) from error
