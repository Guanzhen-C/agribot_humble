"""Filesystem-backed map and bag catalogs used by the mobile gateway."""

from __future__ import annotations

import base64
from dataclasses import dataclass
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Iterable

import yaml


IDENTIFIER_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}")


class CatalogError(RuntimeError):
    """Raised when an artifact is invalid or outside its configured root."""


def validated_identifier(value: str, description: str = "名称") -> str:
    if not isinstance(value, str) or IDENTIFIER_PATTERN.fullmatch(value) is None:
        raise CatalogError(
            f"{description}只能包含字母、数字、下划线和连字符，长度1至64"
        )
    return value


def _load_yaml(path: Path) -> dict:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise CatalogError(f"无法读取YAML文件 {path}: {error}") from error
    if not isinstance(document, dict):
        raise CatalogError(f"YAML文件不是映射: {path}")
    return document


def _route_pose(value: object) -> dict:
    if not isinstance(value, dict):
        raise ValueError("固定路线位姿必须为对象")
    pose = {}
    for axis in ("x", "y", "yaw"):
        number = value.get(axis)
        if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number):
            raise ValueError(f"固定路线{axis}坐标必须为有限数字")
        pose[axis] = float(number)
    return pose


def _next_pgm_token(data: bytes, offset: int) -> tuple[bytes, int]:
    size = len(data)
    while offset < size:
        if data[offset] == ord("#"):
            newline = data.find(b"\n", offset)
            offset = size if newline < 0 else newline + 1
            continue
        if chr(data[offset]).isspace():
            offset += 1
            continue
        break
    start = offset
    while offset < size and not chr(data[offset]).isspace() and data[offset] != ord("#"):
        offset += 1
    if start == offset:
        raise CatalogError("PGM文件头不完整")
    return data[start:offset], offset


def read_pgm(path: Path) -> tuple[int, int, bytes]:
    """Read an 8-bit P2/P5 PGM and return top-to-bottom grayscale pixels."""

    try:
        data = path.read_bytes()
    except OSError as error:
        raise CatalogError(f"无法读取地图图像 {path}: {error}") from error
    magic, offset = _next_pgm_token(data, 0)
    width_token, offset = _next_pgm_token(data, offset)
    height_token, offset = _next_pgm_token(data, offset)
    maximum_token, offset = _next_pgm_token(data, offset)
    try:
        width = int(width_token)
        height = int(height_token)
        maximum = int(maximum_token)
    except ValueError as error:
        raise CatalogError(f"PGM文件头包含非整数: {path}") from error
    if magic not in (b"P2", b"P5") or width <= 0 or height <= 0:
        raise CatalogError(f"不支持的PGM地图: {path}")
    if maximum <= 0 or maximum > 255:
        raise CatalogError(f"只支持8位PGM地图: {path}")

    count = width * height
    if magic == b"P2":
        values = []
        for _ in range(count):
            token, offset = _next_pgm_token(data, offset)
            try:
                values.append(round(int(token) * 255 / maximum))
            except ValueError as error:
                raise CatalogError(f"PGM像素包含非整数: {path}") from error
        return width, height, bytes(values)

    if offset >= len(data) or not chr(data[offset]).isspace():
        raise CatalogError(f"PGM二进制数据前缺少分隔符: {path}")
    if data[offset : offset + 2] == b"\r\n":
        offset += 2
    else:
        offset += 1
    pixels = data[offset : offset + count]
    if len(pixels) != count:
        raise CatalogError(f"PGM像素数量不足: {path}")
    if maximum != 255:
        pixels = bytes(round(value * 255 / maximum) for value in pixels)
    return width, height, pixels


def _signed_grid_bytes(values: Iterable[int]) -> bytes:
    return bytes(value & 0xFF for value in values)


@dataclass(frozen=True)
class GridData:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: bytes
    revision: int = 0

    def payload(self, layer: str) -> dict:
        return {
            "layer": layer,
            "revision": self.revision,
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin": {
                "x": self.origin_x,
                "y": self.origin_y,
                "yaw": self.origin_yaw,
            },
            "encoding": "int8-base64",
            "data": base64.b64encode(self.data).decode("ascii"),
        }


def grid_from_nav2_yaml(yaml_path: Path) -> GridData:
    document = _load_yaml(yaml_path)
    required = ("image", "resolution", "origin")
    if any(key not in document for key in required):
        raise CatalogError(f"不是有效的Nav2地图YAML: {yaml_path}")
    image = Path(str(document["image"])).expanduser()
    if not image.is_absolute():
        image = yaml_path.parent / image
    resolution = float(document["resolution"])
    origin = document["origin"]
    if (
        resolution <= 0.0
        or not isinstance(origin, (list, tuple))
        or len(origin) < 3
    ):
        raise CatalogError(f"Nav2地图分辨率或原点无效: {yaml_path}")
    width, height, pixels = read_pgm(image.resolve())
    negate = bool(int(document.get("negate", 0)))
    occupied_threshold = float(document.get("occupied_thresh", 0.65))
    free_threshold = float(document.get("free_thresh", 0.196))
    if not 0.0 <= free_threshold < occupied_threshold <= 1.0:
        raise CatalogError(f"Nav2地图阈值无效: {yaml_path}")

    # Only 256 possible input values; translate the full map in C, not Python.
    lookup = bytearray(256)
    for pixel in range(256):
        probability = pixel / 255.0 if negate else (255 - pixel) / 255.0
        lookup[pixel] = 100 if probability > occupied_threshold else 0 if probability < free_threshold else 255
    flipped = b"".join(pixels[row * width:(row + 1) * width] for row in range(height - 1, -1, -1))
    return GridData(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        origin_yaw=float(origin[2]),
        data=flipped.translate(bytes(lookup)),
    )


class MapCatalog:
    def __init__(self, root: Path):
        self.root = root.expanduser().resolve()

    def _yaml_path(self, map_id: str) -> Path:
        identifier = validated_identifier(map_id, "地图名称")
        return self.root / f"{identifier}.yaml"

    def list(self) -> list[dict]:
        if not self.root.is_dir():
            return []
        maps = []
        for yaml_path in sorted(self.root.glob("*.yaml")):
            map_id = yaml_path.stem
            if map_id.endswith(("_georeference", "_manifest", "_leveling")):
                continue
            try:
                document = _load_yaml(yaml_path)
                if not {"image", "resolution", "origin"}.issubset(document):
                    continue
                image = Path(str(document["image"])).expanduser()
                if not image.is_absolute():
                    image = yaml_path.parent / image
                image = image.resolve()
                if not image.is_file():
                    continue
                modified = max(yaml_path.stat().st_mtime, image.stat().st_mtime)
                base = self.root / map_id
                maps.append(
                    {
                        "id": map_id,
                        "resolution": float(document["resolution"]),
                        "modified_at": datetime.fromtimestamp(modified).isoformat(),
                        "has_3d": base.with_suffix(".pcd").is_file(),
                        "has_georeference": Path(
                            f"{base}_georeference.yaml"
                        ).is_file(),
                        "has_visual": Path(
                            f"{base}_visual_index.npz"
                        ).is_file(),
                        "has_manifest": Path(f"{base}_manifest.yaml").is_file(),
                        "has_route_presets": Path(f"{base}_routes.json").is_file(),
                    }
                )
            except (CatalogError, OSError, TypeError, ValueError):
                continue
        return maps

    def route_presets(self, map_id: str) -> dict:
        base = self.map_base(map_id)
        path = Path(f"{base}_routes.json")
        if not path.is_file():
            return {"map_id": map_id, "routes": []}
        if not path.resolve().is_relative_to(self.root) or path.stat().st_size > 256_000:
            raise CatalogError("固定路线文件路径或大小无效")
        try:
            document = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(document, dict) or document.get("schema_version") != 1:
                raise ValueError("路线格式版本无效")
            if document.get("map_id") != map_id or document.get("frame_id") != "map":
                raise ValueError("路线与当前地图坐标系不一致")
            binding = document["map_binding"]
            georeference = Path(f"{base}_georeference.yaml")
            if hashlib.sha256(georeference.read_bytes()).hexdigest() != binding["georeference_sha256"]:
                raise ValueError("地图地理配准已改变，请重新转换扫描路线")
            geometry = _load_yaml(base.with_suffix(".yaml"))
            if (geometry["resolution"] != binding["resolution"]
                    or geometry["origin"] != binding["origin"]):
                raise ValueError("地图原点或分辨率已改变，请重新转换扫描路线")
            routes = document["routes"]
            if not isinstance(routes, list) or not 1 <= len(routes) <= 20:
                raise ValueError("固定路线数量无效")
            ids = set()
            for route in routes:
                identifier = validated_identifier(route["id"], "路线编号")
                if identifier in ids:
                    raise ValueError("固定路线编号重复")
                ids.add(identifier)
                if not isinstance(route["label"], str) or not 1 <= len(route["label"]) <= 100:
                    raise ValueError("固定路线名称无效")
                if route["vehicle_type"] not in ("differential", "ackermann"):
                    raise ValueError("固定路线车型无效")
                poses = route["poses"]
                if not isinstance(poses, list) or not 2 <= len(poses) <= 110:
                    raise ValueError("固定路线必须包含2至110个位姿")
                route["poses"] = [_route_pose(pose) for pose in poses]
            return document
        except (OSError, UnicodeError, ValueError, KeyError, TypeError) as error:
            raise CatalogError(f"无法读取固定路线: {error}") from error

    def grid(self, map_id: str) -> GridData:
        yaml_path = self._yaml_path(map_id)
        if not yaml_path.is_file():
            raise CatalogError(f"地图不存在: {map_id}")
        return grid_from_nav2_yaml(yaml_path)

    def map_base(self, map_id: str) -> Path:
        yaml_path = self._yaml_path(map_id)
        if not yaml_path.is_file():
            raise CatalogError(f"地图不存在: {map_id}")
        return yaml_path.with_suffix("")


class BagCatalog:
    def __init__(self, root: Path):
        self.root = root.expanduser().resolve()

    def path(self, bag_id: str) -> Path:
        identifier = validated_identifier(bag_id, "数据包名称")
        path = self.root / identifier
        if not path.is_dir() or not (path / "metadata.yaml").is_file():
            raise CatalogError(f"数据包不存在或尚未完整写盘: {bag_id}")
        return path

    def list(self) -> list[dict]:
        if not self.root.is_dir():
            return []
        bags = []
        for path in sorted(self.root.iterdir(), reverse=True):
            metadata = path / "metadata.yaml"
            if not path.is_dir() or not metadata.is_file():
                continue
            stat = metadata.stat()
            bags.append(
                {
                    "id": path.name,
                    "modified_at": datetime.fromtimestamp(stat.st_mtime).isoformat(),
                    "metadata_bytes": stat.st_size,
                }
            )
        return bags


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
