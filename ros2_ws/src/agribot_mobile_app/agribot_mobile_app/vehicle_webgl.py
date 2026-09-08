"""Versioned catalog and safe file lookup for an external Unity WebGL build."""

from __future__ import annotations

from dataclasses import dataclass
import gzip
import hashlib
from http import HTTPStatus
import mimetypes
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import threading
from typing import BinaryIO

try:
    import brotli
except ImportError:  # pragma: no cover - covered by deployment dependency
    brotli = None


MAX_DECODED_ASSET_BYTES = 2 * 1024 * 1024 * 1024
MIN_CACHE_FREE_BYTES = 128 * 1024 * 1024


class VehicleWebGlError(RuntimeError):
    pass


class VehicleWebGlNotFound(VehicleWebGlError):
    pass


class VehicleWebGlRangeError(VehicleWebGlError):
    def __init__(self, size: int):
        super().__init__("Unity资源请求范围无效")
        self.size = size


@dataclass(frozen=True)
class VehicleWebGlAsset:
    path: Path
    relative_path: str
    size: int
    sha256: str
    content_type: str
    content_encoding: str | None


@dataclass(frozen=True)
class VehicleWebGlResponse:
    asset: VehicleWebGlAsset
    status: HTTPStatus
    offset: int
    length: int
    content_type: str
    content_encoding: str | None
    etag: str
    content_range: str | None = None


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _content_metadata(relative_path: str) -> tuple[str, str | None]:
    source_name = relative_path
    content_encoding = None
    if source_name.endswith(".br"):
        source_name = source_name[:-3]
        content_encoding = "br"
    elif source_name.endswith(".gz"):
        source_name = source_name[:-3]
        content_encoding = "gzip"

    suffix = Path(source_name).suffix.lower()
    if source_name.endswith(".symbols.json"):
        content_type = "application/octet-stream"
    elif suffix == ".wasm":
        content_type = "application/wasm"
    elif suffix == ".data":
        content_type = "application/octet-stream"
    elif suffix == ".js":
        content_type = "application/javascript; charset=utf-8"
    elif suffix == ".json":
        content_type = "application/json; charset=utf-8"
    else:
        content_type = (
            mimetypes.guess_type(source_name)[0] or "application/octet-stream"
        )
        if content_type.startswith("text/"):
            content_type += "; charset=utf-8"
    return content_type, content_encoding


def accepts_content_encoding(value: str | None, encoding: str) -> bool:
    """Return whether an HTTP Accept-Encoding value permits an encoding."""
    if not value:
        return False
    wildcard_quality = None
    for item in value.split(","):
        fields = [field.strip() for field in item.split(";")]
        name = fields[0].lower()
        quality = 1.0
        for parameter in fields[1:]:
            key, separator, raw_value = parameter.partition("=")
            if separator and key.strip().lower() == "q":
                try:
                    quality = float(raw_value.strip())
                except ValueError:
                    quality = 0.0
        if name == encoding.lower():
            return quality > 0
        if name == "*":
            wildcard_quality = quality
    return wildcard_quality is not None and wildcard_quality > 0


def parse_single_byte_range(
    value: str | None,
    size: int,
) -> tuple[int, int] | None:
    """Parse one inclusive byte range."""
    if value is None:
        return None
    match = re.fullmatch(r"bytes=(\d*)-(\d*)", value.strip())
    if not match or size <= 0:
        raise VehicleWebGlRangeError(size)
    start_text, end_text = match.groups()
    if not start_text and not end_text:
        raise VehicleWebGlRangeError(size)
    try:
        start_value = int(start_text) if start_text else None
        end_value = int(end_text) if end_text else None
    except (ValueError, OverflowError) as error:
        raise VehicleWebGlRangeError(size) from error
    if start_value is None:
        suffix_length = end_value
        if suffix_length is None or suffix_length <= 0:
            raise VehicleWebGlRangeError(size)
        return max(0, size - suffix_length), size - 1
    start = start_value
    if start >= size:
        raise VehicleWebGlRangeError(size)
    end = size - 1 if end_value is None else min(end_value, size - 1)
    if end < start:
        raise VehicleWebGlRangeError(size)
    return start, end


def _etag_matches(value: str | None, etag: str) -> bool:
    if not value:
        return False
    expected = etag.removeprefix("W/")
    return any(
        candidate == "*" or candidate.removeprefix("W/") == expected
        for candidate in (part.strip() for part in value.split(","))
    )


def plan_asset_response(
    asset: VehicleWebGlAsset,
    *,
    raw_download: bool = False,
    range_header: str | None = None,
    if_none_match: str | None = None,
    if_range: str | None = None,
) -> VehicleWebGlResponse:
    """Resolve response metadata without reading the asset into memory."""
    etag_suffix = "-raw" if raw_download else ""
    etag = f'"sha256-{asset.sha256}{etag_suffix}"'
    content_type = (
        "application/octet-stream" if raw_download else asset.content_type
    )
    content_encoding = None if raw_download else asset.content_encoding
    if _etag_matches(if_none_match, etag):
        return VehicleWebGlResponse(
            asset=asset,
            status=HTTPStatus.NOT_MODIFIED,
            offset=0,
            length=0,
            content_type=content_type,
            content_encoding=content_encoding,
            etag=etag,
        )

    requested_range = range_header
    if requested_range and if_range and if_range.strip() != etag:
        requested_range = None
    byte_range = parse_single_byte_range(requested_range, asset.size)
    if byte_range is None:
        return VehicleWebGlResponse(
            asset=asset,
            status=HTTPStatus.OK,
            offset=0,
            length=asset.size,
            content_type=content_type,
            content_encoding=content_encoding,
            etag=etag,
        )
    start, end = byte_range
    return VehicleWebGlResponse(
        asset=asset,
        status=HTTPStatus.PARTIAL_CONTENT,
        offset=start,
        length=end - start + 1,
        content_type=content_type,
        content_encoding=content_encoding,
        etag=etag,
        content_range=f"bytes {start}-{end}/{asset.size}",
    )


def stream_asset_response(
    response: VehicleWebGlResponse,
    target: BinaryIO,
    *,
    chunk_size: int = 1024 * 1024,
) -> None:
    """Copy the selected stored bytes while keeping memory use bounded."""
    if response.status == HTTPStatus.NOT_MODIFIED or response.length == 0:
        return
    remaining = response.length
    with response.asset.path.open("rb") as stream:
        stream.seek(response.offset)
        while remaining:
            chunk = stream.read(min(chunk_size, remaining))
            if not chunk:
                raise OSError("Unity资源在传输期间被截断")
            target.write(chunk)
            remaining -= len(chunk)


class VehicleWebGlCatalog:
    """Indexes a deployed build without copying it into the ROS workspace."""

    def __init__(self, root: Path, decoded_cache_root: Path | None = None):
        self.root = root.expanduser()
        self.decoded_cache_root = (
            decoded_cache_root
            or Path.home() / ".cache" / "agribot_mobile_app" / "vehicle-webgl"
        ).expanduser()
        self._lock = threading.Lock()
        self._decode_locks_guard = threading.Lock()
        self._decode_locks: dict[str, threading.Lock] = {}
        self._signature = None
        self._manifest = None

    def _source_files(self) -> list[tuple[str, Path]]:
        if not self.root.is_dir():
            raise VehicleWebGlError(f"三维配置资源目录不存在: {self.root}")
        resolved_root = self.root.resolve()
        files = []
        for path in sorted(self.root.rglob("*")):
            if path.is_symlink() or not path.is_file():
                continue
            resolved = path.resolve()
            try:
                resolved.relative_to(resolved_root)
            except ValueError as error:
                raise VehicleWebGlError("三维配置资源越出指定目录") from error
            relative = path.relative_to(self.root).as_posix()
            if any(
                part.startswith(".") for part in PurePosixPath(relative).parts
            ):
                continue
            files.append((relative, path))
        if not files:
            raise VehicleWebGlError("三维配置资源目录为空")
        if len(files) > 4096:
            raise VehicleWebGlError("三维配置资源文件数量异常")
        return files

    @staticmethod
    def _role(
        files: list[str],
        endings: tuple[str, ...],
        description: str,
    ) -> str:
        matches = [name for name in files if name.endswith(endings)]
        if len(matches) != 1:
            raise VehicleWebGlError(f"无法唯一确定Unity {description}文件")
        return matches[0]

    @staticmethod
    def _product(index_path: Path) -> dict:
        text = index_path.read_text(encoding="utf-8", errors="replace")

        def value(name: str, fallback: str) -> str:
            match = re.search(rf'{name}:\s*"([^"]*)"', text)
            return match.group(1) if match else fallback

        return {
            "company": value("companyName", ""),
            "name": value("productName", "车辆三维配置"),
            "version": value("productVersion", ""),
        }

    def manifest(self) -> dict:
        source_files = self._source_files()
        signature = tuple(
            (relative, path.stat().st_size, path.stat().st_mtime_ns)
            for relative, path in source_files
        )
        with self._lock:
            if signature == self._signature and self._manifest is not None:
                return self._manifest

            relative_names = [relative for relative, _ in source_files]
            if "index.html" not in relative_names:
                raise VehicleWebGlError("Unity WebGL缺少index.html")
            unity = {
                "loader": self._role(relative_names, (".loader.js",), "加载器"),
                "data": self._role(
                    relative_names,
                    (".data", ".data.br", ".data.gz"),
                    "数据",
                ),
                "framework": self._role(
                    relative_names,
                    (".framework.js", ".framework.js.br", ".framework.js.gz"),
                    "框架",
                ),
                "code": self._role(
                    relative_names,
                    (".wasm", ".wasm.br", ".wasm.gz"),
                    "WebAssembly",
                ),
            }

            entries = []
            version_digest = hashlib.sha256()
            total_bytes = 0
            for relative, path in source_files:
                size = path.stat().st_size
                checksum = _sha256(path)
                content_type, content_encoding = _content_metadata(relative)
                entry = {
                    "path": relative,
                    "size": size,
                    "sha256": checksum,
                    "content_type": content_type,
                }
                if content_encoding:
                    entry["content_encoding"] = content_encoding
                entries.append(entry)
                total_bytes += size
                version_digest.update(relative.encode("utf-8"))
                version_digest.update(b"\0")
                version_digest.update(checksum.encode("ascii"))
                version_digest.update(b"\0")

            manifest = {
                "available": True,
                "version": version_digest.hexdigest()[:16],
                "asset_base": "/vehicle-webgl",
                "raw_download": {
                    "header": "X-Agribot-Raw-Asset",
                    "value": "1",
                    "query": "download=raw",
                    "hash_scope": "stored_file_bytes",
                },
                "entrypoint": "index.html",
                "total_bytes": total_bytes,
                "product": self._product(self.root / "index.html"),
                "unity": unity,
                "files": entries,
            }
            self._signature = signature
            self._manifest = manifest
            return manifest

    def public_manifest(self) -> dict:
        try:
            return self.manifest()
        except VehicleWebGlError as error:
            return {
                "available": False,
                "asset_base": "/vehicle-webgl",
                "message": str(error),
                "files": [],
            }

    def asset_descriptor(self, relative_path: str) -> VehicleWebGlAsset:
        pure = PurePosixPath(relative_path)
        if (
            not relative_path
            or pure.is_absolute()
            or ".." in pure.parts
            or "\\" in relative_path
            or pure.as_posix() != relative_path
            or any(part.startswith(".") for part in pure.parts)
            or any(ord(character) < 32 for character in relative_path)
        ):
            raise VehicleWebGlNotFound("Unity资源路径无效")
        manifest = self.manifest()
        entry = next(
            (
                item
                for item in manifest["files"]
                if item["path"] == relative_path
            ),
            None,
        )
        if entry is None:
            raise VehicleWebGlNotFound("Unity资源文件不存在")
        root = self.root.resolve()
        current = self.root
        for part in pure.parts:
            current = current / part
            if current.is_symlink():
                raise VehicleWebGlNotFound("Unity资源文件不存在")
        candidate = (root / Path(*pure.parts)).resolve()
        try:
            candidate.relative_to(root)
        except ValueError as error:
            raise VehicleWebGlNotFound("Unity资源路径越界") from error
        if not candidate.is_file() or candidate.is_symlink():
            raise VehicleWebGlNotFound("Unity资源文件不存在")
        if candidate.stat().st_size != entry["size"]:
            raise VehicleWebGlError("Unity资源在索引后发生变化，请重试")
        return VehicleWebGlAsset(
            path=candidate,
            relative_path=relative_path,
            size=entry["size"],
            sha256=entry["sha256"],
            content_type=entry["content_type"],
            content_encoding=entry.get("content_encoding"),
        )

    def browser_asset_descriptor(
        self,
        relative_path: str,
        accept_encoding: str | None,
    ) -> VehicleWebGlAsset:
        """Select compressed or identity content for an HTTP client."""
        asset = self.asset_descriptor(relative_path)
        if (
            asset.content_encoding is None
            or accepts_content_encoding(
                accept_encoding,
                asset.content_encoding,
            )
        ):
            return asset
        return self._decoded_asset_descriptor(asset)

    def _decoded_asset_descriptor(
        self,
        asset: VehicleWebGlAsset,
    ) -> VehicleWebGlAsset:
        encoding = asset.content_encoding
        if encoding not in ("br", "gzip"):
            raise VehicleWebGlError("Unity资源使用了不支持的压缩格式")
        if encoding == "br" and brotli is None:
            raise VehicleWebGlError(
                "浏览器不支持Brotli传输，RDK还需安装python3-brotli"
            )

        relative = PurePosixPath(asset.relative_path)
        decoded_name = relative.name[: -(len(encoding) + 1)]
        decoded_path = (
            self.decoded_cache_root
            / "identity-v1"
            / encoding
            / asset.sha256
            / Path(*relative.parent.parts)
            / decoded_name
        )
        cache_key = f"{encoding}:{asset.sha256}"
        with self._decode_locks_guard:
            decode_lock = self._decode_locks.setdefault(
                cache_key,
                threading.Lock(),
            )
        with decode_lock:
            if not decoded_path.is_file():
                decoded_path.parent.mkdir(parents=True, exist_ok=True)
                available = shutil.disk_usage(decoded_path.parent).free
                output_limit = min(
                    MAX_DECODED_ASSET_BYTES,
                    max(0, available - MIN_CACHE_FREE_BYTES),
                )
                if output_limit <= 0:
                    raise VehicleWebGlError("Unity网页解压缓存空间不足")
                temporary = decoded_path.with_name(
                    ".{}.{}.{}.tmp".format(
                        decoded_path.name,
                        os.getpid(),
                        threading.get_ident(),
                    )
                )
                try:
                    decoded_size = 0

                    def write_checked(target, chunk):
                        nonlocal decoded_size
                        decoded_size += len(chunk)
                        if decoded_size > output_limit:
                            raise VehicleWebGlError(
                                "Unity网页解压资源超过大小或磁盘限制"
                            )
                        target.write(chunk)

                    if encoding == "br":
                        decoder = brotli.Decompressor()
                        with asset.path.open("rb") as source, temporary.open(
                            "wb"
                        ) as target:
                            chunks = iter(
                                lambda: source.read(1024 * 1024),
                                b"",
                            )
                            for chunk in chunks:
                                write_checked(target, decoder.process(chunk))
                        if not decoder.is_finished():
                            raise VehicleWebGlError("Unity Brotli资源不完整")
                    else:
                        with gzip.open(
                            asset.path,
                            "rb",
                        ) as source, temporary.open("wb") as target:
                            chunks = iter(
                                lambda: source.read(1024 * 1024),
                                b"",
                            )
                            for chunk in chunks:
                                write_checked(target, chunk)
                    with temporary.open("rb+") as target:
                        target.flush()
                        os.fsync(target.fileno())
                    os.replace(temporary, decoded_path)
                except Exception:
                    temporary.unlink(missing_ok=True)
                    raise

        return VehicleWebGlAsset(
            path=decoded_path,
            relative_path=asset.relative_path,
            size=decoded_path.stat().st_size,
            sha256=f"{asset.sha256}-identity",
            content_type=asset.content_type,
            content_encoding=None,
        )

    def asset(self, relative_path: str) -> tuple[Path, str, str | None]:
        descriptor = self.asset_descriptor(relative_path)
        return (
            descriptor.path,
            descriptor.content_type,
            descriptor.content_encoding,
        )
