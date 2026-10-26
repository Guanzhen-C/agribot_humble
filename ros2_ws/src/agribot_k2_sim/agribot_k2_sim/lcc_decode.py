"""Decode full finest portable LCC2/SOG2 attributes without appearance resampling."""
import io
import json
from pathlib import PurePosixPath
import struct
import zipfile

import numpy as np
from PIL import Image


def local_resource(root, name):
    path = PurePosixPath(name)
    if path.is_absolute() or ".." in path.parts or "\\" in name or "\x00" in name:
        raise ValueError("场景资源路径不安全。")
    result = (root / str(path)).resolve()
    if root.resolve() not in result.parents or not result.is_file():
        raise ValueError("场景引用的资源文件缺失：" + name)
    return result


def selected_sources(manifest):
    if (manifest.get("version") != "0.0.3" or manifest.get("shift") != [0, 0, 0]
            or manifest.get("scale") != [1, 1, 1]):
        raise ValueError("当前支持 LCC2 0.0.3、单位尺度和零 shift 的场景。")
    selected = {}

    def visit(node):
        children = node.get("child", {})
        if not children and "3dgs" in node.get("data", {}):
            item = node["data"]["3dgs"]
            selected.setdefault(item["name"], []).append((item["start"], item["count"]))
        for child in children.values():
            visit(child)

    visit(manifest["root"])
    foreground = sum(count for ranges in selected.values() for _, count in ranges)
    if foreground != manifest["lodSplats"][0] or foreground <= 0:
        raise ValueError("最高精度高斯层不完整。")
    environment = int(manifest.get("env", {}).get("splatsCount", 0))
    if environment:
        index = manifest["root"]["data"]["env"]["name"]
        if index in selected:
            raise ValueError("背景和前景资源索引重复。")
        selected[index] = [(0, environment)]
    if foreground + environment >= 2**24:
        raise ValueError("当前完整精度渲染器最多支持少于 16777216 个高斯，不会自动降采样。")
    return selected, foreground, environment


def decode_sog(path):
    with zipfile.ZipFile(path) as archive:
        meta = json.loads(archive.read("meta.json"))
        if meta.get("version") != 2 or "shN" in meta:
            raise ValueError("当前支持 SOG2 portable/DC 数据，不支持的球谐格式不会静默丢弃。")
        n = int(meta["count"])
        if not 0 < n < 2**24:
            raise ValueError("SOG 高斯数量无效。")

        def pixels(name):
            with Image.open(io.BytesIO(archive.read(name))) as image:
                array = np.asarray(image.convert("RGBA")).reshape(-1, 4)
                if len(array) < n:
                    raise ValueError("SOG 属性图像不完整。")
                return array[:n]

        low = pixels(meta["means"]["files"][0])[:, :3].astype(np.uint16)
        high = pixels(meta["means"]["files"][1])[:, :3].astype(np.uint16)
        lo, hi = np.asarray(meta["means"]["mins"]), np.asarray(meta["means"]["maxs"])
        log_position = lo + (hi - lo) * ((low + (high << 8)).astype(np.float64) / 65535)
        positions = (np.sign(log_position) * np.expm1(np.abs(log_position))).astype(np.float32)
        scales = np.asarray(meta["scales"]["codebook"], np.float32)[pixels(meta["scales"]["files"][0])[:, :3]]
        packed = pixels(meta["quats"]["files"][0])
        modes = packed[:, 3].astype(np.int16) - 252
        if not ((modes >= 0) & (modes <= 3)).all():
            raise ValueError("SOG 四元数编码无效。")
        kept = (packed[:, :3].astype(np.float32) / 255 - .5) * np.sqrt(2)
        missing = np.sqrt(np.maximum(0, 1 - (kept * kept).sum(axis=1)))
        quaternions = np.empty((n, 4), np.float32)
        for mode in range(4):
            mask = modes == mode
            quaternions[mask, mode] = missing[mask]
            for k, target in enumerate(i for i in range(4) if i != mode):
                quaternions[mask, target] = kept[mask, k]
        colors = pixels(meta["sh0"]["files"][0])
        dc = np.asarray(meta["sh0"]["codebook"], np.float32)[colors[:, :3]]
        qw, qx, qy, qz = quaternions.T
        rotation = np.empty((n, 3, 3), np.float32)
        rotation[:, 0, 0] = 1 - 2 * (qy*qy + qz*qz)
        rotation[:, 0, 1] = 2 * (qx*qy - qz*qw)
        rotation[:, 0, 2] = 2 * (qx*qz + qy*qw)
        rotation[:, 1, 0] = 2 * (qx*qy + qz*qw)
        rotation[:, 1, 1] = 1 - 2 * (qx*qx + qz*qz)
        rotation[:, 1, 2] = 2 * (qy*qz - qx*qw)
        rotation[:, 2, 0] = 2 * (qx*qz - qy*qw)
        rotation[:, 2, 1] = 2 * (qy*qz + qx*qw)
        rotation[:, 2, 2] = 1 - 2 * (qx*qx + qy*qy)
        covariance = (rotation * np.exp(2 * scales)[:, None, :]) @ rotation.transpose(0, 2, 1)
        result = np.empty((n, 13), dtype="<f4")
        result[:, :3] = positions
        result[:, 3:9] = covariance[:, [0, 0, 0, 1, 1, 2], [0, 1, 2, 1, 2, 2]]
        result[:, 9:12] = np.clip(.5 + dc * .28209479177387814, 0, 1)
        result[:, 12] = colors[:, 3].astype(np.float32) / 255
        if not np.isfinite(result).all():
            raise ValueError("高斯属性中有非有限数值。")
        return result


def decode_scene(root, manifest, cache, progress):
    selected, foreground, environment = selected_sources(manifest)
    total, written = foreground + environment, 0
    resources = manifest["root"]["splatFiles"]
    with cache.open("wb") as output:
        output.write(b"LCCGS001" + struct.pack("<Q", total))
        for index, ranges in sorted(selected.items()):
            data = decode_sog(local_resource(root, resources[index]))
            previous_end = 0
            for start, count in sorted(ranges):
                if (type(start) is not int or type(count) is not int or start < previous_end
                        or start < 0 or count <= 0 or start + count > len(data)):
                    raise ValueError("高斯层的资源范围重叠或越界。")
                output.write(data[start:start + count].tobytes())
                previous_end = start + count
                written += count
            progress(written / total)
    if written != total or cache.stat().st_size != 16 + total * 52:
        raise ValueError("高斯数据导出不完整。")
    return {"render_gaussians": total, "finest_source_gaussians": foreground,
            "environment_gaussians": environment, "extra_downsampling": False}
