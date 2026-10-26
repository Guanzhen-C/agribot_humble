"""Transactional local LCC2 ZIP import for the Unity scenario library."""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import signal
import stat
import struct
import sys
import tempfile
import uuid
import zipfile

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import vtk
import yaml

from .lcc_decode import decode_scene, local_resource, selected_sources
from .prepare import compile_renderer, project_map, read_mesh, write_world


def emit(stage, progress=0, **extra):
    print(json.dumps({"stage": stage, "progress": progress, **extra}, ensure_ascii=False), flush=True)


def validate_name(value):
    if not isinstance(value, str) or not 1 <= len(value.strip()) <= 48:
        raise ValueError("场景名称应为 1～48 个字符。")
    if any(ord(c) < 32 for c in value):
        raise ValueError("场景名称不能包含换行或控制字符。")
    return value.strip()


def extract_archive(archive, target):
    with zipfile.ZipFile(archive) as zf:
        files = zf.infolist()
        if len(files) > 10000 or sum(item.file_size for item in files) > 16 * 1024**3:
            raise ValueError("ZIP 文件数量或解压体积超过当前支持范围。")
        names = set()
        for item in files:
            path = PurePosixPath(item.filename)
            mode = item.external_attr >> 16
            if (path.is_absolute() or ".." in path.parts or "\\" in item.filename
                    or "\x00" in item.filename or stat.S_ISLNK(mode) or item.filename in names):
                raise ValueError("ZIP 包含不安全或重复的路径。")
            names.add(item.filename)
        manifests = [item for item in files if item.filename.lower().endswith(".lcc2")]
        if len(manifests) != 1:
            raise ValueError("请选择包含一份 .lcc2 索引、SOG 高斯和 PLY 碰撞网格的 ZIP。")
        if shutil.disk_usage(target.parent).free < sum(i.file_size for i in files) * 3 + 1024**3:
            raise ValueError("磁盘剩余空间不足，无法完整保留高斯和碰撞数据。")
        for item in files:
            destination = target / item.filename
            if item.is_dir():
                destination.mkdir(parents=True, exist_ok=True)
            else:
                destination.parent.mkdir(parents=True, exist_ok=True)
                with zf.open(item) as source, destination.open("wb") as output:
                    shutil.copyfileobj(source, output)
        manifest_path = target / manifests[0].filename
    return manifest_path.parent, manifest_path


def ground_at(locator, x, y, scan_z):
    hits, ids = vtk.vtkPoints(), vtk.vtkIdList()
    locator.IntersectWithLine([x, y, scan_z - .12], [x, y, scan_z - 6], hits, ids)
    levels = [hits.GetPoint(i)[2] for i in range(hits.GetNumberOfPoints())]
    return max(levels) if levels else None


def choose_spawn(folder, poses, locator, reference_z):
    metadata = yaml.safe_load((folder / "map.yaml").read_text())
    pixels = np.flipud(np.array(Image.open(folder / "map.pgm")))
    clearance = distance_transform_edt(pixels == 254) * metadata["resolution"]
    origin = np.array(metadata["origin"][:2])
    res = metadata["resolution"]
    ys, xs = np.nonzero(clearance >= 1.0)
    if not len(xs):
        raise ValueError("扫描地面缺少足够宽的可行驶出生区域。")
    coordinates = np.column_stack((xs, ys)) * res + origin
    for pose in poses[::max(1, len(poses)//200)]:
        p = np.asarray(pose["T"], dtype=float)
        distances = np.linalg.norm(coordinates - p[:2], axis=1)
        nearby = np.flatnonzero(distances <= 3)
        # Prefer interior road space over the closest curb-side scan position.
        scores = distances[nearby] - 2 * np.minimum(clearance[ys[nearby], xs[nearby]], 2)
        for i in nearby[np.argsort(scores)[:40]]:
            x, y = coordinates[i]
            ground = ground_at(locator, x, y, p[2])
            if ground is None or abs(ground - reference_z) > .35:
                continue
            support = [ground_at(locator, x + .9*math.cos(a), y + .9*math.sin(a), p[2])
                       for a in np.linspace(0, 2*math.pi, 8, endpoint=False)]
            if any(z is None for z in support) or max([ground, *support]) - min([ground, *support]) > .12:
                continue
            w, qx, qy, qz = pose["R"]
            yaw = math.atan2(2*(w*qz+qx*qy), 1-2*(qy*qy+qz*qz))
            return [float(x), float(y), yaw], float(ground - reference_z)
    raise ValueError("无法在原始扫描轨迹附近找到安全地面，请检查扫描地面和位姿数据。")


def prepare_asset(archive, folder, final, checksum):
    emit("解压与校验场景", .03)
    source, manifest_path = extract_archive(archive, folder / "source")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
    selected_sources(manifest)
    mesh_files = manifest["root"].get("meshFiles", [])
    if not mesh_files:
        raise ValueError("场景没有原始 PLY 网格，无法提供车辆碰撞和雷达仿真。")
    poses = json.loads(local_resource(source, "info/poses.json").read_text())["poses"]
    if not poses or any(len(p["T"]) != 3 or len(p["R"]) != 4 or
                        not all(math.isfinite(v) for v in [*p["T"], *p["R"]]) for p in poses):
        raise ValueError("扫描位姿无效，无法确定场景竖直方向及出生位置。")
    gaussian = folder / "gaussian"
    gaussian.mkdir()
    counts = decode_scene(source, manifest, gaussian / "gaussians.bin",
                          lambda fraction: emit("解码完整高斯", .08 + .45*fraction))
    (gaussian / "scene_config.json").write_text(json.dumps({
        **counts, "source_archive": str(archive), "source_manifest": str(manifest_path.relative_to(folder)),
        "georeference": {key: manifest.get(key) for key in ("epsg", "offset", "shift", "scale")},
    }, ensure_ascii=False, indent=2))
    package = Path(__file__).resolve().parents[1]
    if not (package / "vendor").is_dir():
        from ament_index_python.packages import get_package_share_directory
        package = Path(get_package_share_directory("agribot_k2_sim"))
    shutil.copytree(package / "vendor/shaders", gaussian / "shaders")
    shutil.copy2(package / "vendor/renderer/LICENSE", gaussian / "RENDERER_LICENSE")
    emit("准备高斯渲染器", .55)
    compile_renderer(None, gaussian)
    meshes, collisions, reports = [], [], []
    combined = vtk.vtkAppendPolyData()
    (folder / "meshes").mkdir()
    for i, relative in enumerate(mesh_files):
        vertices, faces, mesh = read_mesh(local_resource(source, relative))
        meshes.append((vertices, faces))
        combined.AddInputData(mesh)
        collision = folder / "meshes" / f"collision_{i:02d}.stl"
        writer = vtk.vtkSTLWriter()
        writer.SetFileName(str(collision))
        writer.SetFileTypeToBinary()
        writer.SetInputData(mesh)
        if writer.Write() != 1 or collision.stat().st_size != 84 + 50*len(faces):
            raise ValueError("原始碰撞网格导出失败。")
        collisions.append(collision)
        reports.append({"source": relative, "triangles": len(faces)})
        emit("准备原始碰撞网格", .60 + .15*(i+1)/len(mesh_files))
    combined.Update()
    locator = vtk.vtkOBBTree()
    locator.SetDataSet(combined.GetOutput())
    locator.BuildLocator()
    ground = None
    for p in poses[::max(1, len(poses)//200)]:
        ground = ground_at(locator, *p["T"])
        if ground is not None:
            break
    if ground is None:
        raise ValueError("原始扫描轨迹下方没有可用地面，无法自动放置车辆。")
    emit("生成导航地图与出生点", .79)
    map_report = project_map(meshes, ground, folder)
    spawn, spawn_ground = choose_spawn(folder, poses, locator, ground)
    x, y, yaw = spawn
    camera_pose = [x - 4*math.cos(yaw) + 2*math.sin(yaw),
                   y - 4*math.sin(yaw) - 2*math.cos(yaw), spawn_ground + 2.2, 0, .28, yaw]
    write_world(folder, collisions, -ground, camera_pose, final)
    (folder / "waypoints.yaml").write_text("waypoints: []\n")
    (folder / "scene.json").write_text(json.dumps({
        "schema_version": 1, "id": final.name, "world": "world.sdf", "map": "map.yaml",
        "waypoints": "waypoints.yaml", "spawn": spawn, "spawn_ground_z": spawn_ground,
        "manual_goals_only": True,
    }, indent=2))
    preview = folder / "preview.jpg"
    thumbnail = source / "info/thumb.jpg"
    with Image.open(thumbnail if thumbnail.is_file() else folder / "map_preview.png") as image:
        image.convert("RGB").save(preview, quality=92)
    report = {"schema_version": 1, "archive_sha256": checksum, **counts,
              "mesh_reports": reports, "collision_triangles": sum(r["triangles"] for r in reports),
              "map": map_report, "spawn": spawn, "scene_translation_z_m": -ground,
              "runtime_validated": False, "camera_renderer": "Full original EWA Gaussians",
              "rtk": "Simulated GNSS only; scan georeference retained as metadata"}
    (folder / "preparation.json").write_text(json.dumps(report, indent=2))


def import_scene(archive, name, library):
    name = validate_name(name)
    archive = Path(archive).expanduser().resolve()
    library = Path(library).expanduser().resolve()
    if archive.suffix.lower() != ".zip" or not archive.is_file():
        raise ValueError("请选择存在的 LCC2 ZIP 文件。")
    library.mkdir(parents=True, exist_ok=True)
    with (library / ".import.lock").open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError("已有场景正在导入，请等待或先取消。") from None
        catalog_path = library / "catalog.json"
        catalog = json.loads(catalog_path.read_text()) if catalog_path.exists() else {"schemaVersion": "1.0", "scenarios": []}
        if catalog.get("schemaVersion") != "1.0" or not isinstance(catalog.get("scenarios"), list):
            raise ValueError("本机场景目录格式无效，未覆盖原目录。")
        if any(scene["name"].casefold() == name.casefold() for scene in catalog["scenarios"]):
            raise ValueError("该场景名称已存在，请换一个名称。")
        emit("校验压缩包", .01)
        digest = hashlib.sha256()
        with archive.open("rb") as source:
            for chunk in iter(lambda: source.read(4*1024**2), b""):
                digest.update(chunk)
        checksum = digest.hexdigest()
        final = library / ("lcc2_v2_" + checksum[:24])
        if not final.exists():
            with tempfile.TemporaryDirectory(prefix=".import-", dir=library) as temporary:
                folder = Path(temporary) / "asset"
                folder.mkdir()
                prepare_asset(archive, folder, final, checksum)
                folder.rename(final)
        for required in ("scene.json", "world.sdf", "map.yaml", "map.pgm", "preview.jpg",
                         "gaussian/gaussians.bin", "gaussian/libK2GaussianVisual.so"):
            if not (final / required).is_file():
                raise ValueError("已有场景缓存不完整，请检查本机场景目录。")
        spawn = json.loads((final / "scene.json").read_text())["spawn"]
        scene = {"id": "scan_" + uuid.uuid4().hex[:12], "name": name,
                 "tasks": ["扫描场景"], "source": "LCC2", "spawn": spawn,
                 "imported": True, "sceneManifest": str(final / "scene.json"),
                 "previewFile": str(final / "preview.jpg")}
        catalog["scenarios"].append(scene)
        pending = catalog_path.with_suffix(".tmp")
        pending.write_text(json.dumps(catalog, ensure_ascii=False, indent=2), encoding="utf-8")
        pending.replace(catalog_path)
        emit("导入完成", 1, ok=True, scene=scene)
        return scene


def delete_scene(identifier, library):
    if not isinstance(identifier, str) or not re.fullmatch(r"scan_[A-Za-z0-9_-]+", identifier):
        raise ValueError("只能删除手工导入的场景。")
    library = Path(library).expanduser().resolve()
    if not library.is_dir():
        raise ValueError("该导入场景已不存在。")
    with (library / ".import.lock").open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError("场景目录正在更新，请稍后再试。") from None
        catalog_path = library / "catalog.json"
        catalog = json.loads(catalog_path.read_text())
        if catalog.get("schemaVersion") != "1.0" or not isinstance(catalog.get("scenarios"), list):
            raise ValueError("本机场景目录格式无效，未覆盖原目录。")
        matches = [scene for scene in catalog["scenarios"]
                   if isinstance(scene, dict) and scene.get("id") == identifier]
        if len(matches) != 1 or matches[0].get("imported") is not True:
            raise ValueError("该导入场景已不存在或不允许删除。")
        removed = matches[0]
        manifest = Path(removed["sceneManifest"])
        cache = manifest.parent.resolve()
        if (manifest.name != "scene.json" or manifest.parent.is_symlink() or cache.parent != library
                or not re.fullmatch(r"lcc2_v[0-9]+_[0-9a-f]{24}", cache.name)):
            raise ValueError("场景缓存路径不安全，未删除任何文件。")
        catalog["scenarios"] = [scene for scene in catalog["scenarios"] if scene is not removed]
        shared = any(isinstance(scene, dict) and scene.get("sceneManifest") and
                     Path(scene["sceneManifest"]).resolve().parent == cache for scene in catalog["scenarios"])
        pending = catalog_path.with_suffix(".tmp")
        pending.write_text(json.dumps(catalog, ensure_ascii=False, indent=2), encoding="utf-8")
        staged = None
        try:
            # Detach the cache before publishing, and restore it if publication fails.
            if not shared and cache.exists():
                staged = library / (".deleting-" + uuid.uuid4().hex)
                cache.rename(staged)
            pending.replace(catalog_path)
        except BaseException:
            if staged is not None:
                staged.rename(cache)
            pending.unlink(missing_ok=True)
            raise
        if staged is not None:
            try:
                shutil.rmtree(staged)
            except OSError as error:
                raise ValueError("场景已移除，但缓存清理失败，请检查目录权限：" + str(staged)) from error
        emit("删除完成", 1, ok=True, scene=removed)
        return removed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request-stdin", action="store_true")
    parser.add_argument("--archive")
    parser.add_argument("--name")
    parser.add_argument("--delete-scene", help="Remove an imported scene and its unshared cache, preserving the original ZIP")
    parser.add_argument("--library", default=os.environ.get("AGRIBOT_SCANNED_SCENE_ROOT",
                        str(Path.home() / ".local/share/agribot/scanned_scenes")))
    args = parser.parse_args()
    def cancelled(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, cancelled)
    try:
        request = json.load(sys.stdin) if args.request_stdin else {
            "operation": "delete" if args.delete_scene else "import", "id": args.delete_scene,
            "archive": args.archive, "name": args.name}
        operation = request.get("operation", "import")
        if operation == "delete":
            delete_scene(request["id"], args.library)
        elif operation == "import":
            import_scene(request["archive"], request["name"], args.library)
        else:
            raise ValueError("未知场景操作。")
    except KeyboardInterrupt:
        emit("已取消", ok=False, error="场景导入已取消。")
        raise SystemExit(130)
    except Exception as error:
        emit("导入失败", ok=False, error=str(error))
        raise SystemExit(1)


if __name__ == "__main__":
    main()
