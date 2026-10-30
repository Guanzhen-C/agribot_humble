#!/usr/bin/env python3
"""Create an offline, checksum-verified planning update for an existing RDK install."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_base", type=Path)
    parser.add_argument("route_candidates", type=Path)
    parser.add_argument("evidence", type=Path)
    parser.add_argument("apk", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    mobile = Path(__file__).resolve().parents[1]
    hardware = mobile.parent / "agribot_hardware_bringup"
    base = args.map_base.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if output.exists() or Path(str(output) + ".tar.gz").exists():
        raise ValueError("Refusing to overwrite a release")
    output.mkdir(parents=True)

    def copy(source, target):
        target = output / target
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)

    for directory in ("agribot_mobile_app", "resource", "tools"):
        for path in (mobile / directory).rglob("*"):
            if path.is_file() and "__pycache__" not in path.parts:
                copy(path, Path("code/agribot_mobile_app") / path.relative_to(mobile))
    for name in ("setup.py", "setup.cfg", "package.xml", "README.md"):
        copy(mobile / name, Path("code/agribot_mobile_app") / name)
    for path in (mobile / "web/dist").rglob("*"):
        if path.is_file() and "downloads" not in path.relative_to(mobile / "web/dist").parts:
            copy(path, Path("code/agribot_mobile_app") / path.relative_to(mobile))
    copy(args.apk, Path("code/agribot_mobile_app/web/dist/downloads") / args.apk.name)
    for name in ("nav2_params_differential_planner_only.yaml", "nav2_params_differential_fastlivo_mapped.yaml"):
        copy(hardware / "differential/config" / name, Path("code/agribot_hardware_bringup/differential/config") / name)
    for suffix in (".pgm", ".yaml", "_georeference.yaml"):
        copy(Path(str(base) + suffix), Path("maps") / (base.name + suffix))
    copy(args.route_candidates, Path("maps") / (base.name + "_routes.json"))
    copy(Path(str(base) + "_routes.json"), Path("maps") / (base.name + "_routes_original.json"))
    for path in args.evidence.iterdir():
        if path.is_file():
            copy(path, Path("evidence") / path.name)
    copy(mobile / "tools/apply_planning_update.sh", "apply_planning_update.sh")
    copy(mobile / "tools/planning_deployment.txt", "使用说明.txt")
    manifest = {"map_id": base.name, "version": "0.2.9", "target": "existing RDK X5 ROS 2 Humble workspace",
                "required_existing_pcd_sha256": digest(base.with_suffix(".pcd")),
                "map_pgm_sha256": digest(base.with_suffix(".pgm")),
                "preserves_existing_gateway_config_and_sensor_extrinsics": True,
                "vehicle_motion_is_not_started": True,
                "validation_scope": "Desktop planning/HTTP/UI validation; ARM64 full-route timing and real vehicle test remain for deployment"}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    hashes = [f"{digest(path)}  {path.relative_to(output).as_posix()}" for path in sorted(output.rglob("*")) if path.is_file()]
    (output / "SHA256SUMS").write_text("\n".join(hashes) + "\n", encoding="utf-8")
    archive = Path(str(output) + ".tar.gz")
    with tarfile.open(archive, "w:gz", compresslevel=3) as stream:
        stream.add(output, arcname=output.name)
    print(json.dumps({"folder": str(output), "archive": str(archive), "archive_sha256": digest(archive), "archive_bytes": archive.stat().st_size}, indent=2))


if __name__ == "__main__":
    main()
