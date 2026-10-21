#!/usr/bin/env python3
"""Sequential, resumable CAD tessellation. It does not create drivable vehicles."""

import argparse
import hashlib
import json
from pathlib import Path
import resource
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from agribot_farm_sim.inventory import extract_member


def digest(path):
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(block)
    return sha.hexdigest()


def choose_geometry(entry):
    candidates = [item for item in entry["geometryCandidates"] if item["format"] in ("step", "stp")]
    if not candidates:
        candidates = [item for item in entry["geometryCandidates"] if item["format"] in ("iges", "igs")]
    if len(candidates) != 1:
        raise ValueError("requires exactly one STEP (preferred) or IGES assembly; select/assemble ambiguous or mesh files manually")
    if candidates[0].get("hashStatus") != "verified":
        raise ValueError("run catalog --hash-neutral before converting")
    return candidates[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("catalog", type=Path)
    parser.add_argument("--id", action="append", required=True, dest="ids")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=600)
    parser.add_argument("--memory-gib", type=int, default=8)
    args = parser.parse_args()
    if args.timeout <= 0 or args.memory_gib < 1:
        parser.error("timeout and memory limit must be positive")
    catalog = json.loads(args.catalog.read_text())
    lookup = {item["id"]: item for item in catalog["entries"]}
    if any(key not in lookup or Path(key).name != key or key in (".", "..") for key in args.ids):
        parser.error("unknown or invalid catalog ID")
    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.output / "batch_report.json"
    records = {item["id"]: item for item in json.loads(report_path.read_text())} if report_path.exists() else {}
    failures = 0

    def memory_limit():
        size = args.memory_gib * 1024 ** 3
        resource.setrlimit(resource.RLIMIT_AS, (size, size))

    for key in dict.fromkeys(args.ids):
        entry = lookup[key]
        record = {"id": key, "name": entry["name"], "status": "pending", "runtimeSelectable": False}
        started = time.monotonic()
        try:
            selected = choose_geometry(entry)
            directory = args.output / key / selected["sha256"]
            directory.mkdir(parents=True, exist_ok=True)
            source = directory / ("source." + selected["format"])
            if not source.exists():
                extract_member(catalog["sourceRoot"], entry["archive"], selected["member"], source)
            if digest(source) != selected["sha256"]:
                raise ValueError("source hash mismatch")
            report = directory / "mesh/mesh_info.json"
            if report.exists() and (directory / "mesh/visual.obj").is_file():
                info = json.loads(report.read_text())
                if info["sourceSha256"] != selected["sha256"]:
                    raise ValueError("existing converted mesh hash mismatch")
                record["status"] = "cached_geometry_only"
            else:
                with (directory / "conversion.log").open("w") as log:
                    subprocess.run(["/usr/bin/python3", str(Path(__file__).with_name("convert_step.py")),
                                    str(source), str(directory / "mesh")], check=True,
                                   stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout,
                                   preexec_fn=memory_limit)
                info = json.loads(report.read_text())
                if info["faces"] <= 0:
                    raise ValueError("converted mesh is empty")
                record["status"] = "converted_geometry_only"
            record.update(directory=str(directory.resolve()), faces=info["faces"], sizeM=info["sizeM"])
        except (OSError, ValueError, KeyError, subprocess.SubprocessError) as exc:
            record.update(status="needs_review", reason=str(exc))
            failures += 1
        record["wallSeconds"] = round(time.monotonic() - started, 3)
        records[key] = record
        temporary = report_path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(list(records.values()), ensure_ascii=False, indent=2) + "\n")
        temporary.replace(report_path)
        print(json.dumps(record, ensure_ascii=False), flush=True)
    return int(failures > 0)


if __name__ == "__main__":
    sys.exit(main())
