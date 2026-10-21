"""Build a reviewable CAD intake queue; this is not a runtime vehicle registry."""

from collections import Counter
from pathlib import Path
import hashlib
from zipfile import ZipFile

from .inventory import CAD_FORMATS


RULES = (
    ("aerial_vehicle", ("无人机", "drone", "helicopter")),
    ("trailer", ("拖车", "trailer", "dumper", "bale loader", "coulter-cart")),
    ("tractor", ("拖拉机", "tractor", "traktoren", "john_deere", "mccormik")),
    ("harvester", ("收割", "收获", "harvest", "cr1090")),
    ("seeding_implement", ("播种", "移栽", "插秧", "种植", "seeder", "sower", "planter", "planting")),
    ("tillage_implement", ("犁", "耙", "耕", "开沟", "压土", "碾压", "plow", "plough", "harrow", "cultivator", "tiller", "roller")),
    ("mowing_implement", ("割草", "除草", "修剪", "mower", "milling", "forest-chain")),
    ("spraying_implement", ("喷", "spray")),
    ("stationary_equipment", ("清洗", "脱粒", "剥皮", "粉碎", "加工", "烘干", "筛选", "种植架", "dryer", "silo", "feeder", "conveyor", "thresher", "shredder", "sheller", "separator", "disk-mill", "debarker")),
)


def category_hint(name):
    """Filename hints only: do not infer drive type, mounting or operating limits."""
    lowered = name.casefold()
    for category, keywords in RULES:
        matches = [word for word in keywords if word in lowered]
        if matches:
            return category, matches
    return "unclassified", []


def build_catalog(inventory, *, hash_neutral=False):
    root = Path(inventory["sourceRoot"]).resolve(strict=True)
    entries, hashes = [], {}
    for archive in inventory["archives"]:
        category, keywords = category_hint(Path(archive["archive"]).stem)
        candidates = [dict(item) for item in archive["files"]
                      if "." + item["format"] in CAD_FORMATS]
        entry = {
            "id": archive["id"], "name": Path(archive["archive"]).stem,
            "archive": archive["archive"], "categoryHint": category,
            "categoryEvidence": {"method": "filename_keywords", "matches": keywords},
            "reviewRequired": True, "runtimeSelectable": False,
            "redistributionLicense": archive["redistributionLicense"],
            "intakeStatus": "neutral_geometry_pending" if candidates else "native_export_required",
            "nativeFormats": sorted({item["format"] for item in archive["files"]
                                     if "." + item["format"] not in CAD_FORMATS}),
            "geometryCandidates": candidates,
            "nestedArchives": archive.get("nestedArchives", []),
            "validation": {"geometry": "not_run", "kinematics": "not_run",
                           "mounting": "not_run", "navigation": "not_run"},
        }
        if archive.get("error"):
            entry.update(intakeStatus="archive_error", error=archive["error"])
        elif not candidates and not entry["nativeFormats"]:
            entry["intakeStatus"] = "nested_archive_review" if entry["nestedArchives"] else "geometry_not_found"
        elif hash_neutral and candidates:
            path = (root / archive["archive"]).resolve(strict=True)
            if not path.is_relative_to(root):
                raise ValueError("Archive must be inside the source library")
            with ZipFile(path) as source:
                for candidate in candidates:
                    info = source.getinfo(candidate["member"])
                    if info.file_size > 1024 ** 3:
                        candidate["hashStatus"] = "skipped_over_1GiB"
                        continue
                    digest = hashlib.sha256()
                    with source.open(info) as stream:
                        for block in iter(lambda: stream.read(1024 * 1024), b""):
                            digest.update(block)
                    sha = digest.hexdigest()
                    candidate.update(sha256=sha, hashStatus="verified")
                    hashes.setdefault(sha, []).append({"id": entry["id"], "member": candidate["member"]})
        entries.append(entry)
    return {"schemaVersion": 1, "kind": "resource_intake_catalog", "sourceRoot": str(root),
            "summary": {"archives": len(entries),
                        "statusCounts": dict(Counter(e["intakeStatus"] for e in entries)),
                        "categoryHintCounts": dict(Counter(e["categoryHint"] for e in entries)),
                        "neutralFiles": sum(len(e["geometryCandidates"]) for e in entries),
                        "hashedUniqueGeometryFiles": len(hashes) if hash_neutral else None},
            "duplicateGeometryGroups": [{"sha256": sha, "members": items}
                                        for sha, items in sorted(hashes.items()) if len(items) > 1],
            "entries": entries}
