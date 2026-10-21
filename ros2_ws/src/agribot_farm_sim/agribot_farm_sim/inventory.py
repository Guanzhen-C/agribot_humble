"""Index user-provided CAD archives without importing them into either repository."""

from collections import Counter
import hashlib
from pathlib import Path
import re
from zipfile import BadZipFile, ZipFile


CAD_FORMATS = {".step", ".stp", ".iges", ".igs", ".stl", ".obj", ".fbx"}
NATIVE_FORMATS = {".sldprt", ".sldasm", ".prt", ".asm", ".x_t", ".x_b", ".catpart", ".catproduct",
                  ".iam", ".ipt", ".par", ".3dm", ".sat", ".3mf"}


def file_format(name):
    versioned = re.search(r"\.(prt|asm)\.\d+$", name, re.IGNORECASE)
    return "." + versioned.group(1).lower() if versioned else Path(name).suffix.lower()


def display_name(info):
    if not info.flag_bits & 0x800:
        try:
            return info.filename.encode("cp437").decode("gb18030")
        except (UnicodeError, LookupError):
            pass
    return info.filename


def inventory(root):
    root = Path(root).resolve(strict=True)
    archives = []
    for path in sorted(p for p in root.rglob("*") if p.is_file() and p.suffix.lower() == ".zip"):
        relative = path.relative_to(root).as_posix()
        entry = {"id": hashlib.sha256(relative.encode()).hexdigest()[:16],
                 "archive": relative, "compressedBytes": path.stat().st_size,
                 "source": "user_supplied", "redistributionLicense": "unverified",
                 "status": "inventory_only", "files": [], "nestedArchives": []}
        try:
            with ZipFile(path) as archive:
                formats = Counter()
                for info in archive.infolist():
                    suffix = file_format(info.filename)
                    formats[suffix] += 1
                    if suffix in (".zip", ".rar", ".7z"):
                        entry["nestedArchives"].append({"member": info.filename,
                                                        "name": display_name(info), "bytes": info.file_size})
                    if suffix in CAD_FORMATS | NATIVE_FORMATS:
                        entry["files"].append({"member": info.filename, "name": display_name(info),
                                               "format": suffix.lstrip("."),
                                               "bytes": info.file_size, "crc32": info.CRC})
                entry["formats"] = dict(sorted(formats.items()))
                entry["convertibleFiles"] = sum(formats[suffix] for suffix in CAD_FORMATS)
        except (BadZipFile, OSError) as exc:
            entry["error"] = str(exc)
        archives.append(entry)
    return {"schemaVersion": "1.0", "sourceRoot": str(root), "archives": archives,
            "archiveCount": len(archives),
            "convertibleArchiveCount": sum(bool(item.get("convertibleFiles")) for item in archives)}


def extract_member(root, archive_name, member, destination):
    """Extract one chosen CAD member to an explicit destination, never its embedded path."""
    root = Path(root).resolve(strict=True)
    path = (root / archive_name).resolve(strict=True)
    if not path.is_relative_to(root):
        raise ValueError("Archive must be inside the source library")
    with ZipFile(path) as archive:
        info = archive.getinfo(member)
        if Path(member).suffix.lower() not in CAD_FORMATS:
            raise ValueError("Only neutral CAD or mesh members can be extracted")
        if info.file_size > 1024 ** 3:
            raise ValueError("Selected CAD file exceeds the 1 GiB import limit")
        destination = Path(destination)
        if destination.exists():
            raise FileExistsError(destination)
        destination.parent.mkdir(parents=True, exist_ok=True)
        import shutil
        with archive.open(info) as source, destination.open("xb") as target:
            shutil.copyfileobj(source, target)
    return destination
