import argparse
import json
from pathlib import Path

from .inventory import extract_member, inventory


def main():
    parser = argparse.ArgumentParser(description="Agricultural simulation resources")
    commands = parser.add_subparsers(dest="command", required=True)
    scan = commands.add_parser("inventory")
    scan.add_argument("root", type=Path)
    scan.add_argument("--output", type=Path, required=True)
    catalog = commands.add_parser("catalog", help="index all archives, without declaring them simulation-ready")
    catalog.add_argument("root", type=Path)
    catalog.add_argument("--output", type=Path, required=True)
    catalog.add_argument("--hash-neutral", action="store_true", help="verify exact geometry duplicates by SHA-256")
    extract = commands.add_parser("extract")
    extract.add_argument("root", type=Path)
    extract.add_argument("archive")
    extract.add_argument("member")
    extract.add_argument("--output", type=Path, required=True)
    mount = commands.add_parser("mount", help="prepare a local nominal tractor/implement export")
    mount.add_argument("vehicle", type=Path)
    mount.add_argument("asset", type=Path)
    mount.add_argument("--output", type=Path, required=True)
    mount.add_argument("--vehicle-id", required=True)
    args = parser.parse_args()
    if args.command == "catalog":
        from .catalog import build_catalog
        result = build_catalog(inventory(args.root), hash_neutral=args.hash_neutral)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result["summary"], ensure_ascii=False))
    elif args.command == "inventory":
        result = inventory(args.root)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"Archives: {result['archiveCount']}; neutral CAD/mesh archives: {result['convertibleArchiveCount']}")
    elif args.command == "extract":
        print(extract_member(args.root, args.archive, args.member, args.output))
    else:
        from .mounts import prepare_mount
        print(prepare_mount(args.vehicle, args.asset, args.output, args.vehicle_id))


if __name__ == "__main__":
    main()
