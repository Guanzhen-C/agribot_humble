"""Command-line entry point for the Unity-to-ROS configuration pipeline."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from agribot_vehicle_config_tools.generator import generate_bundle
from agribot_vehicle_config_tools.model import ConfigError, load_and_validate
from agribot_vehicle_config_tools.verifier import verify_current


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="agribot_vehicle_config",
        description="Validate a Unity vehicle export and generate ROS configuration.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    validate_parser = subparsers.add_parser("validate", help="validate one exported JSON file")
    validate_parser.add_argument("input", type=Path)

    generate_parser = subparsers.add_parser("generate", help="generate a ROS configuration bundle")
    generate_parser.add_argument("input", type=Path)
    generate_parser.add_argument("--output", type=Path, required=True)
    generate_parser.add_argument(
        "--workspace-src",
        type=Path,
        help="ROS workspace src directory used for validated runtime templates",
    )

    import_parser = subparsers.add_parser(
        "import-unity",
        help="validate one Unity export, store its canonical JSON and generate ROS files",
    )
    import_parser.add_argument("export_directory", type=Path)
    import_parser.add_argument("--canonical", type=Path, required=True)
    import_parser.add_argument("--output", type=Path, required=True)
    import_parser.add_argument("--workspace-src", type=Path, required=True)

    verify_parser = subparsers.add_parser(
        "verify-current",
        help="compare an export with the current validated Ackermann files",
    )
    verify_parser.add_argument("input", type=Path)
    verify_parser.add_argument("--workspace-src", type=Path, required=True)
    verify_parser.add_argument("--json-report", type=Path)
    return parser


def _print_warnings(warnings: list[str]) -> None:
    for warning in warnings:
        print(f"WARNING: {warning}", file=sys.stderr)


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    input_path = (
        args.export_directory / "vehicle_config.json"
        if args.command == "import-unity"
        else args.input
    )
    try:
        config, warnings = load_and_validate(input_path)
    except ConfigError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    _print_warnings(warnings)

    if args.command == "validate":
        print(f"OK: {input_path} ({config['vehicleId']}, schema {config['schemaVersion']})")
        return 0

    if args.command == "generate":
        manifest = generate_bundle(
            config,
            input_path,
            args.output,
            workspace_src=args.workspace_src,
        )
        print(
            f"OK: generated {len(manifest['files'])} files in {args.output} "
            f"for {config['vehicleId']}"
        )
        return 0


    if args.command == "import-unity":
        manifest = generate_bundle(
            config,
            input_path,
            args.output,
            workspace_src=args.workspace_src,
        )
        args.canonical.parent.mkdir(parents=True, exist_ok=True)
        args.canonical.write_text(
            json.dumps(config, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(
            f"OK: imported {config['vehicleId']} and generated "
            f"{len(manifest['files'])} files in {args.output}"
        )
        return 0

    results = verify_current(config, args.workspace_src)
    failed = [result for result in results if not result.passed]
    for result in results:
        status = "PASS" if result.passed else "FAIL"
        print(f"{status:4} {result.name} [{result.source}]")
        if not result.passed:
            print(f"     expected: {result.expected}")
            print(f"     actual:   {result.actual}")
    if args.json_report:
        args.json_report.parent.mkdir(parents=True, exist_ok=True)
        args.json_report.write_text(
            json.dumps(
                [result.__dict__ for result in results],
                ensure_ascii=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
    if failed:
        print(f"ERROR: {len(failed)} of {len(results)} checks failed", file=sys.stderr)
        return 1
    print(f"OK: all {len(results)} current Ackermann checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
