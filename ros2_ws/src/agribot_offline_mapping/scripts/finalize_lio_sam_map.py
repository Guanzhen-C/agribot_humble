#!/usr/bin/env python3

import argparse
from pathlib import Path
import shutil
import subprocess


def copy_global_map(source_directory: Path, map_base: Path) -> Path:
    source = source_directory / "GlobalMap.pcd"
    if not source.is_file():
        raise FileNotFoundError(f"LIO-SAM GlobalMap.pcd not found: {source}")
    map_base.parent.mkdir(parents=True, exist_ok=True)
    destination = map_base.with_suffix(".pcd")
    temporary = destination.with_suffix(".pcd.tmp")
    shutil.copyfile(source, temporary)
    temporary.replace(destination)
    return destination


def project_nav2_map(
    pcd_path: Path,
    map_base: Path,
    resolution: float,
    minimum_z: float,
    maximum_z: float,
    padding: float,
    dilation: float,
) -> None:
    subprocess.run(
        [
            "ros2",
            "run",
            "agribot_hardware_bringup",
            "pcd_to_nav2_map.py",
            str(pcd_path),
            str(map_base),
            "--resolution",
            str(resolution),
            "--min-z",
            str(minimum_z),
            "--max-z",
            str(maximum_z),
            "--padding",
            str(padding),
            "--dilation",
            str(dilation),
        ],
        check=True,
    )


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Copy the optimized LIO-SAM PCD and generate its Nav2 map"
    )
    parser.add_argument("lio_sam_map_directory", type=Path)
    parser.add_argument("map_base", type=Path)
    parser.add_argument("--resolution", type=float, default=0.05)
    # LIO-SAM's map origin is the first lidar pose. This is the calibrated
    # physical band from the C16 optical center to 1 m above the ground.
    parser.add_argument("--min-z", type=float, default=0.0)
    parser.add_argument("--max-z", type=float, default=0.6395)
    parser.add_argument("--padding", type=float, default=1.0)
    parser.add_argument("--dilation", type=float, default=0.05)
    parser.add_argument("--skip-projection", action="store_true")
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    if arguments.resolution <= 0.0 or arguments.max_z <= arguments.min_z:
        raise SystemExit("resolution must be positive and max-z must exceed min-z")
    if arguments.padding < 0.0 or arguments.dilation < 0.0:
        raise SystemExit("padding and dilation must be non-negative")
    pcd_path = copy_global_map(
        arguments.lio_sam_map_directory, arguments.map_base
    )
    if not arguments.skip_projection:
        project_nav2_map(
            pcd_path,
            arguments.map_base,
            arguments.resolution,
            arguments.min_z,
            arguments.max_z,
            arguments.padding,
            arguments.dilation,
        )
    print(f"Finalized optimized map: {pcd_path}")


if __name__ == "__main__":
    main()
