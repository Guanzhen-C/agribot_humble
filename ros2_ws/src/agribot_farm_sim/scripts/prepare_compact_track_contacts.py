#!/usr/bin/env python3
"""Prepare a new Car3 export aligned to the current model's idler centers."""

import argparse
import json
from pathlib import Path
import shutil

from agribot_vehicle_config_tools.model import validate
from agribot_vehicle_config_tools.track_contacts import align_car3_contacts


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a new output directory; preserve the old contact model")
    config = align_car3_contacts(json.loads((args.source / "vehicle_config.json").read_text()), args.source)
    validate(config)
    shutil.copytree(args.source, args.output)
    (args.output / "vehicle_config.json").write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(config["metadata"]["simulationTrackContactGeometry"], indent=2))


if __name__ == "__main__":
    main()
