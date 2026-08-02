import importlib.util
import struct
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).parents[1]
SCRIPT_PATH = PACKAGE_ROOT / "scripts" / "pcd_to_nav2_map.py"
SPEC = importlib.util.spec_from_file_location("pcd_to_nav2_map", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def write_binary_pcd(path, points):
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    )
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        for point in points:
            stream.write(struct.pack("<ffff", *point))


def test_binary_pcd_height_projection_and_nav2_output(tmp_path):
    pcd_path = tmp_path / "map.pcd"
    output_base = tmp_path / "projected"
    write_binary_pcd(
        pcd_path,
        [
            (0.0, 0.0, 0.05, 1.0),
            (0.0, 0.0, 0.20, 1.0),
            (1.0, 1.0, 1.20, 1.0),
            (2.0, 2.0, 1.21, 1.0),
        ],
    )

    points = MODULE.read_pcd_xyz(pcd_path)
    cells, origin, selected_count = MODULE.project_occupancy(
        points, resolution=0.5, min_z=0.10, max_z=1.20, padding=0.0,
        dilation=0.0
    )
    pgm_path, yaml_path = MODULE.write_nav2_map(
        cells, origin, output_base, resolution=0.5
    )

    assert selected_count == 2
    assert cells.shape == (3, 3)
    assert np.count_nonzero(cells == 0) == 2
    assert np.allclose(origin, [0.0, 0.0])
    assert pgm_path.read_bytes().startswith(b"P5\n3 3\n255\n")
    assert "image: projected.pgm" in yaml_path.read_text()
    assert "resolution: 0.5" in yaml_path.read_text()


def test_mrpt_map_conversion_uses_official_txt2mm(tmp_path):
    converter = tmp_path / "txt2mm"
    converter.write_text(
        "#!/bin/sh\n"
        "while [ \"$#\" -gt 0 ]; do\n"
        "  if [ \"$1\" = \"--output\" ]; then shift; output=$1; fi\n"
        "  shift\n"
        "done\n"
        "printf metric-map > \"$output\"\n",
        encoding="ascii",
    )
    converter.chmod(0o755)

    points = np.array(
        [[0.0, 1.0, 2.0], [np.nan, 0.0, 0.0], [3.0, 4.0, 5.0]]
    )
    mm_path = MODULE.write_mrpt_map(
        points, tmp_path / "mapped", converter=str(converter)
    )

    assert mm_path == tmp_path / "mapped.mm"
    assert mm_path.read_bytes() == b"metric-map"
    assert list(tmp_path.glob("*.xyz")) == []
