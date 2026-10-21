import json
from pathlib import Path
import subprocess
import sys

import pytest


def test_optional_surface_conversion_keeps_body_panels_and_existing_solid_indices(tmp_path):
    sys.path.insert(0, "/usr/lib/freecad-python3/lib")
    pytest.importorskip("FreeCAD")
    part = pytest.importorskip("Part")
    cad = part.makeCompound([part.makeBox(100, 100, 100), part.makePlane(400, 300)])
    source = tmp_path/"solid_and_open_panel.step"
    cad.exportStep(str(source))
    script = Path(__file__).parents[1]/"scripts/convert_step.py"
    results = []
    for name, flags in (("solids", []), ("complete", ["--include-open-surfaces"])):
        target = tmp_path/name
        subprocess.run([sys.executable, str(script), str(source), str(target), *flags],
            check=True, capture_output=True, timeout=30)
        results.append(json.loads((target/"mesh_info.json").read_text()))
    old, complete = results
    assert old["includedOpenFaces"] == 0
    assert complete["includedOpenFaces"] == 1
    assert len(complete["components"]) == len(old["components"]) + 1
    assert complete["components"][0] == old["components"][0]
    assert complete["components"][-1]["geometryType"] == "open_surfaces"
    assert complete["faces"] > old["faces"]
