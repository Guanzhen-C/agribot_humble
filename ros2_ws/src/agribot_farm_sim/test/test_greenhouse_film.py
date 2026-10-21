import hashlib
import json
from pathlib import Path

import pytest


ASSET = Path(__file__).resolve().parents[1] / "assets/harbot_greenhouse"


def test_plastic_film_material_is_visible_and_translucent():
    metadata = json.loads((ASSET / "source.json").read_text())["plasticFilm"]
    materials = {}
    current = None
    for line in (ASSET / "scene.mtl").read_text().splitlines():
        fields = line.split()
        if not fields or fields[0].startswith("#"):
            continue
        if fields[0] == "newmtl":
            current = materials.setdefault(fields[1], {})
        elif current is not None:
            current[fields[0]] = fields[1:]
    for name in metadata["materials"]:
        material = materials[name]
        assert float(material["d"][0]) == pytest.approx(metadata["opacity"])
        assert .15 < metadata["opacity"] < .5
        assert list(map(float, material["Kd"])) == pytest.approx(metadata["colour"])
        assert material["illum"] == ["2"]
    referenced = {line.split(maxsplit=1)[1].strip()
                  for line in (ASSET / "scene.obj").open() if line.startswith("usemtl ")}
    assert set(metadata["materials"]) <= referenced


@pytest.mark.parametrize("filename,expected", [
    ("scene.obj", "403434408f373cae5053a18505812d5f7c62f9860e33edb660ed8c2affd12e21"),
    ("collision.obj", "ebf8e6619e8384623d3c4586831610dba2893f9c1c5e695c0ed831f5565927e3"),
    ("map.pgm", "4787272dad1b900711e3a42739795fc2e40aad34bd2fbbeb0e19ca893d3bc5ad"),
    ("map.yaml", "4cd3b715cd8ebeeeb224378b05f5374aca85985b063b3a8fb082f24a9c62bd3b"),
])
def test_film_does_not_change_existing_geometry_or_navigation(filename, expected):
    assert hashlib.sha256((ASSET / filename).read_bytes()).hexdigest() == expected
