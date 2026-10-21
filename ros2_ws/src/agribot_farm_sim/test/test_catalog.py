from zipfile import ZipFile

from agribot_farm_sim.catalog import build_catalog, category_hint
from agribot_farm_sim.inventory import inventory


def test_catalog_preserves_native_and_unknown_resources(tmp_path):
    for name, member in [("tractor.ZIP", "assembly.iam"), ("unknown.zip", "part.step")]:
        with ZipFile(tmp_path / name, "w") as archive:
            archive.writestr(member, b"geometry")
    result = build_catalog(inventory(tmp_path))
    entries = {entry["name"]: entry for entry in result["entries"]}
    assert entries["tractor"]["nativeFormats"] == ["iam"]
    assert entries["tractor"]["intakeStatus"] == "native_export_required"
    assert entries["unknown"]["categoryHint"] == "unclassified"
    assert entries["unknown"]["intakeStatus"] == "neutral_geometry_pending"
    assert all(not entry["runtimeSelectable"] for entry in result["entries"])
    assert result["summary"]["hashedUniqueGeometryFiles"] is None


def test_exact_duplicates_are_recorded_without_discarding_variants(tmp_path):
    for name, content in [("one.zip", b"same"), ("two.zip", b"same"), ("three.zip", b"different")]:
        with ZipFile(tmp_path / name, "w") as archive:
            archive.writestr("model.step", content)
    result = build_catalog(inventory(tmp_path), hash_neutral=True)
    assert len(result["entries"]) == 3
    assert result["summary"]["hashedUniqueGeometryFiles"] == 2
    assert len(result["duplicateGeometryGroups"]) == 1
    assert len(result["duplicateGeometryGroups"][0]["members"]) == 2
    assert all(entry["reviewRequired"] for entry in result["entries"])


def test_category_is_not_an_automatic_compatibility_decision():
    assert category_hint("tractor trailer")[0] == "trailer"
    assert category_hint("spray-drone")[0] == "aerial_vehicle"
    assert category_hint("something")[0] == "unclassified"
