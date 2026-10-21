from zipfile import ZipFile

import pytest

from agribot_farm_sim.inventory import extract_member, file_format, inventory


def test_inventory_distinguishes_native_cad_and_convertible_mesh(tmp_path):
    with ZipFile(tmp_path / "tractor.zip", "w") as archive:
        archive.writestr("tractor.sldasm", "native")
        archive.writestr("tractor.step", "neutral")
    with ZipFile(tmp_path / "native.zip", "w") as archive:
        archive.writestr("mower.sldprt", "native")
    result = inventory(tmp_path)
    assert result["archiveCount"] == 2
    assert result["convertibleArchiveCount"] == 1
    assert all(item["status"] == "inventory_only" for item in result["archives"])


def test_extraction_does_not_trust_embedded_paths(tmp_path):
    library = tmp_path / "library"
    library.mkdir()
    with ZipFile(library / "part.zip", "w") as archive:
        archive.writestr("../../escape.step", "cad")
    target = tmp_path / "selected.step"
    extract_member(library, "part.zip", "../../escape.step", target)
    assert target.read_text() == "cad"
    assert not (tmp_path / "escape.step").exists()
    with pytest.raises(FileExistsError):
        extract_member(library, "part.zip", "../../escape.step", target)
    with pytest.raises(ValueError):
        extract_member(library, "../selected.step", "part.step", tmp_path / "new.step")


def test_proe_versions_and_nested_archives_are_not_silently_lost(tmp_path):
    assert file_format("gear.PRT.13") == ".prt"
    with ZipFile(tmp_path / "nested.ZIP", "w") as archive:
        archive.writestr("assembly.asm.1", b"native")
        archive.writestr("inner.zip", b"placeholder")
    record = inventory(tmp_path)["archives"][0]
    assert record["files"][0]["format"] == "asm"
    assert len(record["nestedArchives"]) == 1
