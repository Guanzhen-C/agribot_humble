import pytest

from agribot_farm_sim.legacy_sdf import _LegacyGeometryParser, read_harvester_geometry


def test_unquoted_attributes_and_multiline_pose_are_preserved():
    parser = _LegacyGeometryParser()
    parser.feed('<sdf version="1.6"><model name=plant><link name= leaf_1 >'
                '<pose>0 1\n2 0 0 0</pose><visual name="leaf"/></link></model></sdf>')
    parser.close()
    assert not parser.stack
    root = parser.builder.close()
    assert root.find("model/link").attrib["name"] == "leaf_1"
    assert root.findtext("model/link/pose").split() == ["0", "1", "2", "0", "0", "0"]
    assert root.find("model/link/visual").attrib["name"] == "leaf"


def test_changed_upstream_geometry_is_rejected(tmp_path):
    source = tmp_path / "model.sdf"
    source.write_text('<sdf><model name="other"/></sdf>')
    with pytest.raises(ValueError, match="changed upstream"):
        read_harvester_geometry(source)


def test_unbalanced_legacy_input_is_not_silently_recovered():
    with pytest.raises(ValueError, match="Unbalanced"):
        _LegacyGeometryParser().feed('<sdf><model></sdf>')
