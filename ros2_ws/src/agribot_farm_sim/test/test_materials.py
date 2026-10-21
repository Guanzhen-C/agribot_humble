from agribot_farm_sim.materials import normalize_ambient


def test_ambient_uses_each_materials_diffuse_and_keeps_other_channels(tmp_path):
    path = tmp_path / "vehicle.mtl"
    original = ("# source\nnewmtl rubber\nKa 1 1 1\nKd .03 .04 .05\nKs .1 .1 .1\n"
                "newmtl paint\nKa 1 1 1\nKd .2 .6 .3\nd 1\n")
    path.write_text(original)
    normalize_ambient(path)
    expected = original.replace("Ka 1 1 1\nKd .03", "Ka .03 .04 .05\nKd .03")
    expected = expected.replace("Ka 1 1 1\nKd .2", "Ka .2 .6 .3\nKd .2")
    assert path.read_text() == expected
    normalize_ambient(path)
    assert path.read_text() == expected
