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


def test_white_cad_rubber_is_black_without_recolouring_hubs(tmp_path):
    path = tmp_path / "vehicle.mtl"
    path.write_text("newmtl 03_细孔橡胶_4\nKd 1 1 1\nKa 1 1 1\n"
                    "newmtl 02_暖白轮毂与饰板_7\nKd .9 .85 .72\n"
                    "newmtl white_paint\nKd 1 1 1\n")
    normalize_ambient(path)
    actual = path.read_text()
    assert "Kd 0.035 0.04 0.045\nKa 0.035 0.04 0.045" in actual
    assert "newmtl 02_暖白轮毂与饰板_7\nKd .9 .85 .72" in actual
    assert "newmtl white_paint\nKd 1 1 1" in actual
    normalize_ambient(path)
    assert path.read_text() == actual
