import hashlib
import json

import pytest

from agribot_hil_apple.appearance import grey_chassis_materials, prepare_grey_chassis


def test_grey_shell_retains_lights_and_track_materials():
    source = ('newmtl LvDai_WarmWhitePaint_19\nKa .86 .86 .8\nKd .86 .86 .8\nKs 1 1 1\nNs 900\n'
              'newmtl LvDai_BrushedMetal_10\nKd .72 .74 .76\n'
              'newmtl LvDai_FrostedLightCover_8\nKd .85 .88 .86\n'
              'newmtl compact_track_rubber\nKd .045 .052 .058\n'
              'newmtl compact_wheel_warm_white\nKd .72 .74 .70\n')
    result, count = grey_chassis_materials(source)
    assert count == 2
    assert result.count('Kd .32 .33 .34') == 2
    assert 'Kd .85 .88 .86' in result
    assert 'Kd .045 .052 .058' in result
    assert 'Kd .72 .74 .70' in result


def test_display_variant_preserves_geometry_and_shared_sources(tmp_path):
    generated = tmp_path/'original'
    models = generated/'models'
    models.mkdir(parents=True)
    mesh = models/'vehicle_visual.obj'
    mesh.write_text('mtllib vehicle_visual.mtl\nv 1 2 3\nf 1 1 1\n')
    material = mesh.with_suffix('.mtl')
    material.write_text('newmtl LvDai_WarmWhitePaint_19\nKd .86 .86 .8\n')
    before = {p.name: p.read_bytes() for p in models.iterdir()}
    variant = prepare_grey_chassis(generated, tmp_path/'scenario')
    assert variant.read_bytes() == mesh.read_bytes()
    assert {p.name: p.read_bytes() for p in models.iterdir()} == before
    assert variant.with_suffix('.mtl').read_text() != material.read_text()
    report = json.loads((variant.parent/'provenance.json').read_text())
    assert report['result_sha256'] == hashlib.sha256(mesh.read_bytes()).hexdigest()


def test_unknown_chassis_materials_are_not_silently_recoloured():
    with pytest.raises(ValueError):
        grey_chassis_materials('newmtl unknown\nKd 1 1 1\n')
