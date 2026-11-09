"""Scenario-local display variants; never recolour shared vehicle/sensor exports."""
import hashlib
import json
from pathlib import Path
import shutil


def grey_chassis_materials(text):
    result, paint, count = [], False, 0
    for line in text.splitlines(keepends=True):
        fields = line.split()
        if fields and fields[0] == 'newmtl':
            name = ' '.join(fields[1:])
            paint = name.startswith('LvDai_') and 'LightCover' not in name
            count += int(paint)
        if paint and fields:
            values = dict(Ka='.24 .25 .26', Kd='.32 .33 .34',
                          Ks='.18 .18 .18', Ns='60')
            if fields[0] in values:
                line = f'{fields[0]} {values[fields[0]]}\n'
        result.append(line)
    if not count:
        raise ValueError('Expected tracked chassis materials were not found')
    return ''.join(result), count


def prepare_grey_chassis(generated, directory):
    source = Path(generated)/'models/vehicle_visual.obj'
    material = source.with_suffix('.mtl')
    destination = Path(directory)/'assets/chassis'
    destination.mkdir(parents=True, exist_ok=True)
    mesh = destination/source.name
    shutil.copy2(source, mesh)
    text, changed = grey_chassis_materials(material.read_text())
    (destination/material.name).write_text(text)
    report = dict(source=str(source.resolve()), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  result_sha256=hashlib.sha256(mesh.read_bytes()).hexdigest(), changed_materials=changed,
                  scope='Display only; chassis geometry unchanged, lights/tracks/wheel hubs/sensors retained')
    assert report['source_sha256'] == report['result_sha256']
    (destination/'provenance.json').write_text(json.dumps(report, indent=2)+'\n')
    return mesh.resolve()
