#!/usr/bin/env python3
"""Replace a vehicle family's Unity display geometry, not its simulation physics.

Run in Blender after convert_step/prepare_tractor with decimation disabled.
The canonical simulation assets and configuration JSON files are left untouched.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import struct
import sys

import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def records(path):
    result = []
    with path.open('rb') as stream:
        if stream.read(4) != b'AFM1':
            raise ValueError('Unexpected preview format')
        count, = struct.unpack('<I', stream.read(4))
        if not 0 < count <= 500:
            raise ValueError('Invalid preview group count')
        for _ in range(count):
            length, = struct.unpack('<I', stream.read(4))
            if not 0 < length <= 256:
                raise ValueError('Invalid preview link name')
            name = stream.read(length)
            header = stream.read(20)
            triangles = struct.unpack('<4fI', header)[4]
            if not 0 < triangles <= 300000:
                raise ValueError('Invalid preview chunk')
            data = stream.read(triangles * 36)
            if len(data) != triangles * 36:
                raise ValueError('Invalid preview chunk')
            result.append((name, header, data))
        if stream.read(1):
            raise ValueError('Trailing preview bytes')
    return result


def save(path, parts):
    low, high = np.full(3, np.inf), np.full(3, -np.inf)
    triangles = 0
    with path.open('wb') as stream:
        stream.write(b'AFM1' + struct.pack('<I', len(parts)))
        for name, header, data in parts:
            stream.write(struct.pack('<I', len(name)) + name + header + data)
            points = np.frombuffer(data, dtype='<f4').reshape(-1, 3)
            low, high = np.minimum(low, points.min(axis=0)), np.maximum(high, points.max(axis=0))
            triangles += len(points) // 3
    return dict(meshGroups=len(parts), triangles=triangles, minimumRos=low.tolist(), maximumRos=high.tolist(),
                previewSha256=digest(path))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--unity-project', type=Path, required=True)
    parser.add_argument('--workspace-src', type=Path, required=True)
    parser.add_argument('--base-vehicle', required=True)
    parser.add_argument('--normalized', type=Path, required=True)
    parser.add_argument('--mesh-info', type=Path, required=True)
    parser.add_argument('--backup', type=Path, required=True)
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    if args.backup.exists():
        raise ValueError('Use a new backup directory')
    workspace = args.workspace_src.resolve()
    sys.path[:0] = [str(Path(__file__).parent), str(workspace / 'agribot_vehicle_config_tools')]
    from prepare_unity_farm import preview
    from agribot_vehicle_config_tools.generator import _split_integrated_visual_obj
    info = json.loads(args.mesh_info.read_text())
    report = json.loads((args.normalized / 'cad_import.json').read_text())
    if info['sourceSha256'] != report['sourceSha256'] or report['sourceFaces'] != report['visualFaces']:
        raise ValueError('The display source must preserve every original tessellation face')
    mesh_root = args.normalized / 'preview_meshes'
    _split_integrated_visual_obj(args.normalized / 'vehicle_visual.obj', mesh_root)
    shutil.copy2(args.normalized / 'vehicle_visual.mtl', mesh_root / 'vehicle_visual.mtl')
    catalog = json.loads((args.unity_project / 'Assets/AgribotConfig/Resources/farm_vehicle_presets.json').read_text())
    family = [entry['id'] for entry in catalog['presets'] if entry.get('baseVehicleId') == args.base_vehicle]
    args.backup.mkdir(parents=True)
    staging = args.backup / 'staging'
    staging.mkdir()
    base_bundle = workspace / 'agribot_vehicle_description/generated' / args.base_vehicle
    preview(base_bundle, workspace, staging / 'base.bytes', mesh_root)
    host = records(staging / 'base.bytes')
    host_names = {part[0] for part in host}
    original_base = json.loads((base_bundle / 'vehicle_config.json').read_text())
    pending = []
    for vehicle in family:
        raw = args.unity_project / 'Assets/StreamingAssets/LocalFarmExports' / vehicle
        display = args.unity_project / 'Assets/AgribotConfig/Resources/LocalFarmAssets' / vehicle / 'preview.bytes'
        config = json.loads((raw / 'vehicle_config.json').read_text())
        if config['sensors'] != original_base['sensors'] or config['geometry']['wheelCenters'] != original_base['geometry']['wheelCenters']:
            raise ValueError('Family mount/axle poses do not match: ' + vehicle)
        before_hash = digest(raw / 'vehicle_config.json')
        prior = records(display)
        if not host_names.issubset({part[0] for part in prior}):
            raise ValueError('Family body links do not match: ' + vehicle)
        parts = host + [part for part in prior if part[0] not in host_names]
        target = staging / (vehicle + '.bytes')
        stats = save(target, parts)
        manifest_path = raw / 'asset_manifest.json'
        manifest = json.loads(manifest_path.read_text())
        backup = args.backup / vehicle
        backup.mkdir()
        shutil.copy2(display, backup / 'preview.bytes')
        shutil.copy2(manifest_path, backup / 'asset_manifest.json')
        manifest.update(stats)
        manifest['previewGeometry'] = dict(method='original_cad_no_decimation', sourceStepSha256=info['sourceSha256'],
            linearDeflectionMm=info['linearDeflectionMm'], angularDeflectionRad=info['angularDeflectionRad'],
            sourceFaces=report['sourceFaces'], displaySourceFaces=report['visualFaces'],
            decimationApplied=False, simulationAssetsUnchanged=True, canonicalConfigSha256=before_hash)
        pending.append((display, target, manifest_path, manifest, before_hash, raw))
    try:
        for display, target, manifest_path, manifest, before_hash, raw in pending:
            shutil.copy2(target, display)
            manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
            if digest(raw / 'vehicle_config.json') != before_hash:
                raise RuntimeError('Canonical configuration changed unexpectedly')
            print(json.dumps(dict(vehicle=manifest['vehicleId'], triangles=manifest['triangles'],
                groups=manifest['meshGroups'], canonicalConfigUnchanged=True)), flush=True)
    except Exception:
        for display, _, manifest_path, manifest, _, _ in pending:
            backup = args.backup / manifest['vehicleId']
            shutil.copy2(backup / 'preview.bytes', display)
            shutil.copy2(backup / 'asset_manifest.json', manifest_path)
        raise


if __name__ == '__main__':
    main()
