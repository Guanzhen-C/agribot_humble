"""Read-only scene summaries for the assistant, without exposing scan assets."""
import hashlib
import json
import math
import os
from pathlib import Path
import re


SCENE_TOOL = {
    'type': 'function',
    'function': {
        'name': 'list_scenes',
        'description': '查询当前实际可选的场景，区分内置默认场景和用户导入场景，返回各类数量、名称和可推荐方案。目录尚未加载或版本变化时调用；knowledge.sceneCatalogStatus为unchanged时直接复用已有目录，不重复查询。只读，不导入、选择或启动场景。',
        'parameters': {'type': 'object', 'properties': {}, 'required': [], 'additionalProperties': False},
    },
}


def fixed_profile_summary(item, root, digest):
    if not item.get('simulationProfile'):
        return None
    path = Path(item['simulationProfile']).resolve()
    if root not in path.parents:
        raise ValueError('Fixed profile is outside the imported-scene library')
    data = path.read_bytes()
    profile = json.loads(data)
    if (profile.get('id') != 'fuxi_farm_104' or profile.get('route_mode') != 'smac_preset_waypoints'
            or profile.get('vehicle_id') != 'unity_car3'
            or profile.get('algorithm_id') != 'stvl_off_navsat_smac_hybrid_rpp'):
        raise ValueError('Unknown fixed scene recipe')
    baseline = (path.parent / 'baseline_result.json').read_bytes()
    proof = json.loads(baseline)
    route_path = (path.parent / profile['template'] / 'saved_route.json').resolve()
    if path.parent not in route_path.parents:
        raise ValueError('Fixed route is outside the profile')
    route_bytes = route_path.read_bytes()
    route = json.loads(route_bytes)
    if (not proof.get('passed') or proof.get('diagnostic_only') or proof.get('anchor_count') != 104
            or proof.get('controller_plugin') != 'agribot_vehicle_description::PositionOnlyRegulatedPurePursuitController'
            or hashlib.sha256(route_bytes).hexdigest() != profile.get('route_sha256')
            or len(route.get('manual_poses', [])) != 104):
        raise ValueError('Fixed scene evidence is incomplete')
    digest.update(data)
    digest.update(baseline)
    digest.update(route_bytes)
    return dict(id='fuxi_farm_104', car='Car3', implement='none',
        components=dict(perception='stvl', vision='off', localization='navsat', planner='smac_hybrid', controller='rpp'),
        routeMode='smac_preset_waypoints', waypointCount=104, verification='原104点路线完整物理仿真通过',
        mapPolicy='与真机二维底图及原Smac参数一致，启动时重新规划并逐点校验轨迹')


def imported_scenes():
    root = Path(os.environ.get('AGRIBOT_SCANNED_SCENE_ROOT',
                str(Path.home() / '.local/share/agribot/scanned_scenes'))).expanduser().resolve()
    path = root / 'catalog.json'
    try:
        catalog = json.loads(path.read_text())
    except (OSError, ValueError):
        return []
    if not isinstance(catalog, dict) or catalog.get('schemaVersion') != '1.0' or not isinstance(catalog.get('scenarios'), list):
        return []
    result, known = [], set()
    for item in catalog['scenarios']:
        try:
            identifier, name = item['id'], item['name']
            if (item.get('imported') is not True or not isinstance(identifier, str)
                    or not re.fullmatch(r'scan_[A-Za-z0-9_-]+', identifier) or identifier in known
                    or not isinstance(name, str) or not 1 <= len(name.strip()) <= 48
                    or any(ord(c) < 32 for c in name)):
                continue
            manifest = Path(item['sceneManifest']).resolve()
            if root not in manifest.parents:
                continue
            bundle = json.loads(manifest.read_text())
            spawn = bundle['spawn']
            if (bundle.get('schema_version') != 1 or not isinstance(spawn, list) or len(spawn) != 3
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in spawn)):
                continue
            digest = hashlib.sha256(manifest.read_bytes())
            for key in ('world', 'map', 'waypoints'):
                asset = (manifest.parent / bundle[key]).resolve()
                if manifest.parent not in asset.parents or not asset.is_file():
                    raise ValueError('Scene bundle is incomplete')
                digest.update(asset.read_bytes())
            digest.update(json.dumps([identifier, name.strip()], ensure_ascii=False).encode())
            profile = fixed_profile_summary(item, root, digest)
            entry = dict(id=identifier, name=name.strip(), tasks=['扫描场景'],
                imported=True, source='LCC2', sceneConfigSha256=digest.hexdigest(),
                usageDescription='', environmentUnderstanding='未进行场景语义分析',
                gnss='仿真生成的GNSS，不代表扫描地点真实卫星信号可用')
            if profile:
                entry.update(simulationProfile=profile, tasks=['104个预设经停点', '差速履带仿真'],
                    usageDescription='使用真机同二维底图和104个原始经停点，由现有Smac生成轨迹，KF-GINS定位、RPP跟踪。')
            result.append(entry)
            known.add(identifier)
        except (OSError, ValueError, TypeError, KeyError):
            continue
    return result


def list_scenes(knowledge):
    scenes = knowledge['scenes']
    builtin = [s for s in scenes if not s.get('imported')]
    imported = [s for s in scenes if s.get('imported')]
    imported_ids = {s['id'] for s in imported}
    choices = [c for c in knowledge['choices'] if c['id'].split('|')[1] in imported_ids]
    policy_ids = {c['policy'] for c in choices}
    return dict(ok=True, counts=dict(builtin=len(builtin), imported=len(imported), total=len(scenes)),
                builtin_scenes=builtin, imported_scenes=imported,
                choices=choices, policies={k: v for k, v in knowledge['policies'].items() if k in policy_ids},
                limitation='可导入、可选择不等于全部车辆路线已经验证；未知用途和通行条件需结合用户说明。')
