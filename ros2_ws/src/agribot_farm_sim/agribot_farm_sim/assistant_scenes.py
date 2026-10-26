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
            result.append(dict(id=identifier, name=name.strip(), tasks=['扫描场景'],
                imported=True, source='LCC2', sceneConfigSha256=digest.hexdigest(),
                usageDescription='', environmentUnderstanding='未进行场景语义分析',
                gnss='仿真生成的GNSS，不代表扫描地点真实卫星信号可用'))
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
