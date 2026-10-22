"""Natural-language selection over current, user-confirmed simulation recipes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import urllib.error
import urllib.request

from .compatibility import GROUP_ORDER, component_cases, read_catalog, select_algorithm


EXCLUSIONS = {('Car7', 'farmyard'), ('Car9', 'farmyard'), ('Car2', 'strawberry_field')}


class InvalidModelReply(ValueError):
    pass


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def candidates(workspace_src):
    src = Path(workspace_src)
    config_dir = src / 'agribot_farm_sim/config'
    catalog = read_catalog(config_dir / 'compatibility.json')
    cars = {v['car']: v for v in catalog['vehicles']}
    bindings = {v['car'] + '__' + v['implement']: v for v in catalog['bindings']}
    scenes = {v['id']: v for v in json.loads((config_dir / 'scenarios.json').read_text())['scenarios']}
    exclusions = EXCLUSIONS
    configs = {}
    def configuration(binding):
        key = binding['preset']
        if key not in configs:
            path = src / 'agribot_vehicle_description/generated' / key / 'vehicle_config.json'
            configs[key] = (json.loads(path.read_text()), sha(path)) if path.is_file() else (None, None)
        return configs[key]
    found = {}
    for path in sorted(config_dir.glob('manual_confirmations*.json')):
        record = json.loads(path.read_text())
        if record.get('status') != 'PASS' or record.get('evidenceType') != 'user_group_confirmation':
            continue
        trial = Path(record.get('representativeInteractiveTrial', '')) / 'session.json'
        session = json.loads(trial.read_text()) if trial.is_file() else {}
        for entry in record.get('confirmations', []):
            binding = bindings.get(entry.get('binding'))
            scene = entry.get('scenario', record.get('scenario'))
            if not binding or scene not in scenes or binding['implement'] == 'none':
                continue
            car = binding['car']
            if (car, scene) in exclusions or (scene == 'greenhouse' and car not in ('Car0', 'Car3')):
                continue
            vehicle, digest = configuration(binding)
            if not vehicle or digest != entry.get('vehicleConfigSha256') or entry.get('status') != 'PASS':
                continue
            recipe = select_algorithm(catalog, binding, scene)
            algorithm = record['algorithm']
            if algorithm != recipe['algorithm']:
                continue
            # Apply the user's representative-implement policy, not an automatic test claim.
            for target in bindings.values():
                if target['car'] != car or target['implement'] == 'none':
                    continue
                target_config, target_sha = configuration(target)
                if not target_config or select_algorithm(catalog, target, scene)['algorithm'] != algorithm:
                    continue
                explicit = next((e for e in record['confirmations'] if e['binding'] == car + '__' + target['implement']), None)
                # Freeze membership at confirmation time: new implements need a new record.
                if not explicit or explicit.get('status') != 'PASS':
                    continue
                same_geometry = target_config['geometry'] == vehicle['geometry']
                if not same_geometry and not (explicit and explicit.get('requiresIndividualDynamicsReview')):
                    continue
                if explicit and explicit.get('vehicleConfigSha256') != target_sha:
                    continue
                identifier = '|'.join((car, scene, target['implement']))
                individually = (session.get('binding') == car + '__' + target['implement']
                                and session.get('vehicleConfigSha256') == target_sha
                                and session.get('status') == 'USER_REPORTED_SUCCESS')
                found[identifier] = dict(id=identifier, car=car, vehicleName=cars[car]['name'],
                    vehicleType=cars[car]['type'],
                    preset=target['preset'], implement=target['implement'], implementName=target['name'],
                    scene=scene, sceneName=scenes[scene]['name'], algorithm=algorithm,
                    vehicleConfigSha256=target_sha,
                    verification='人工代表试验确认' if individually else '用户分组确认，未逐项重跑',
                    evidence=path.name,
                    dynamicsReview=bool(explicit and explicit.get('requiresIndividualDynamicsReview')),
                    geometry=dict(lengthM=target_config['geometry']['bodyExtents']['frontM']+target_config['geometry']['bodyExtents']['rearM'],
                                  widthM=target_config['geometry']['bodyExtents']['leftM']+target_config['geometry']['bodyExtents']['rightM'],
                                  wheelbaseM=target_config['geometry']['wheelbaseM'],
                                  minTurningRadiusM=target_config['geometry']['minTurningRadiusM']))
    return sorted(found.values(), key=lambda r: r['id'])


def assistant_catalog(workspace_src):
    """Keep complete product inventory separate from recipe evidence and restrictions."""
    src = Path(workspace_src)
    config_dir = src / 'agribot_farm_sim/config'
    catalog = read_catalog(config_dir / 'compatibility.json')
    scenes = json.loads((config_dir / 'scenarios.json').read_text())['scenarios']
    cars = {v['car']: v for v in catalog['vehicles']}
    confirmed = {r['id']: r for r in candidates(src)}
    choices = []
    for binding in catalog['bindings']:
        if not binding.get('navigationRecommendationAllowed', True):
            continue
        path = src / 'agribot_vehicle_description/generated' / binding['preset'] / 'vehicle_config.json'
        if not path.is_file():
            continue
        config = json.loads(path.read_text())
        car = cars[binding['car']]
        extents = config['geometry']['bodyExtents']
        for scene in scenes:
            if scene['id'] not in catalog['sceneGroups'][binding.get('sceneGroup', car['sceneGroup'])]:
                continue
            if ((car['car'], scene['id']) in EXCLUSIONS or
                    (scene['id'] == 'greenhouse' and car['car'] not in ('Car0', 'Car3'))):
                continue
            recipe = select_algorithm(catalog, binding, scene['id'])
            options = {key: list(values) for key, values in catalog['components'].items()}
            if recipe.get('allowedLocalizationModes') is not None:
                options['localization'] = recipe['allowedLocalizationModes']
            identifier = '|'.join((car['car'], scene['id'], binding['implement']))
            choices.append(dict(id=identifier, car=car['car'], vehicleName=car['name'], vehicleType=car['type'],
                preset=binding['preset'], implement=binding['implement'], implementName=binding['name'],
                scene=scene['id'], sceneName=scene['name'], algorithm=recipe['algorithm'],
                vehicleConfigSha256=sha(path), allowedComponents=options, conditions=recipe['conditions'],
                guidance='沿用既有运行方案' if identifier in confirmed else '按适配关系与算法条件提供建议',
                geometry=dict(lengthM=extents['frontM'] + extents['rearM'],
                    widthM=extents['leftM'] + extents['rightM'],
                    wheelbaseM=config['geometry']['wheelbaseM'],
                    minTurningRadiusM=config['geometry']['minTurningRadiusM'])))
    inventory = [dict(car=v['car'], name=v['name'], type=v['type']) for v in catalog['vehicles']]
    counts = {kind: sum(v['type'] == kind for v in inventory) for kind in sorted({v['type'] for v in inventory})}
    policies, policy_ids, compact_choices = {}, {}, []
    for row in choices:
        policy = dict(defaultAlgorithm=row['algorithm'], allowedLocalizationModes=row['allowedComponents']['localization'],
                      conditions=row['conditions'])
        signature = json.dumps(policy, sort_keys=True)
        if signature not in policy_ids:
            policy_ids[signature] = str(len(policy_ids))
            policies[policy_ids[signature]] = policy
        compact_choices.append(dict(id=row['id'], policy=policy_ids[signature],
                                   familiar=row['id'] in confirmed))
    knowledge = dict(inventory=dict(vehicles=inventory, vehicleCount=len(inventory), countsByType=counts),
        components=catalog['components'],
        scenes=[{key: s[key] for key in ('id', 'name', 'tasks')} for s in scenes],
        vehicles={v['car']: dict(name=v['name'], type=v['type'],
            implements=[dict(id=b['implement'], name=b['name']) for b in catalog['bindings'] if b['car'] == v['car']],
            scenes=sorted({r['scene'] for r in choices if r['car'] == v['car']})) for v in catalog['vehicles']},
        exclusions=[dict(car=car, scene=scene) for car, scene in sorted(EXCLUSIONS)],
        policies=policies, choices=compact_choices)
    # Geometry is per mounting, not necessarily the same for every implement.
    knowledge['mountings'] = {r['car'] + '__' + r['implement']: r['geometry'] for r in choices}
    return choices, knowledge


def configuration():
    path = Path(os.environ.get('AGRIBOT_LLM_CONFIG', str(Path.home() / '.config/agribot/llm.json')))
    config = json.loads(path.read_text()) if path.is_file() else {}
    key = os.environ.get('DEEPSEEK_API_KEY') or config.get('api_key')
    if not key:
        raise ValueError('未配置 DeepSeek API Key，请在本机私有配置中设置。')
    base = config.get('base_url', 'https://api.deepseek.com').rstrip('/')
    if base not in ('https://api.deepseek.com', 'https://api.deepseek.com/v1'):
        raise ValueError('当前仅允许官方 DeepSeek HTTPS 接口，避免将凭据发送到其他地址。')
    return key, base, config.get('model', 'deepseek-flash')


def request_model_reply(payload):
    key, base, model = configuration()
    payload['model'] = model
    request = urllib.request.Request(base + '/chat/completions',
        json.dumps(payload, ensure_ascii=False).encode(),
        headers={'Content-Type': 'application/json', 'Authorization': 'Bearer ' + key})
    try:
        with urllib.request.urlopen(request, timeout=35) as response:
            raw = response.read(512_001)
        if len(raw) > 512_000:
            raise InvalidModelReply('模型返回内容过大。')
        choice = json.loads(raw)['choices'][0]
        if choice.get('finish_reason') != 'stop':
            raise InvalidModelReply('模型输出未完整结束，请简洁返回。')
        return json.loads(choice['message']['content'])
    except urllib.error.HTTPError as error:
        raise ValueError(f'DeepSeek接口请求失败（HTTP {error.code}），请检查本机密钥、余额和网络。') from None
    except (urllib.error.URLError, TimeoutError):
        raise ValueError('DeepSeek连接超时或网络不可用，请稍后重试。') from None
    except (KeyError, IndexError, TypeError, json.JSONDecodeError):
        raise InvalidModelReply('模型返回内容无法解析，请严格返回完整JSON。') from None


def public_recipe(row):
    return {k: v for k, v in row.items() if k not in ('verification', 'evidence', 'allowedComponents', 'conditions')}


def validate_reply(reply, rows, context=None):
    mapping = {r['id']: r for r in rows}
    if not isinstance(reply, dict) or not isinstance(reply.get('recommendations'), list):
        raise ValueError('模型返回格式无效，请重试。')
    items = reply['recommendations']
    if len(items) > 3:
        raise ValueError('模型返回过多方案，请重试。')
    result, seen = [], set()
    for item in items:
        identifier = item.get('candidate_id', item.get('id', '')) if isinstance(item, dict) else ''
        if not isinstance(identifier, str):
            raise ValueError('模型返回格式无效，请重试。')
        base, _, variant = identifier.partition('@')
        if not isinstance(item, dict) or base not in mapping:
            raise ValueError('模型返回了不支持或重复的方案，请重试。')
        row = dict(mapping[base])
        components = item.get('components')
        if variant:
            options = row.get('allowedComponents', {})
            resolved = next((dict(zip(GROUP_ORDER, values)) for values in component_cases(dict(components=options))
                             if '_'.join(values) == variant), None) if set(options) == set(GROUP_ORDER) else None
            if resolved is None or (components is not None and components != resolved):
                raise ValueError('推荐算法不符合当前车型与场景限制，请重新生成。')
            components = resolved
        if components is not None:
            allowed = row.get('allowedComponents', {})
            if (not isinstance(components, dict) or set(components) != set(GROUP_ORDER) or
                    any(not isinstance(components[k], str) or components[k] not in allowed.get(k, []) for k in GROUP_ORDER)):
                raise ValueError('推荐算法不符合当前车型与场景限制，请重新生成。')
            algorithm = '_'.join(components[k] for k in GROUP_ORDER)
            if algorithm != row['algorithm']:
                row['id'] += '@' + algorithm
                row['algorithm'] = algorithm
                row['guidance'] = '按算法适用条件提供的替代建议'
        action = item.get('action', 'configuration')
        if action not in ('configuration', 'algorithm'):
            raise ValueError('未知推荐操作。')
        if action == 'algorithm' and any(row[k] != (context or {}).get(k) for k in ('car', 'scene', 'implement')):
            raise ValueError('算法建议不能更换当前车辆、机具或场景。')
        if (action == 'configuration' and context and all(row[k] == context.get(k) for k in ('car', 'scene', 'implement'))
                and {'Radar', 'Camera', 'Imu', 'RTK'} <= set(context.get('sensors') or [])):
            action = 'algorithm'
        if row['id'] in seen:
            raise ValueError('模型返回了不支持或重复的方案，请重试。')
        reason = item.get('reason')
        if not isinstance(reason, str) or not reason.strip() or len(reason) > 1200:
            raise ValueError('模型推荐理由格式无效。')
        seen.add(row['id'])
        result.append({**public_recipe(row), 'reason': reason, 'action': action})
    question = reply.get('clarification', '')
    answer = reply.get('answer', '')
    if not isinstance(answer, str) or len(answer) > 4000:
        raise ValueError('助手返回内容无效。')
    if not isinstance(question, str) or len(question) > 1200 or (not result and not question.strip() and not answer.strip()):
        raise ValueError('模型未给出推荐或补充问题，请重试。')
    memory = reply.get('memory', '')
    if not isinstance(memory, str) or len(memory) > 4000:
        raise ValueError('会话备忘格式无效。')
    return dict(recommendations=result, clarification=question, answer=answer, memory=memory,
                limitation='仿真配置建议不代替真实作业与安全评估。')


def recommend(text, rows, call=None, mode='assistant', context=None, history=None, knowledge=None, memory=''):
    if not isinstance(text, str) or not text.strip() or len(text) > 2000:
        raise ValueError('请输入1至2000字的任务需求。')
    if mode not in ('assistant', 'vehicle', 'algorithm', 'help'):
        raise ValueError('未知助手模式。')
    context = context if isinstance(context, dict) else {}
    if mode == 'vehicle':
        context = {}
    if mode == 'algorithm':
        rows = [r for r in rows if all(r[k] == context.get(k) for k in ('car', 'scene', 'implement'))]
        if not rows:
            return dict(recommendations=[], answer='当前装配与场景暂未提供匹配的推荐算法，请选择其他场景或机具；仍可手动使用算法库。',
                        clarification='', limitation='仿真配置建议不代替真实作业与安全评估。')
    prompt_path = Path(__file__).resolve().parents[1] / 'config/assistant_system_prompt.txt'
    if not prompt_path.is_file():
        from ament_index_python.packages import get_package_share_directory
        prompt_path = Path(get_package_share_directory('agribot_farm_sim')) / 'config/assistant_system_prompt.txt'
    prompt = prompt_path.read_text()
    if knowledge is None:
        _, knowledge = assistant_catalog(Path(__file__).resolve().parents[2])
    # A caller may scope the permitted choices without hiding the full inventory.
    allowed_ids = {r['id'] for r in rows}
    knowledge = {**knowledge, 'choices': [c for c in knowledge['choices'] if c['id'] in allowed_ids]}
    turns = [dict(role=t['role'], content=t['content'][:4000]) for t in (history or [])[-40:]
             if isinstance(t, dict) and t.get('role') in ('user', 'assistant') and isinstance(t.get('content'), str)]
    payload = dict(model='deepseek-flash', messages=[dict(role='system', content=prompt),
        dict(role='system', content=json.dumps(dict(knowledge=knowledge), ensure_ascii=False)), *turns,
        dict(role='user', content=json.dumps(dict(request=text, mode=mode, current_state=context, memory=memory[:4000]), ensure_ascii=False))],
        response_format={'type': 'json_object'}, thinking={'type': 'disabled'}, max_tokens=3000, stream=False)
    for attempt in range(2):
        try:
            reply = (call or request_model_reply)(payload)
            try:
                return validate_reply(reply, rows, context)
            except ValueError as error:
                raise InvalidModelReply(str(error)) from None
        except (InvalidModelReply, json.JSONDecodeError) as error:
            if attempt:
                raise ValueError('助手返回的方案格式或适配关系有误，请重新提问。') from None
            payload['messages'].append(dict(role='system', content=
                '上次输出未通过本地校验。请重新返回完整JSON；方案必须用candidate_id基础ID、action、components、reason，'
                '不能复制历史中的完整配置对象。严格遵守允许定位和当前状态，不执行操作。校验信息：' +
                ('JSON格式无效' if isinstance(error, json.JSONDecodeError) else str(error))))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace-src', type=Path, required=True)
    parser.add_argument('--catalog-only', action='store_true')
    parser.add_argument('--candidate-id')
    args = parser.parse_args()
    try:
        rows, knowledge = assistant_catalog(args.workspace_src)
        if args.candidate_id:
            base, _, algorithm = args.candidate_id.partition('@')
            matches = [r for r in rows if r['id'] == base]
            if not matches:
                raise ValueError('该推荐的配置已变化，请重新获取推荐。')
            row = matches[0]
            if algorithm:
                components = next((dict(zip(GROUP_ORDER, values)) for values in component_cases(
                    dict(components=row['allowedComponents'])) if '_'.join(values) == algorithm), None)
                if components is None:
                    raise ValueError('该算法已不符合当前场景限制，请重新获取推荐。')
                result = validate_reply(dict(recommendations=[dict(candidate_id=base, components=components,
                    reason='重新核对当前适配关系。')]), rows)
            else:
                result = dict(recommendations=[public_recipe(row)])
        else:
            if args.catalog_only:
                result = dict(candidates=rows, knowledge=knowledge)
            else:
                request = json.load(sys.stdin)
                result = recommend(request['text'], rows, mode=request.get('mode', 'assistant'), context=request.get('context'),
                    history=request.get('history') if isinstance(request.get('history'), list) else [],
                    memory=request.get('memory') if isinstance(request.get('memory'), str) else '', knowledge=knowledge)
        print(json.dumps(dict(ok=True, **result), ensure_ascii=False))
    except Exception as error:
        # Do not include request headers, SDK traces or response bodies containing secrets.
        message = str(error) if isinstance(error, ValueError) else '推荐服务发生错误，请检查本机配置和记录。'
        print(json.dumps(dict(ok=False, error=message), ensure_ascii=False))
        raise SystemExit(1)


if __name__ == '__main__':
    main()
