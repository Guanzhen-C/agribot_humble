import json
from pathlib import Path

import pytest

from agribot_farm_sim import natural_recommendation as assistant
from agribot_farm_sim.assistant_scenes import imported_scenes, list_scenes


SRC = Path(__file__).resolve().parents[2]


@pytest.fixture
def library(tmp_path, monkeypatch):
    monkeypatch.setenv('AGRIBOT_SCANNED_SCENE_ROOT', str(tmp_path))
    return tmp_path


def add_scene(root, name='自建园区'):
    folder = root / 'asset'
    folder.mkdir(exist_ok=True)
    for file in ('world.sdf', 'map.yaml', 'waypoints.yaml'):
        (folder / file).write_text('fixture')
    manifest = folder / 'scene.json'
    manifest.write_text(json.dumps(dict(schema_version=1, world='world.sdf', map='map.yaml',
        waypoints='waypoints.yaml', spawn=[0, 0, 0])))
    (root / 'catalog.json').write_text(json.dumps(dict(schemaVersion='1.0', scenarios=[dict(
        id='scan_fixture', name=name, imported=True, sceneManifest=str(manifest))])))
    return folder


def tool_call(name='list_scenes', arguments='{}'):
    return {'_tool_message': dict(role='assistant', content='', tool_calls=[
        dict(id='call_scene_1', type='function', function=dict(name=name, arguments=arguments))])}


def test_scene_groups_counts_and_imported_choices_are_not_default_scene_proof(library):
    add_scene(library, '苹果园')
    rows, knowledge = assistant.assistant_catalog(SRC)
    result = list_scenes(knowledge)
    assert result['counts'] == dict(builtin=9, imported=1, total=10)
    assert result['imported_scenes'][0]['name'] == '苹果园'
    assert result['imported_scenes'][0]['imported'] is True
    assert result['imported_scenes'][0]['usageDescription'] == ''
    assert all(not s['imported'] and s['source'] == 'builtin' for s in result['builtin_scenes'])
    choices = [r for r in rows if r['scene'] == 'scan_fixture']
    assert choices and {r['car'] for r in choices} == {f'Car{i}' for i in range(10)}
    assert all(r['sceneImported'] and r['sceneConfigSha256'] for r in choices)
    assert all(not c['familiar'] for c in result['choices'] if '|scan_fixture|' in c['id'])
    assert str(library) not in json.dumps(result, ensure_ascii=False)


def test_missing_or_escaped_scene_files_are_not_available(library):
    folder = add_scene(library)
    assert len(imported_scenes()) == 1
    (folder / 'world.sdf').unlink()
    assert imported_scenes() == []
    (folder / 'scene.json').write_text(json.dumps(dict(schema_version=1, spawn=[0, 0, 0],
        world='../catalog.json', map='map.yaml', waypoints='waypoints.yaml')))
    assert imported_scenes() == []


def test_scene_version_changes_without_changing_vehicle_version(library):
    folder = add_scene(library)
    rows, _ = assistant.assistant_catalog(SRC)
    first = next(r for r in rows if r['id'] == 'Car0|scan_fixture|none')
    (folder / 'world.sdf').write_text('changed scene')
    rows, _ = assistant.assistant_catalog(SRC)
    second = next(r for r in rows if r['id'] == first['id'])
    assert first['sceneConfigSha256'] != second['sceneConfigSha256']
    assert first['vehicleConfigSha256'] == second['vehicleConfigSha256']


def test_deleted_import_is_not_returned_by_fresh_query_or_candidate_catalog(library):
    add_scene(library)
    rows, knowledge = assistant.assistant_catalog(SRC)
    assert any(r['id'] == 'Car0|scan_fixture|none' for r in rows)
    (library / 'catalog.json').write_text(json.dumps(dict(schemaVersion='1.0', scenarios=[])))
    rows, knowledge = assistant.assistant_catalog(SRC)
    result = list_scenes(knowledge)
    assert result['counts'] == dict(builtin=9, imported=0, total=9)
    assert result['imported_scenes'] == [] and result['choices'] == []
    assert all(r['scene'] != 'scan_fixture' for r in rows)


def test_actual_tool_roundtrip_refreshes_catalog_and_resolves_clickable_import(library):
    rows, knowledge = assistant.assistant_catalog(SRC)
    calls = []
    def model(payload):
        calls.append(1)
        if len(calls) == 1:
            assert [t['function']['name'] for t in payload['tools']] == ['list_scenes']
            assert payload['tool_choice'] == 'auto'
            assert 'scenes' not in json.loads(payload['messages'][1]['content'])['knowledge']
            add_scene(library, '刚导入的中文园区')
            return tool_call()
        result = json.loads(payload['messages'][-1]['content'])
        assert payload['messages'][-2]['tool_calls'][0]['id'] == 'call_scene_1'
        assert payload['messages'][-1]['role'] == 'tool'
        assert payload['messages'][-1]['tool_call_id'] == 'call_scene_1'
        assert payload['tool_choice'] == 'none'
        assert result['counts'] == dict(builtin=9, imported=1, total=10)
        assert result['imported_scenes'][0]['name'] == '刚导入的中文园区'
        return dict(answer='这是用户导入的园区，可以先使用Car0进行巡检验证。', recommendations=[
            dict(candidate_id='Car0|scan_fixture|none', action='configuration', reason='沿用默认定位和控制，先确认通行空间。')])
    result = assistant.recommend('用刚导入的园区给我一个方案', rows, model, knowledge=knowledge,
        scene_catalog=lambda: assistant.assistant_catalog(SRC))
    assert len(calls) == 2 and result['sceneToolUsed'] is True
    card = result['recommendations'][0]
    assert card['scene'] == 'scan_fixture' and card['sceneName'] == '刚导入的中文园区'
    assert card['sceneImported'] and card['sceneConfigSha256']
    assert card['algorithm'] == 'stvl_off_fastlivo_rtk_smac_hybrid_mppi'


@pytest.mark.parametrize('name,args', [('run_shell', '{}'), ('list_scenes', '{"path":"/etc/passwd"}'),
                                       ('list_scenes', '[]'), ('list_scenes', '{bad')])
def test_only_the_no_argument_read_only_tool_is_allowed(library, name, args):
    rows, knowledge = assistant.assistant_catalog(SRC)
    calls = []
    def model(payload):
        calls.append(1)
        if len(calls) == 1:
            return tool_call(name, args)
        return dict(answer='只能查询场景，不会执行文件或运动操作。', recommendations=[])
    result = assistant.recommend('查询场景', rows, model, knowledge=knowledge,
        scene_catalog=lambda: pytest.fail('Invalid tool must not execute'))
    assert result['sceneToolUsed'] is False and len(calls) == 2


def test_imported_recommendation_requires_this_turn_tool_query(library):
    add_scene(library)
    rows, knowledge = assistant.assistant_catalog(SRC)
    calls = []
    card = dict(candidate_id='Car0|scan_fixture|none', reason='推荐待验证的园区巡检。')
    def model(payload):
        calls.append(1)
        if len(calls) == 2:
            assert payload['tool_choice'] == dict(type='function', function=dict(name='list_scenes'))
            return tool_call()
        return dict(recommendations=[card])
    result = assistant.recommend('推荐新场景方案', rows, model, knowledge=knowledge)
    assert len(calls) == 3 and result['sceneToolUsed'] is True


def test_scene_query_is_reused_for_controller_vision_and_implement_followups(library):
    add_scene(library)
    rows, knowledge = assistant.assistant_catalog(SRC)
    calls = []
    def first(payload):
        calls.append(1)
        if len(calls) == 1:
            return tool_call()
        return dict(answer='内置9个，导入1个。', recommendations=[])
    result = assistant.recommend('现在有几个场景', rows, first, knowledge=knowledge)
    token = result['sceneCatalogVersion']
    assert len(token) == 64 and result['modelRequestCount'] == 2 and result['sceneToolUsed']
    for question, tool, vision, controller in [('换一个控制器', 'none', 'off', 'rpp'),
            ('加上视觉', 'none', 'object_detection', 'rpp'), ('加个机械臂', 'lite6', 'object_detection', 'rpp')]:
        calls.clear()
        def followup(payload):
            calls.append(1)
            assert payload['tool_choice'] == 'none'
            known = json.loads(payload['messages'][1]['content'])['knowledge']
            assert known['sceneCatalogStatus'] == 'unchanged'
            assert known['sceneCatalog']['counts'] == dict(builtin=9, imported=1, total=10)
            assert any(c['id'] == 'Car0|scan_fixture|' + tool for c in known['choices'])
            return dict(recommendations=[dict(candidate_id='Car0|scan_fixture|' + tool, reason='保留其他组件，只改用户要求的部分。',
                components=dict(perception='stvl', vision=vision, localization='fastlivo_rtk', planner='smac_hybrid', controller=controller))])
        result = assistant.recommend(question, rows, followup, knowledge=knowledge,
            context=dict(sceneCatalogVersion=token), scene_catalog=lambda: pytest.fail('Unchanged catalog must not be queried again'))
        assert len(calls) == 1 and result['modelRequestCount'] == 1
        assert result['sceneCatalogReused'] and not result['sceneToolUsed'] and result['sceneCatalogVersion'] == token


@pytest.mark.parametrize('change', ['world', 'name', 'delete'])
def test_catalog_content_change_invalidates_previous_conversation_cache(library, change):
    folder = add_scene(library)
    _, before = assistant.assistant_catalog(SRC)
    if change == 'world':
        (folder / 'world.sdf').write_text('updated world')
    else:
        catalog = json.loads((library / 'catalog.json').read_text())
        if change == 'name':
            catalog['scenarios'][0]['name'] = '新名称'
        else:
            catalog['scenarios'] = []
        (library / 'catalog.json').write_text(json.dumps(catalog))
    rows, knowledge = assistant.assistant_catalog(SRC)
    assert knowledge['sceneCatalogVersion'] != before['sceneCatalogVersion']
    calls = []
    def model(payload):
        calls.append(1)
        if len(calls) == 1:
            assert payload['tool_choice'] == 'auto'
            assert json.loads(payload['messages'][1]['content'])['knowledge']['sceneCatalogStatus'] == 'not_loaded'
            return tool_call()
        return dict(answer='按最新目录回答。', recommendations=[])
    result = assistant.recommend('现在有几个场景', rows, model, knowledge=knowledge,
        context=dict(sceneCatalogVersion=before['sceneCatalogVersion']))
    assert result['sceneToolUsed'] and not result['sceneCatalogReused'] and len(calls) == 2
    assert result['sceneCatalogVersion'] == knowledge['sceneCatalogVersion']


def test_cache_does_not_remove_current_algorithm_scope_or_component_checks(library):
    add_scene(library)
    rows, knowledge = assistant.assistant_catalog(SRC)
    context = dict(sceneCatalogVersion=knowledge['sceneCatalogVersion'], car='Car0', scene='scan_fixture', implement='none')
    calls = []
    def model(payload):
        calls.append(1)
        known = json.loads(payload['messages'][1]['content'])['knowledge']
        assert [c['id'] for c in known['choices']] == ['Car0|scan_fixture|none']
        return dict(recommendations=[dict(candidate_id='Car0|scan_fixture|none', action='algorithm', reason='沿用当前装配。')])
    result = assistant.recommend('只调整算法', rows, model, mode='algorithm', context=context, knowledge=knowledge)
    assert len(calls) == 1 and result['sceneCatalogReused']
    context.update(car='Car4', scene='arable_field')
    def disallowed(payload):
        return dict(recommendations=[dict(candidate_id='Car4|arable_field|none', reason='错误的定位不得因缓存而被接受。',
            components=dict(perception='stvl', vision='off', localization='fastlivo_rtk', planner='smac_hybrid', controller='mppi'))])
    with pytest.raises(ValueError, match='格式或适配'):
        assistant.recommend('仍然用激光视觉定位', rows, disallowed, mode='algorithm', context=context, knowledge=knowledge)


def test_followup_after_count_question_forces_lookup_instead_of_repeating_rejected_recipe(library):
    add_scene(library)
    rows, knowledge = assistant.assistant_catalog(SRC)
    history = [dict(role='user', content='现在有几个场景'),
               dict(role='assistant', content='内置9个、导入1个，自建园区。'),
               dict(role='user', content='用小农机在这个新导入场景运行')]
    calls = []
    def model(payload):
        calls.append(payload['tool_choice'])
        if len(calls) == 2:
            assert payload['tool_choice'] == dict(type='function', function=dict(name='list_scenes'))
            return tool_call()
        return dict(answer='用当前Car0，点击后应用，不自动启动仿真。', recommendations=[dict(
            candidate_id='Car0|scan_fixture|none', action='configuration', reason='沿用默认方案，先核对通行条件。')])
    result = assistant.recommend('继续', rows, call=model, knowledge=knowledge, history=history,
        context=dict(car='Car0', scene='scan_fixture', implement='none', sceneImported=True, sensors=[]),
        memory='已查询导入场景数量，还未配置算法。', scene_catalog=lambda: assistant.assistant_catalog(SRC))
    forced = dict(type='function', function=dict(name='list_scenes'))
    assert calls == [forced, forced, 'none']
    assert result['sceneToolUsed'] is True and result['recommendations'][0]['scene'] == 'scan_fixture'


def test_forced_scene_query_does_not_accept_deleted_scene_or_disallowed_algorithm(library):
    add_scene(library)
    rows, knowledge = assistant.assistant_catalog(SRC)
    calls = []
    def model(payload):
        calls.append(1)
        if len(calls) == 2:
            (library / 'catalog.json').write_text(json.dumps(dict(schemaVersion='1.0', scenarios=[])))
            return tool_call()
        return dict(recommendations=[dict(candidate_id='Car0|scan_fixture|none', reason='过时方案不得应用。')])
    with pytest.raises(ValueError, match='格式或适配'):
        assistant.recommend('继续刚才的场景', rows, call=model, knowledge=knowledge,
            scene_catalog=lambda: assistant.assistant_catalog(SRC))
    assert len(calls) == 3


def test_tool_loop_is_bounded_and_greetings_need_no_tool(library):
    rows, knowledge = assistant.assistant_catalog(SRC)
    calls = []
    def repeated(payload):
        calls.append(1)
        return tool_call()
    with pytest.raises(ValueError, match='格式或适配'):
        assistant.recommend('查询场景', rows, repeated, knowledge=knowledge)
    assert len(calls) == 3
    result = assistant.recommend('你好', rows, lambda _: dict(answer='你好。', recommendations=[]), knowledge=knowledge)
    assert result['sceneToolUsed'] is False


def test_http_function_call_is_not_misparsed_as_final_json(monkeypatch):
    monkeypatch.setattr(assistant, 'configuration', lambda: ('private-test-key', 'https://api.deepseek.com', 'deepseek-flash'))
    message = tool_call()['_tool_message']
    class Response:
        def __enter__(self): return self
        def __exit__(self, *_): return False
        def read(self, _):
            return json.dumps(dict(choices=[dict(finish_reason='tool_calls', message=message)])).encode()
    def open_request(request, timeout):
        payload = json.loads(request.data)
        assert payload['tools'][0]['function']['name'] == 'list_scenes'
        return Response()
    monkeypatch.setattr(assistant.urllib.request, 'urlopen', open_request)
    assert assistant.request_model_reply(dict(tools=[assistant.SCENE_TOOL])) == {'_tool_message': message}
