import subprocess
import urllib.request

import pytest

from agribot_farm_sim import assistant_transport as transport


def settings(monkeypatch, **overrides):
    values = {'mode': 'manual', 'ignore-hosts': ['localhost', '127.0.0.1'],
              'host': '127.0.0.1', 'port': 7890, **overrides}
    monkeypatch.setattr(transport, '_desktop_setting', lambda schema, key: values[key])


def test_manual_desktop_https_proxy_is_used_without_shell_proxy(monkeypatch):
    settings(monkeypatch)
    monkeypatch.setattr(transport.urllib.request, 'getproxies', lambda: {})
    monkeypatch.setattr(transport.urllib.request, 'proxy_bypass', lambda host: False)
    monkeypatch.setattr(transport.urllib.request, 'urlopen', lambda *a, **kw: pytest.fail('must use desktop proxy'))
    calls = []
    class Opener:
        def open(self, request, timeout):
            calls.append((request.full_url, timeout))
            return 'response'
    def build(handler):
        assert handler.proxies == {'https': 'http://127.0.0.1:7890'}
        return Opener()
    monkeypatch.setattr(transport.urllib.request, 'build_opener', build)
    assert transport.model_urlopen(urllib.request.Request('https://api.deepseek.com/models')) == 'response'
    assert calls == [('https://api.deepseek.com/models', 25)]


@pytest.mark.parametrize('proxies,bypass', [({'https': 'http://configured:3128'}, False), ({}, True)])
def test_explicit_environment_and_no_proxy_are_preserved(monkeypatch, proxies, bypass):
    monkeypatch.setattr(transport.urllib.request, 'getproxies', lambda: proxies)
    monkeypatch.setattr(transport.urllib.request, 'proxy_bypass', lambda host: bypass)
    monkeypatch.setattr(transport, 'desktop_https_proxy', lambda host: pytest.fail('must preserve explicit routing'))
    monkeypatch.setattr(transport.urllib.request, 'urlopen', lambda request, timeout: timeout)
    assert transport.model_urlopen(urllib.request.Request('https://api.deepseek.com/models'), 7) == 7


@pytest.mark.parametrize('overrides', [
    {'mode': 'none'}, {'mode': 'auto'}, {'host': ''}, {'host': 'bad/path'},
    {'port': 0}, {'port': 65536}, {'port': True}, {'ignore-hosts': ['*.deepseek.com']},
])
def test_disabled_invalid_or_excluded_desktop_proxy_is_ignored(monkeypatch, overrides):
    settings(monkeypatch, **overrides)
    assert transport.desktop_https_proxy('api.deepseek.com') is None


@pytest.mark.parametrize('error', [FileNotFoundError(), subprocess.TimeoutExpired('gsettings', 1), ValueError()])
def test_missing_desktop_tools_never_prevent_ai_requests(monkeypatch, error):
    def unavailable(*args):
        raise error
    monkeypatch.setattr(transport, '_desktop_setting', unavailable)
    assert transport.desktop_https_proxy('api.deepseek.com') is None


def test_ipv6_proxy_has_valid_url_authority(monkeypatch):
    settings(monkeypatch, host='::1')
    assert transport.desktop_https_proxy('api.deepseek.com') == 'http://[::1]:7890'
