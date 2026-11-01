"""Use the desktop's configured HTTPS proxy when a GUI has no shell environment."""
import ast
import fnmatch
import subprocess
import urllib.request


def _desktop_setting(schema, key):
    result = subprocess.run(
        ['gsettings', 'get', schema, key], check=True, capture_output=True,
        text=True, timeout=1,
    )
    return ast.literal_eval(result.stdout.strip())


def desktop_https_proxy(host):
    try:
        if _desktop_setting('org.gnome.system.proxy', 'mode') != 'manual':
            return None
        excluded = _desktop_setting('org.gnome.system.proxy', 'ignore-hosts')
        if not isinstance(excluded, list) or not all(isinstance(item, str) for item in excluded):
            return None
        if (any(fnmatch.fnmatchcase(host.lower(), pattern.lower()) for pattern in excluded) or
                urllib.request.proxy_bypass_environment(host, {'no': ','.join(excluded)})):
            return None
        proxy_host = _desktop_setting('org.gnome.system.proxy.https', 'host')
        port = _desktop_setting('org.gnome.system.proxy.https', 'port')
        if (not isinstance(proxy_host, str) or not proxy_host or
                any(c.isspace() or c in '/@?#' for c in proxy_host) or
                type(port) is not int or not 1 <= port <= 65535):
            return None
        if ':' in proxy_host and not proxy_host.startswith('['):
            proxy_host = '[' + proxy_host + ']'
        return 'http://' + proxy_host + ':' + str(port)
    except (OSError, subprocess.SubprocessError, ValueError, SyntaxError):
        return None


def model_urlopen(request, timeout=25):
    # Preserve explicit shell proxies and NO_PROXY; never change process-wide routing.
    if urllib.request.getproxies().get('https') or urllib.request.proxy_bypass(request.host):
        return urllib.request.urlopen(request, timeout=timeout)
    proxy = desktop_https_proxy(request.host)
    if proxy is None:
        return urllib.request.urlopen(request, timeout=timeout)
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({'https': proxy}))
    return opener.open(request, timeout=timeout)
