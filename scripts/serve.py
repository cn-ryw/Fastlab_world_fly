#!/usr/bin/env python3
#
# Copyright 2026 Manifold Tech Ltd.
# Author: MENG Guotao <mengguotao@manifoldtech.cn>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

"""Local-only HTTP server for the Google 3D Tiles flight app.

Only the application entry point and the explicit ``/src``, ``/asset`` and
``/ThirdParty/Cesium`` mounts are exposed.  The project root is deliberately
*not* a document root: it contains Git metadata, checkpoints, scripts and
experiment notes which must never be reachable through the development web
server.

The server also provides the same-origin ``/api/path`` persistence API used by
the gate-course editor.
"""

import http.server
import socketserver
import os
import re
import sys
import tempfile
from pathlib import Path
from urllib.parse import unquote, urlsplit

PORT = 8080
DEFAULT_HOST = '127.0.0.1'
PROJECT_ROOT = Path(__file__).resolve().parents[1]
PATHS_DIR = PROJECT_ROOT / 'asset' / 'gate-paths'
# Cesium 静态资源目录：优先用环境变量，其次检查常见 Docker 镜像挂载点
CESIUM_DIR = Path(os.environ.get(
    'CESIUM_DIR',
    '/var/www/ThirdParty/Cesium' if os.path.isdir('/var/www/ThirdParty/Cesium')
    else str(PROJECT_ROOT / 'third_party' / 'Cesium')
)).resolve()
MAX_PATH_BODY = 64 * 1024  # 64 KB — tracks are a few hundred bytes each
SAFE_NAME_RE = re.compile(r'^[A-Za-z0-9._-]{1,200}\.json$')
ROOT_STATIC_FILES = {
    '/': 'index.html',
    '/index.html': 'index.html',
    # Expose only the reviewed shared controller defaults, not the whole
    # config directory or any other project-root files.
    '/config/drone_controller_config.json': 'config/drone_controller_config.json',
}
STATIC_MOUNTS = {
    '/src/': PROJECT_ROOT / 'src',
    '/asset/': PROJECT_ROOT / 'asset',
    '/ThirdParty/Cesium/': CESIUM_DIR,
}


def _safe_path_file(name):
    """Validate the `<safe_name>.json` path param and resolve it inside
    PATHS_DIR.  Returns absolute file path, or None if the name is unsafe
    (contains path-traversal, wrong extension, empty, too long, etc.).
    Defence-in-depth: even though the regex would reject `..`, we still
    do a real-path check on the resolved file so symlinks can't escape.
    """
    if not name or not SAFE_NAME_RE.match(name):
        return None
    base = PATHS_DIR.resolve()
    candidate = (base / name).resolve()
    try:
        if os.path.commonpath([str(candidate), str(base)]) != str(base):
            return None
    except ValueError:
        return None
    return str(candidate)


def _contains_forbidden_segment(relative_path):
    """Reject traversal, hidden files and platform-specific path tricks."""
    if '\x00' in relative_path or '\\' in relative_path:
        return True
    parts = relative_path.split('/')
    return any(part in {'.', '..'} or part.startswith('.') for part in parts if part)


def _resolve_inside(base, relative_path):
    """Resolve a URL-relative path and prove it remains under *base*."""
    if _contains_forbidden_segment(relative_path):
        return None
    base = Path(base).resolve()
    candidate = (base / relative_path).resolve()
    try:
        if os.path.commonpath([str(candidate), str(base)]) != str(base):
            return None
    except ValueError:
        return None
    return candidate


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        self._resolved_static_path = None
        super().__init__(*args, directory=str(PROJECT_ROOT), **kwargs)

    def translate_path(self, path):
        """Return only the path pre-authorised by :meth:`_static_path`."""
        if self._resolved_static_path is not None:
            return str(self._resolved_static_path)
        # ``send_head`` must never fall back to PROJECT_ROOT if a future
        # handler method calls it without first performing the whitelist check.
        return str(PROJECT_ROOT / '__forbidden_static_path__')

    def _static_path(self):
        try:
            url_path = unquote(urlsplit(self.path or '/').path, errors='strict')
        except (UnicodeDecodeError, ValueError):
            return None
        if url_path in ROOT_STATIC_FILES:
            candidate = _resolve_inside(PROJECT_ROOT, ROOT_STATIC_FILES[url_path])
        else:
            candidate = None
            for prefix, base in STATIC_MOUNTS.items():
                if url_path.startswith(prefix):
                    candidate = _resolve_inside(base, url_path[len(prefix):])
                    break
        # No directory listing and no implicit index lookup.  Symlink targets
        # have already been constrained by ``_resolve_inside``.
        return candidate if candidate is not None and candidate.is_file() else None

    def _serve_static(self, head_only=False):
        resolved = self._static_path()
        if resolved is None:
            self._send_plain(404, 'not found')
            return
        self._resolved_static_path = resolved
        try:
            if head_only:
                response = self.send_head()
                if response:
                    response.close()
            else:
                super().do_GET()
        finally:
            self._resolved_static_path = None

    def handle(self):
        try:
            super().handle()
        except (BrokenPipeError, ConnectionResetError) as e:
            print(f"Client connection closed while handling request: {e}", file=sys.stderr)

    def end_headers(self):
        # This API is intentionally same-origin.  Do not add wildcard CORS:
        # without it an unrelated web page cannot read or mutate local paths.
        origin = self.headers.get('Origin')
        if origin and self._origin_allowed(origin):
            self.send_header('Access-Control-Allow-Origin', origin)
            self.send_header('Vary', 'Origin')
            self.send_header('Access-Control-Allow-Methods', 'GET, PUT, DELETE, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('X-Content-Type-Options', 'nosniff')
        # Per-path caching: immutable assets get long max-age; HTML revalidates
        path_no_query = (self.path or '').split('?', 1)[0]
        # 开发服务器：所有动态资源 no-cache。长缓存只用于版本化的第三方库。
        if path_no_query.startswith('/src/') or path_no_query.startswith('/api/'):
            self.send_header('Cache-Control', 'no-store')
        elif path_no_query.startswith('/ThirdParty/') or path_no_query.startswith('/asset/vendor/'):
            self.send_header('Cache-Control', 'public, max-age=86400')
        else:
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
        super().end_headers()

    def _allowed_hosts(self):
        """Return the fixed loopback Host allowlist for this listener."""
        listen_port = int(self.server.server_address[1])
        return {("127.0.0.1", listen_port), ("localhost", listen_port)}

    def _request_host(self):
        raw_host = self.headers.get('Host', '').strip()
        if not raw_host or any(character in raw_host for character in '\r\n/?#@'):
            return None
        try:
            parsed = urlsplit(f'//{raw_host}')
            hostname = (parsed.hostname or '').lower()
            port = parsed.port
        except ValueError:
            return None
        if port is None:
            port = int(self.server.server_address[1])
        return hostname, port

    def _host_allowed(self):
        return self._request_host() in self._allowed_hosts()

    def _reject_untrusted_host(self):
        # Do not derive trust from a caller-controlled Host header.  A fixed
        # loopback allowlist prevents DNS rebinding from turning an attacker
        # origin into a same-origin gate-path API client.
        if not self._host_allowed():
            self._send_plain(403, 'untrusted Host header')
            return True
        return False

    def _origin_allowed(self, origin):
        if not self._host_allowed():
            return False
        if (
            not isinstance(origin, str)
            or not origin
            or origin != origin.strip()
            or any(ord(character) < 0x21 or ord(character) == 0x7f
                   for character in origin)
        ):
            return False
        try:
            parsed = urlsplit(origin)
            hostname = (parsed.hostname or '').lower()
            port = parsed.port
            username = parsed.username
            password = parsed.password
        except (TypeError, ValueError):
            return False

        # A browser Origin is a serialized origin, not an arbitrary URL: it
        # has no credentials, path, query or fragment.  This server is plain
        # HTTP even when configured to listen on port 443, so HTTPS origins
        # must not be treated as equivalent.
        if (
            parsed.scheme.lower() != 'http'
            or not parsed.netloc
            or username is not None
            or password is not None
            or parsed.path
            or parsed.query
            or parsed.fragment
            or parsed.netloc.endswith(':')
        ):
            return False

        normalized_port = 80 if port is None else port
        return (hostname, normalized_port) in self._allowed_hosts()

    def _reject_cross_origin_api(self):
        origin = self.headers.get('Origin')
        if origin and not self._origin_allowed(origin):
            self._send_plain(403, 'cross-origin API access denied')
            return True
        return False

    def guess_type(self, path):
        if path.endswith('.js'):
            return 'application/javascript'
        return super().guess_type(path)

    # ---- Path persistence API ----------------------------------------
    # GET  /api/path/<name>.json         → 200 JSON body | 404
    # PUT  /api/path/<name>.json         → 204 on success, 400 on bad body, 413 on oversize
    # DELETE /api/path/<name>.json       → 204 on success, 404 if missing
    # OPTIONS /api/path/<name>.json      → 204 (pre-flight CORS)
    # The regex on names keeps the filesystem surface flat; clients
    # build names from `<sanitized_scene_name>_<size>.json` so the key is
    # stable across browsers and survives renames of the scene on disk.

    def _handle_api(self):
        """Parse `/api/path/<name>` from self.path. Returns the resolved
        filesystem path, or sends an error and returns None."""
        m = re.match(r'^/api/path/([^/?#]+)$', self.path)
        if not m:
            self._send_plain(404, 'not a path route')
            return None
        file_path = _safe_path_file(m.group(1))
        if file_path is None:
            self._send_plain(400, 'invalid path name')
            return None
        return file_path

    def _send_plain(self, code, msg):
        body = msg.encode('utf-8') + b'\n'
        self.send_response(code)
        self.send_header('Content-Type', 'text/plain; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        if self._reject_untrusted_host():
            return
        # CORS pre-flight for PUT/DELETE issued by the browser.
        if self.path.startswith('/api/path/'):
            if self._reject_cross_origin_api():
                return
            self.send_response(204)
            self.end_headers()
            return
        self._send_plain(405, 'method not allowed')

    def do_GET(self):
        if self._reject_untrusted_host():
            return
        if self.path.startswith('/api/path/'):
            if self._reject_cross_origin_api():
                return
            fp = self._handle_api()
            if fp is None:
                return
            if not os.path.isfile(fp):
                self._send_plain(404, 'not found')
                return
            try:
                with open(fp, 'rb') as f:
                    body = f.read()
            except OSError as e:
                self._send_plain(500, f'read failed: {e}')
                return
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self._serve_static()

    def do_HEAD(self):
        if self._reject_untrusted_host():
            return
        if self.path.startswith('/api/path/'):
            self._send_plain(405, 'method not allowed')
            return
        self._serve_static(head_only=True)

    def do_PUT(self):
        if self._reject_untrusted_host():
            return
        if not self.path.startswith('/api/path/'):
            self._send_plain(405, 'PUT only allowed on /api/path/')
            return
        if self._reject_cross_origin_api():
            return
        fp = self._handle_api()
        if fp is None:
            return
        length = int(self.headers.get('Content-Length', '0') or '0')
        if length <= 0:
            self._send_plain(400, 'empty body')
            return
        if length > MAX_PATH_BODY:
            self._send_plain(413, f'body too large (>{MAX_PATH_BODY} bytes)')
            return
        body = self.rfile.read(length)
        # Minimal JSON sanity check — catch obvious garbage here so the
        # client gets immediate feedback instead of a mystery-500 later.
        # Full schema validation lives in the client (path-store.js).
        try:
            import json
            json.loads(body.decode('utf-8'))
        except Exception as e:
            self._send_plain(400, f'not valid JSON: {e}')
            return
        os.makedirs(PATHS_DIR, exist_ok=True)
        try:
            # Write to a tempfile and rename so a crash mid-write doesn't
            # leave half a file on disk. Same-dir rename is atomic on
            # POSIX; good enough for single-user local persistence.
            with tempfile.NamedTemporaryFile('wb', dir=PATHS_DIR, delete=False) as f:
                tmp = f.name
                f.write(body)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, fp)
        except OSError as e:
            try:
                if 'tmp' in locals():
                    os.unlink(tmp)
            except OSError:
                pass
            print(f'gate path write failed: {e}', file=sys.stderr)
            self._send_plain(500, 'write failed')
            return
        self.send_response(204)
        self.end_headers()

    def do_DELETE(self):
        if self._reject_untrusted_host():
            return
        if not self.path.startswith('/api/path/'):
            self._send_plain(405, 'DELETE only allowed on /api/path/')
            return
        if self._reject_cross_origin_api():
            return
        fp = self._handle_api()
        if fp is None:
            return
        if not os.path.isfile(fp):
            self._send_plain(404, 'not found')
            return
        try:
            os.remove(fp)
        except OSError as e:
            self._send_plain(500, f'delete failed: {e}')
            return
        self.send_response(204)
        self.end_headers()


class ReusableTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

if __name__ == '__main__':
    port = int(sys.argv[1]) if len(sys.argv) > 1 else PORT
    host = sys.argv[2] if len(sys.argv) > 2 else os.environ.get('MINDCLOUD_WEB_HOST', DEFAULT_HOST)
    os.makedirs(PATHS_DIR, exist_ok=True)
    with ReusableTCPServer((host, port), Handler) as httpd:
        print(f"Google 3D Tiles Flight running at http://{host}:{port}")
        print(f"Gate-path persistence: {PATHS_DIR}")
        print("Press Ctrl+C to stop")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServer stopped.")
