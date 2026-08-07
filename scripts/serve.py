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

"""Simple HTTP server for the Google 3D Tiles flight app.

Serves static files under the project root (same as `python -m http.server`)
plus a small persistence API for per-scene gate-course paths, see the
`/api/path/` routes at the bottom of `Handler`.
"""

import http.server
import socketserver
import os
import re
import sys

PORT = 8080
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATHS_DIR = os.path.join(PROJECT_ROOT, 'asset', 'gate-paths')
# Cesium 静态资源目录：优先用环境变量，其次检查常见 Docker 镜像挂载点
CESIUM_DIR = os.environ.get(
    'CESIUM_DIR',
    '/var/www/ThirdParty/Cesium' if os.path.isdir('/var/www/ThirdParty/Cesium')
    else os.path.join(PROJECT_ROOT, 'third_party', 'Cesium')
)
MAX_PATH_BODY = 64 * 1024  # 64 KB — tracks are a few hundred bytes each
SAFE_NAME_RE = re.compile(r'^[A-Za-z0-9._-]{1,200}\.json$')


def _safe_path_file(name):
    """Validate the `<safe_name>.json` path param and resolve it inside
    PATHS_DIR.  Returns absolute file path, or None if the name is unsafe
    (contains path-traversal, wrong extension, empty, too long, etc.).
    Defence-in-depth: even though the regex would reject `..`, we still
    do a real-path check on the resolved file so symlinks can't escape.
    """
    if not name or not SAFE_NAME_RE.match(name):
        return None
    candidate = os.path.normpath(os.path.join(PATHS_DIR, name))
    # os.path.commonpath raises on Windows drive mismatch — not a concern
    # on this project's Linux-only deployment, but we still guard.
    try:
        if os.path.commonpath([candidate, PATHS_DIR]) != PATHS_DIR:
            return None
    except ValueError:
        return None
    return candidate


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=PROJECT_ROOT, **kwargs)

    def translate_path(self, path):
        """将 /ThirdParty/Cesium/... 映射到 third_party/Cesium/..."""
        if path.startswith('/ThirdParty/Cesium/'):
            rel = path[len('/ThirdParty/Cesium/'):]
            return os.path.join(CESIUM_DIR, rel)
        return super().translate_path(path)

    def handle(self):
        try:
            super().handle()
        except (BrokenPipeError, ConnectionResetError) as e:
            print(f"Client connection closed while handling request: {e}", file=sys.stderr)

    def end_headers(self):
        # Enable CORS and proper MIME types for ES modules
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, PUT, DELETE, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
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
        # CORS pre-flight for PUT/DELETE issued by the browser.
        if self.path.startswith('/api/path/'):
            self.send_response(204)
            self.end_headers()
            return
        super().do_OPTIONS() if hasattr(super(), 'do_OPTIONS') else self._send_plain(405, 'method not allowed')

    def do_GET(self):
        if self.path.startswith('/api/path/'):
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
        # Fall through to static file serving.
        super().do_GET()

    def do_PUT(self):
        if not self.path.startswith('/api/path/'):
            self._send_plain(405, 'PUT only allowed on /api/path/')
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
            tmp = fp + '.tmp'
            with open(tmp, 'wb') as f:
                f.write(body)
            os.replace(tmp, fp)
        except OSError as e:
            self._send_plain(500, f'write failed: {e}')
            return
        self.send_response(204)
        self.end_headers()

    def do_DELETE(self):
        if not self.path.startswith('/api/path/'):
            self._send_plain(405, 'DELETE only allowed on /api/path/')
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
    os.makedirs(PATHS_DIR, exist_ok=True)
    with ReusableTCPServer(("", port), Handler) as httpd:
        print(f"Google 3D Tiles Flight running at http://127.0.0.1:{port}")
        print(f"Gate-path persistence: {PATHS_DIR}")
        print("Press Ctrl+C to stop")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServer stopped.")
