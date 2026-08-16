"""Security regression tests for the local static and gate-path server."""

import http.client
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

import serve  # noqa: E402


class ServeSecurityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = serve.ReusableTCPServer(("127.0.0.1", 0), serve.Handler)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=2)

    @classmethod
    def request(cls, method, path, headers=None, body=None):
        connection = http.client.HTTPConnection(*cls.server.server_address, timeout=3)
        connection.request(method, path, body=body, headers=headers or {})
        response = connection.getresponse()
        data = response.read()
        result = response.status, dict(response.getheaders()), data
        connection.close()
        return result

    def test_explicit_static_mounts_remain_available(self):
        paths = [
            "/", "/index.html", "/src/main.js", "/asset/vendor/playcanvas.min.js",
            "/ThirdParty/Cesium/Cesium.js", "/ThirdParty/Cesium/Widgets/widgets.css",
        ]
        for path in paths:
            with self.subTest(path=path):
                status, _, _ = self.request("HEAD", path)
                self.assertEqual(status, 200)

    def test_sensitive_and_traversal_paths_are_denied(self):
        paths = [
            "/.git/HEAD", "/.git/config", "/CLAUDE.md", "/README.md",
            "/scripts/serve.py", "/tests/test_serve_security.py",
            "/third_party/DA360/checkpoints/DA360_large.pth", "/src/", "/asset/",
            "/ThirdParty/Cesium/",
            "/ThirdParty/Cesium/%2e%2e/%2e%2e/scripts/serve.py",
            "/src/%2e%2e/%2e%2e/etc/hostname",
        ]
        for path in paths:
            with self.subTest(path=path):
                status, _, _ = self.request("GET", path)
                self.assertEqual(status, 404)

    def test_gate_api_rejects_cross_origin_request(self):
        status, headers, _ = self.request(
            "GET", "/api/path/does-not-exist.json",
            headers={"Origin": "https://attacker.invalid"},
        )
        self.assertEqual(status, 403)
        self.assertNotIn("Access-Control-Allow-Origin", headers)

    def test_dns_rebinding_host_is_rejected_for_static_and_api(self):
        _, port = self.server.server_address
        attacker_host = f"attacker.invalid:{port}"
        for path, headers in (
            ("/", {"Host": attacker_host}),
            (
                "/api/path/does-not-exist.json",
                {"Host": attacker_host, "Origin": f"http://{attacker_host}"},
            ),
        ):
            with self.subTest(path=path):
                status, response_headers, _ = self.request("GET", path, headers=headers)
                self.assertEqual(status, 403)
                self.assertNotIn("Access-Control-Allow-Origin", response_headers)

    def test_gate_api_allows_same_origin_request(self):
        host, port = self.server.server_address
        origin = f"http://{host}:{port}"
        status, headers, _ = self.request(
            "GET", "/api/path/does-not-exist.json",
            headers={"Origin": origin, "Host": f"{host}:{port}"},
        )
        self.assertEqual(status, 404)
        self.assertEqual(headers["Access-Control-Allow-Origin"], origin)

    @staticmethod
    def origin_allowed(listener_port, origin, host="127.0.0.1"):
        handler = object.__new__(serve.Handler)
        handler.server = SimpleNamespace(server_address=("127.0.0.1", listener_port))
        handler.headers = {"Host": host}
        return handler._origin_allowed(origin)

    def test_http_default_port_origin_may_omit_port(self):
        for host in ("127.0.0.1", "localhost"):
            for origin in (f"http://{host}", f"http://{host}:80"):
                with self.subTest(host=host, origin=origin):
                    self.assertTrue(self.origin_allowed(80, origin, host))

    def test_origin_parser_rejects_unsafe_or_malformed_origins(self):
        rejected = [
            "https://127.0.0.1",
            "http://attacker.invalid",
            "http://127.0.0.1:81",
            "http://user@127.0.0.1",
            "http://user:password@127.0.0.1",
            "http://127.0.0.1:",
            "http://127.0.0.1/path",
            "http://127.0.0.1?query",
            "http://127.0.0.1#fragment",
            " http://127.0.0.1",
            "http://127.0.0.1 ",
            "http:///127.0.0.1",
            "http://127.0.0.1:99999",
            "null",
        ]
        for origin in rejected:
            with self.subTest(origin=origin):
                self.assertFalse(self.origin_allowed(80, origin))

    def test_port_443_listener_still_requires_plain_http_origin(self):
        self.assertTrue(self.origin_allowed(443, "http://localhost:443", "localhost"))
        self.assertFalse(self.origin_allowed(443, "http://localhost", "localhost"))
        self.assertFalse(self.origin_allowed(443, "https://localhost", "localhost"))

    def test_symlink_cannot_escape_static_mount(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mount = root / "mount"
            outside = root / "secret.txt"
            mount.mkdir()
            outside.write_text("secret", encoding="utf-8")
            (mount / "escape.txt").symlink_to(outside)
            self.assertIsNone(serve._resolve_inside(mount, "escape.txt"))


if __name__ == "__main__":
    unittest.main()
