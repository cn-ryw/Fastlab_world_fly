#!/usr/bin/env python3
"""Drive a real Firefox preview session and summarize browser-side timing.

The benchmark deliberately stops before YOPO goal selection.  It measures the
same six-face panorama capture and render loop used during planning without
printing browser network URLs (which may contain provider credentials).
"""

from __future__ import annotations

import argparse
import io
import json
import math
import os
import re
import shutil
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request


DEPTH_RE = re.compile(
    r"\[depth\]\s+[\d.]+Hz\s+.*?capture=(?P<capture>[\d.]+)ms"
    r"\s+render=(?P<render>[\d.]+)ms.*?age=(?P<age>[\d.]+)ms"
)
ABSOLUTE_URL_RE = re.compile(r"https?://[^\s<>\"']+", re.IGNORECASE)
CREDENTIAL_FIELD_RE = re.compile(
    r"(?i)\b(api[_-]?key|key|token|access[_-]?token|authorization)="
    r"[^\s&,;]+"
)


def _redact_url_match(match: re.Match[str]) -> str:
    """Remove URL userinfo, query and fragment while retaining useful origin/path."""
    raw = match.group(0)
    # These characters commonly terminate a URL embedded in prose.  Strip
    # them before parsing and then append them unchanged to preserve the error
    # message's punctuation.
    trailing = ""
    while raw and raw[-1] in ".,;)]}":
        trailing = raw[-1] + trailing
        raw = raw[:-1]
    try:
        parsed = urllib.parse.urlsplit(raw)
        hostname = parsed.hostname
        if not hostname:
            return "<redacted-url>" + trailing
        host = f"[{hostname}]" if ":" in hostname else hostname
        try:
            port = parsed.port
        except ValueError:
            port = None
        authority = f"{host}:{port}" if port is not None else host
        clean = urllib.parse.urlunsplit(
            (parsed.scheme.lower(), authority, parsed.path, "", "")
        )
        return clean + trailing
    except (TypeError, ValueError):
        return "<redacted-url>" + trailing


def sanitize_text(value: object) -> str:
    """Make diagnostic text safe to print even when a provider URL is present."""
    text = ABSOLUTE_URL_RE.sub(_redact_url_match, str(value))
    text = CREDENTIAL_FIELD_RE.sub(r"\1=<redacted>", text)
    return "".join(char if char >= " " else "?" for char in text)


def percentile(values: list[float], fraction: float) -> float | None:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return None
    index = max(0, min(len(finite) - 1, math.ceil(fraction * len(finite)) - 1))
    return finite[index]


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def loopback_no_proxy(value: str | None) -> str:
    entries = [entry.strip() for entry in (value or "").split(",")]
    unique = [entry for index, entry in enumerate(entries) if entry and entry not in entries[:index]]
    for required in ("127.0.0.1", "localhost", "::1"):
        if required not in unique:
            unique.append(required)
    return ",".join(unique)


class WebDriver:
    def __init__(self, endpoint: str, request_timeout_s: float = 30.0):
        self.endpoint = endpoint.rstrip("/")
        self.request_timeout_s = request_timeout_s
        # The browser should inherit the workstation proxy, but the control
        # channel is always loopback.  Explicitly bypass environment proxies
        # here because a missing NO_PROXY would otherwise send geckodriver
        # commands to Clash/ShellCrash.
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.session_id: str | None = None

    def request(self, method: str, path: str, payload: object | None = None):
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            f"{self.endpoint}{path}",
            data=data,
            method=method,
            headers={"Content-Type": "application/json"},
        )
        try:
            with self.opener.open(
                request,
                timeout=self.request_timeout_s,
            ) as response:
                body = json.load(response)
        except urllib.error.HTTPError as exc:
            try:
                body = json.loads(exc.read().decode("utf-8", errors="replace"))
                value = body.get("value", {}) if isinstance(body, dict) else {}
                detail = value.get("message") if isinstance(value, dict) else None
                error = value.get("error") if isinstance(value, dict) else None
            except (AttributeError, json.JSONDecodeError, UnicodeDecodeError):
                detail = None
                error = None
            summary = detail or exc.reason or f"HTTP {exc.code}"
            prefix = f"WebDriver {error}: " if error else "WebDriver request failed: "
            raise RuntimeError(sanitize_text(prefix + str(summary))) from None
        if not isinstance(body, dict):
            raise RuntimeError("WebDriver returned a non-object response")
        value = body.get("value")
        if isinstance(value, dict) and value.get("error"):
            raise RuntimeError(sanitize_text(
                f"WebDriver {value.get('error')}: {value.get('message')}"
            ))
        return value

    def create(self, firefox_binary: str):
        value = self.request(
            "POST",
            "/session",
            {
                "capabilities": {
                    "alwaysMatch": {
                        "browserName": "firefox",
                        "moz:firefoxOptions": {
                            "binary": firefox_binary,
                            "args": ["-private-window"],
                            "prefs": {
                                "network.proxy.type": 5,
                            },
                        },
                    }
                }
            },
        )
        if not isinstance(value, dict) or not value.get("sessionId"):
            raise RuntimeError("WebDriver did not return a session ID")
        self.session_id = str(value["sessionId"])

    def close(self):
        if not self.session_id:
            return
        try:
            self.request("DELETE", f"/session/{self.session_id}")
        except Exception:
            pass
        self.session_id = None

    def navigate(self, url: str):
        if not self.session_id:
            raise RuntimeError("WebDriver session has not been created")
        self.request("POST", f"/session/{self.session_id}/url", {"url": url})

    def execute(self, script: str):
        if not self.session_id:
            raise RuntimeError("WebDriver session has not been created")
        return self.request(
            "POST",
            f"/session/{self.session_id}/execute/sync",
            {"script": script, "args": []},
        )


def wait_until(driver: WebDriver, script: str, timeout_s: float, label: str):
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            if driver.execute(script):
                return
        except Exception as exc:  # page navigation and reloads are transient
            last_error = exc
        time.sleep(0.5)
    detail = f": {last_error}" if last_error else ""
    raise TimeoutError(f"timed out waiting for {label}{detail}")


def format_metric(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.1f} ms"


def executable(value: str, label: str) -> str:
    if os.path.sep in value:
        candidate = os.path.abspath(value)
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    else:
        candidate = shutil.which(value)
        if candidate:
            return candidate
    raise FileNotFoundError(f"{label} executable was not found: {value}")


def native_geckodriver(value: str) -> str:
    """Avoid Snap's command proxy so the benchmark owns and can reap the process."""
    resolved = executable(value, "geckodriver")
    if os.path.realpath(resolved) != "/usr/bin/snap":
        return resolved
    bundled = "/snap/firefox/current/usr/lib/firefox/geckodriver"
    if os.path.isfile(bundled) and os.access(bundled, os.X_OK):
        return bundled
    raise FileNotFoundError(
        "Snap geckodriver proxy was found, but its native executable is unavailable"
    )


def native_firefox(value: str) -> str:
    """Pair native geckodriver with Firefox's real Snap-packaged binary."""
    resolved = executable(value, "Firefox")
    if os.path.realpath(resolved) != "/usr/bin/snap":
        return resolved
    bundled = "/snap/firefox/current/usr/lib/firefox/firefox"
    if os.path.isfile(bundled) and os.access(bundled, os.X_OK):
        return bundled
    raise FileNotFoundError(
        "Snap Firefox proxy was found, but its native executable is unavailable"
    )


def run_self_test() -> int:
    assert percentile([], 0.95) is None
    assert percentile([3.0, float("nan"), 1.0, 2.0], 0.50) == 2.0
    assert percentile([1.0, 2.0, 3.0, 4.0], 0.95) == 4.0

    line = (
        "[depth] 9.5Hz mode=preview capture=51ms render=37.5ms "
        "scene=37ms age=149ms"
    )
    match = DEPTH_RE.search(line)
    assert match is not None
    assert match.groupdict() == {"capture": "51", "render": "37.5", "age": "149"}

    unsafe = "failed https://person:password@example.test/a/b?key=example-secret#part"
    safe = sanitize_text(unsafe)
    assert safe == "failed https://example.test/a/b"
    assert "password" not in safe and "example-secret" not in safe
    assert sanitize_text("key=example-secret") == "key=<redacted>"
    assert loopback_no_proxy("example.test, localhost,example.test") == (
        "example.test,localhost,127.0.0.1,::1"
    )

    class FakeOpener:
        def __init__(self, response=None, error=None):
            self.response = response
            self.error = error
            self.requests: list[urllib.request.Request] = []

        def open(self, request, timeout):
            assert timeout == 30.0
            self.requests.append(request)
            if self.error:
                raise self.error
            return self.response

    session_response = io.BytesIO(json.dumps({
        "value": {"sessionId": "self-test-session", "capabilities": {}},
    }).encode("utf-8"))
    driver = WebDriver("http://127.0.0.1:4444")
    driver.opener = FakeOpener(response=session_response)
    driver.create("/snap/bin/firefox")
    session_request = driver.opener.requests[0]
    session_payload = json.loads(session_request.data.decode("utf-8"))
    assert session_request.get_method() == "POST"
    assert session_request.full_url == "http://127.0.0.1:4444/session"
    assert session_payload["capabilities"]["alwaysMatch"]["moz:firefoxOptions"][
        "binary"
    ] == "/snap/bin/firefox"
    prefs = session_payload["capabilities"]["alwaysMatch"]["moz:firefoxOptions"][
        "prefs"
    ]
    assert prefs == {"network.proxy.type": 5}

    error_body = io.BytesIO(json.dumps({
        "value": {
            "error": "javascript error",
            "message": unsafe,
        },
    }).encode("utf-8"))
    http_error = urllib.error.HTTPError(
        "http://127.0.0.1:4444/session/self-test/execute/sync",
        500,
        "Internal Server Error",
        {},
        error_body,
    )
    error_driver = WebDriver("http://127.0.0.1:4444")
    error_driver.opener = FakeOpener(error=http_error)
    try:
        error_driver.request("POST", "/session/self-test/execute/sync", {})
    except RuntimeError as exc:
        message = str(exc)
        assert message.endswith("failed https://example.test/a/b")
        assert "password" not in message and "example-secret" not in message
    else:
        raise AssertionError("WebDriver HTTP errors must fail closed")

    print("benchmark_firefox_preview self-test: PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8080/?panoProfile=flight&panoPreloadRequired=0")
    parser.add_argument("--firefox", default=os.environ.get("FIREFOX_BIN", "/snap/bin/firefox"))
    parser.add_argument("--geckodriver", default=os.environ.get("GECKODRIVER_BIN", "geckodriver"))
    parser.add_argument("--sample-seconds", type=float, default=30.0)
    parser.add_argument("--startup-timeout", type=float, default=240.0)
    parser.add_argument("--warmup-depth-samples", type=int, default=5)
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="test protocol/parser/redaction helpers without launching Firefox",
    )
    args = parser.parse_args()

    if args.self_test:
        return run_self_test()
    if not math.isfinite(args.sample_seconds) or args.sample_seconds < 5:
        parser.error("--sample-seconds must be finite and at least 5")
    if not math.isfinite(args.startup_timeout) or args.startup_timeout <= 0:
        parser.error("--startup-timeout must be finite and greater than zero")
    if args.warmup_depth_samples < 0:
        parser.error("--warmup-depth-samples must be non-negative")

    firefox_binary = native_firefox(args.firefox)
    geckodriver_binary = native_geckodriver(args.geckodriver)

    port = free_port()
    env = os.environ.copy()
    env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
    env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
    env["MOZ_X11_EGL"] = "0"
    env["LIBGL_ALWAYS_SOFTWARE"] = "0"
    env["no_proxy"] = loopback_no_proxy(env.get("no_proxy"))
    env["NO_PROXY"] = loopback_no_proxy(env.get("NO_PROXY"))
    env.pop("GTK_MODULES", None)
    gecko = subprocess.Popen(
        [
            geckodriver_binary,
            "--host", "127.0.0.1",
            "--port", str(port),
            "--log", "error",
        ],
        env=env,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    driver = WebDriver(f"http://127.0.0.1:{port}")
    try:
        deadline = time.monotonic() + 15
        last_status_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                status = driver.request("GET", "/status")
                if isinstance(status, dict) and status.get("ready") is True:
                    break
                last_status_error = RuntimeError("geckodriver reported not ready")
            except Exception as exc:
                last_status_error = exc
            if gecko.poll() is not None:
                raise RuntimeError(
                    f"geckodriver exited before becoming ready (status {gecko.returncode})"
                )
            else:
                time.sleep(0.2)
        else:
            detail = f": {sanitize_text(last_status_error)}" if last_status_error else ""
            raise RuntimeError(f"geckodriver did not become ready{detail}")

        driver.create(firefox_binary)
        driver.navigate(args.url)
        wait_until(
            driver,
            "return typeof window.googleTilesFlightStart === 'function';",
            args.startup_timeout,
            "application shell",
        )
        driver.execute(
            """
            window.__previewBenchLogs = [];
            const originalLog = console.log.bind(console);
            console.log = (...args) => {
                const line = typeof args[0] === 'string' ? args[0] : '';
                if (line.startsWith('[depth]')) window.__previewBenchLogs.push(line);
                if (window.__previewBenchLogs.length > 3000) window.__previewBenchLogs.shift();
                originalLog(...args);
            };
            window.__previewBenchRestoreConsole = () => { console.log = originalLog; };
            window.googleTilesFlightStart();
            return true;
            """
        )
        wait_until(
            driver,
            """
            const overlay = document.getElementById('placement-overlay');
            const coords = document.getElementById('spawn-coords');
            const loading = document.getElementById('loading-overlay');
            return overlay?.classList.contains('visible') === true
                && (coords?.textContent || '').trim().startsWith('Spawn:')
                && loading?.classList.contains('visible') !== true;
            """,
            args.startup_timeout,
            "placement mode and automatic spawn",
        )
        driver.execute(
            """
            window.dispatchEvent(new KeyboardEvent('keydown', {code:'KeyO', key:'o', bubbles:true}));
            window.dispatchEvent(new KeyboardEvent('keyup', {code:'KeyO', key:'o', bubbles:true}));
            return true;
            """
        )
        wait_until(
            driver,
            "return document.getElementById('view-choice-overlay')?.classList.contains('visible') === true;",
            args.startup_timeout,
            "flight preload and view selection",
        )
        driver.execute(
            """
            const button = document.querySelector('[data-view-choice="first"]');
            if (!button) throw new Error('first-person view button is missing');
            button.click();
            window.__previewBenchLogs = [];
            return true;
            """
        )
        wait_until(
            driver,
            """
            const overlay = document.getElementById('view-choice-overlay');
            const hud = document.getElementById('hud');
            return !!overlay && !overlay.classList.contains('visible')
                && !!hud && !hud.classList.contains('hidden')
                && !!window.__drone
                && typeof window.__getPerceptionFrame === 'function';
            """,
            10,
            "first-person flight mode",
        )
        if args.warmup_depth_samples:
            wait_until(
                driver,
                f"return (window.__previewBenchLogs || []).length >= {args.warmup_depth_samples};",
                args.startup_timeout,
                f"{args.warmup_depth_samples} preview warmup samples",
            )
        driver.execute(
            """
            // Exclude placement/preload/warmup work from the measured window.
            window.__previewBenchLogs = [];
            window.__previewBenchFrames = [];
            window.__previewBenchRunning = true;
            let previous = performance.now();
            const sample = now => {
                if (!window.__previewBenchRunning) return;
                window.__previewBenchFrames.push(now - previous);
                previous = now;
                requestAnimationFrame(sample);
            };
            requestAnimationFrame(sample);
            return true;
            """
        )
        time.sleep(args.sample_seconds)
        result = driver.execute(
            """
            window.__previewBenchRunning = false;
            const result = {
                logs: [...(window.__previewBenchLogs || [])],
                frameDeltas: [...(window.__previewBenchFrames || [])],
                perceptionReady: window.__getPerceptionFrame
                    ? !!window.__getPerceptionFrame()
                    : false
            };
            window.__previewBenchRestoreConsole?.();
            delete window.__previewBenchRestoreConsole;
            return result;
            """
        )

        if not isinstance(result, dict):
            raise RuntimeError("benchmark page returned an invalid result")
        if result.get("perceptionReady") is not True:
            raise RuntimeError("preview never produced a perception frame")

        captures: list[float] = []
        renders: list[float] = []
        ages: list[float] = []
        for line in result.get("logs", []):
            match = DEPTH_RE.search(str(line))
            if not match:
                continue
            captures.append(float(match.group("capture")))
            renders.append(float(match.group("render")))
            ages.append(float(match.group("age")))
        frame_deltas = [float(value) for value in result.get("frameDeltas", [])[5:]]

        capture_p50 = percentile(captures, 0.50)
        capture_p95 = percentile(captures, 0.95)
        render_p95 = percentile(renders, 0.95)
        age_p95 = percentile(ages, 0.95)
        frame_p95 = percentile(frame_deltas, 0.95)
        print(f"samples: depth={len(captures)} raf={len(frame_deltas)}")
        print(f"capture p50/p95: {format_metric(capture_p50)} / {format_metric(capture_p95)}")
        print(f"render p95:       {format_metric(render_p95)}")
        print(f"preview age p95:  {format_metric(age_p95)}")
        print(f"RAF/physics-frame p95: {format_metric(frame_p95)}")

        enough = len(captures) >= 10 and len(frame_deltas) >= 100
        passed = enough and capture_p95 is not None and frame_p95 is not None \
            and capture_p95 <= 60.0 and frame_p95 <= 33.3
        print(
            f"target: {'PASS' if passed else 'NOT MET'} "
            "(capture p95<=60ms, RAF/physics-frame p95<=33.3ms)"
        )
        return 0 if passed else 2
    finally:
        driver.close()
        if gecko.poll() is None:
            gecko.terminate()
            try:
                gecko.wait(timeout=5)
            except subprocess.TimeoutExpired:
                gecko.kill()
                gecko.wait(timeout=5)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("benchmark interrupted", file=sys.stderr)
        sys.exit(130)
    except Exception as exc:
        print(f"benchmark failed: {sanitize_text(exc)}", file=sys.stderr)
        sys.exit(1)
