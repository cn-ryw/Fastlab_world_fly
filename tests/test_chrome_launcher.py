"""Static safety and fallback contracts for the dedicated Chrome launcher."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "launch-chrome-gpu.sh").read_text(encoding="utf-8")


def test_chrome_launcher_keeps_persistent_non_incognito_profile():
    assert '"--user-data-dir=${PROFILE_DIR}"' in SOURCE
    assert "--incognito" not in SOURCE
    assert "--disable-extensions" in SOURCE
    assert "--disable-background-mode" in SOURCE


def test_chrome_launcher_defaults_to_guarded_nvidia_prime():
    assert 'CHROME_GPU_MODE:-auto' in SOURCE
    assert "auto|nvidia|desktop" in SOURCE
    assert "__NV_PRIME_RENDER_OFFLOAD=1" in SOURCE
    assert '__NV_PRIME_RENDER_OFFLOAD_PROVIDER="${provider}"' in SOURCE
    assert "__GLX_VENDOR_LIBRARY_NAME=nvidia" in SOURCE
    assert "NVIDIA-G0" in SOURCE
    assert "probe_nvidia_launch" in SOURCE
    assert "desktop fallback" in SOURCE
    assert "clear_stale_profile_singletons" in SOURCE
    assert "SingletonLock" in SOURCE
    assert "profile_lock_pid" in SOURCE
    assert "profile_tree_pids" in SOURCE


def test_chrome_launcher_uses_safe_x11_angle_gl_contract():
    required = (
        "--ozone-platform=x11",
        "--use-gl=angle",
        "--use-angle=gl",
        "--no-first-run",
        "--no-default-browser-check",
        "--disable-sync",
        "--disable-default-apps",
    )
    for argument in required:
        assert argument in SOURCE

    forbidden = (
        "--use-angle=vulkan",
        "--ignore-gpu-blocklist",
        "--disable-gpu-sandbox",
        "--disable-gpu-driver-bug-workaround",
        "--disable-gpu-watchdog",
    )
    for argument in forbidden:
        assert argument not in SOURCE


def test_chrome_launcher_detects_known_gpu_failures_before_fallback():
    for marker in (
        "Invalid visual ID requested",
        "GLDisplayEGL::Initialize failed",
        "use-gl=disabled",
        "SwiftShader",
    ):
        assert marker in SOURCE
    assert "stop_profile_processes" in SOURCE
    assert "CHROME_PROFILE_DIR" in SOURCE
