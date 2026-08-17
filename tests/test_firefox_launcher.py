"""Persistence and credential-safety contracts for the Firefox launcher."""

import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "launch-firefox-gpu.sh").read_text(encoding="utf-8")


def test_firefox_launcher_uses_a_dedicated_persistent_profile():
    assert "FIREFOX_PROFILE_DIR" in SOURCE
    assert 'mkdir -p -- "${PROFILE_DIR}"' in SOURCE
    assert 'chmod 700 -- "${PROFILE_DIR}"' in SOURCE
    assert "must be an absolute path" in SOURCE
    assert "realpath --canonicalize-missing" in SOURCE
    assert "must not be the filesystem root or home directory" in SOURCE
    assert "must not be a symbolic link" in SOURCE
    assert '--profile "${PROFILE_DIR}" --new-window "${TARGET_URL}"' in SOURCE
    assert "persistent, non-private" in SOURCE
    assert "--private-window" not in SOURCE
    assert "--new-instance" not in SOURCE
    assert "--no-remote" not in SOURCE


def test_firefox_launcher_does_not_inject_cesium_credentials_on_command_line():
    assert "mindcloud_cesium_ion_token" not in SOURCE
    assert "ionToken" not in SOURCE
    assert "CESIUM_ION_TOKEN" not in SOURCE


def run_with_profile(profile):
    environment = os.environ.copy()
    environment.update({
        "DISPLAY": ":test",
        "FIREFOX_BIN": "/bin/true",
        "FIREFOX_PROFILE_DIR": profile,
        "http_proxy": "http://127.0.0.1:9",
        "https_proxy": "http://127.0.0.1:9",
        "all_proxy": "http://127.0.0.1:9",
        "HTTP_PROXY": "http://127.0.0.1:9",
        "HTTPS_PROXY": "http://127.0.0.1:9",
        "ALL_PROXY": "http://127.0.0.1:9",
    })
    return subprocess.run(
        [str(ROOT / "launch-firefox-gpu.sh")],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        timeout=5,
        check=False,
    )


def test_firefox_launcher_rejects_broad_or_relative_profile_targets():
    for profile in ("/", os.environ["HOME"] + "/", "relative-profile"):
        result = run_with_profile(profile)
        assert result.returncode != 0
        assert "FIREFOX_PROFILE_DIR" in result.stderr


def test_firefox_launcher_rejects_a_profile_symlink(tmp_path):
    target = tmp_path / "target"
    target.mkdir()
    link = tmp_path / "profile-link"
    link.symlink_to(target, target_is_directory=True)
    result = run_with_profile(str(link))
    assert result.returncode != 0
    assert "must not be a symbolic link" in result.stderr
