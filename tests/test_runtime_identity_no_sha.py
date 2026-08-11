"""Regression checks for the user-approved hash-free runtime identity contract."""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_launch_and_service_runtime_do_not_reintroduce_content_hashes():
    runtime_files = (
        "start-all.sh",
        "Dockerfile.da360",
        "Dockerfile.da360-yopo",
        "scripts/start_da360_api.sh",
        "scripts/combined_server.py",
        "scripts/da360_server.py",
        "scripts/fit_da360_metric.py",
        "scripts/yopo_bridge.py",
        "src/panorama-sensor.js",
    )
    forbidden = ("sha256", "sha-256", "service_fingerprint")
    for relative in runtime_files:
        source = (ROOT / relative).read_text(encoding="utf-8").lower()
        for token in forbidden:
            assert token not in source, f"{relative} reintroduced {token}"


def test_dependency_manifest_contains_readable_versions_without_digests():
    manifest_path = ROOT / "dependencies.versions.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    serialized = json.dumps(manifest, sort_keys=True).lower()
    assert "sha256" not in serialized
    assert "@sha" not in serialized
    assert manifest["runtime_dependencies"]["cesium"]["version"] == "1.117"
    assert manifest["runtime_dependencies"]["playcanvas"]["version"] == "2.17.2"
    assert manifest["runtime_dependencies"]["da360"]["repository"].endswith(
        "/Insta360-Research-Team/DA360.git"
    )
    assert manifest["runtime_dependencies"]["da360"]["required_files"]
    assert manifest["runtime_dependencies"]["yopo"]["repository"].endswith(
        "/zwhhhhh9/YOPO_360.git"
    )
    assert manifest["runtime_dependencies"]["yopo"]["required_files"]
    assert manifest["container_images"]["yopo_base"].endswith(
        ":sim-u2004-noetic-py38"
    )
