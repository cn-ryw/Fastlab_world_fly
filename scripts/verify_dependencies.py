#!/usr/bin/env python3
"""Validate declared dependency versions and required local paths without hashes."""

import argparse
import json
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MANIFEST_PATH = ROOT / "dependencies.versions.json"
DOCKERFILE = ROOT / "Dockerfile.da360-yopo"


def _require_file(path, label, errors):
    if not path.is_file() or path.stat().st_size <= 0:
        errors.append(f"{label} is missing or empty: {path}")


def verify(manifest, da360_model=None, yopo_model=None, skip_checkpoints=False):
    errors = []
    runtime = manifest["runtime_dependencies"]

    cesium = runtime["cesium"]
    cesium_entry = ROOT / cesium["entrypoint"]
    _require_file(cesium_entry, "Cesium entrypoint", errors)
    if cesium_entry.is_file():
        header = cesium_entry.read_text(encoding="utf-8", errors="ignore")[:2048]
        if f"Version {cesium['version']}" not in header:
            errors.append("Cesium entrypoint version does not match dependencies.versions.json")

    playcanvas = runtime["playcanvas"]
    playcanvas_path = ROOT / playcanvas["local_path"]
    _require_file(playcanvas_path, "PlayCanvas runtime", errors)
    if playcanvas_path.is_file():
        header = playcanvas_path.read_text(encoding="utf-8", errors="ignore")[:1024]
        expected = f"PlayCanvas Engine v{playcanvas['version']} revision {playcanvas['revision']}"
        if expected not in header:
            errors.append("PlayCanvas version does not match dependencies.versions.json")

    da360_path = ROOT / runtime["da360"]["local_path"]
    if not da360_path.is_dir():
        errors.append(f"DA360 source directory is missing: {da360_path}")
    for relative in runtime["da360"].get("required_files", ()):
        _require_file(da360_path / relative, "DA360 source file", errors)

    yopo = runtime["yopo"]
    yopo_path = ROOT / yopo["local_path"]
    if not yopo_path.is_dir():
        errors.append(f"YOPO source directory is missing: {yopo_path}")
    for relative in yopo.get("required_files", ()):
        _require_file(yopo_path / relative, "YOPO source file", errors)
    for field in ("base_config", "profile"):
        _require_file(ROOT / yopo[field], f"YOPO {field}", errors)
    for wheel in runtime["wheels"]:
        _require_file(ROOT / wheel, "Python wheel", errors)

    expected_base = manifest["container_images"]["yopo_base"]
    if not re.fullmatch(r"[^\s@:]+(?:/[^\s@:]+)+:[A-Za-z0-9._-]+", expected_base):
        errors.append("YOPO base image must use a readable repository tag")
    _require_file(DOCKERFILE, "combined Dockerfile", errors)
    if DOCKERFILE.is_file():
        dockerfile = DOCKERFILE.read_text(encoding="utf-8")
        if f"ARG YOPO_BASE_IMAGE={expected_base}" not in dockerfile:
            errors.append("Dockerfile YOPO_BASE_IMAGE does not match dependencies.versions.json")

    if not skip_checkpoints:
        checkpoints = manifest["model_checkpoints"]
        model_paths = {
            "da360": Path(da360_model) if da360_model else ROOT / checkpoints["da360"]["default_host_path"],
            "yopo": Path(yopo_model) if yopo_model else Path(checkpoints["yopo"]["default_host_path"]),
        }
        for name, path in model_paths.items():
            _require_file(path, f"{name} checkpoint", errors)
            if path.name != checkpoints[name]["name"]:
                errors.append(f"{name} checkpoint filename must be {checkpoints[name]['name']}")
    return errors


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=MANIFEST_PATH)
    parser.add_argument("--da360-model", type=Path)
    parser.add_argument("--yopo-model", type=Path)
    parser.add_argument("--skip-checkpoints", action="store_true")
    args = parser.parse_args(argv)
    with args.manifest.open(encoding="utf-8") as stream:
        manifest = json.load(stream)
    errors = verify(manifest, args.da360_model, args.yopo_model, args.skip_checkpoints)
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print("Dependency versions verified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
