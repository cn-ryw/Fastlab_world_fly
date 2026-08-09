#!/usr/bin/env python3
"""Verify ignored runtime assets and model files against dependencies.lock.json."""

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LOCK_PATH = ROOT / "dependencies.lock.json"


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def cesium_tree_manifest_sha256(directory):
    """Match `find ... -print0 | sort -z | xargs -0 sha256sum | sha256sum`."""
    directory = Path(directory)
    digest = hashlib.sha256()
    for path in sorted(item for item in directory.rglob("*") if item.is_file()):
        file_digest = sha256_file(path)
        relative_display = path.relative_to(ROOT).as_posix()
        digest.update(f"{file_digest}  {relative_display}\n".encode("utf-8"))
    return digest.hexdigest()


def require_equal(label, actual, expected, errors):
    if actual != expected:
        errors.append(f"{label}: {actual!r} != locked {expected!r}")


def verify(lock, da360_model=None, yopo_model=None, skip_checkpoints=False):
    errors = []
    runtime = lock["runtime_dependencies"]

    cesium = runtime["cesium"]
    cesium_entry = ROOT / cesium["entrypoint"]
    if not cesium_entry.is_file():
        errors.append(f"Cesium entrypoint missing: {cesium_entry}")
    else:
        require_equal("Cesium entrypoint size", cesium_entry.stat().st_size, cesium["entrypoint_size_bytes"], errors)
        require_equal("Cesium entrypoint sha256", sha256_file(cesium_entry), cesium["entrypoint_sha256"], errors)
        require_equal(
            "Cesium tree manifest sha256",
            cesium_tree_manifest_sha256(ROOT / cesium["local_path"]),
            cesium["tree_manifest_sha256"],
            errors,
        )

    playcanvas = runtime["playcanvas"]
    playcanvas_path = ROOT / playcanvas["local_path"]
    if not playcanvas_path.is_file():
        errors.append(f"PlayCanvas bundle missing: {playcanvas_path}")
    else:
        require_equal("PlayCanvas size", playcanvas_path.stat().st_size, playcanvas["size_bytes"], errors)
        require_equal("PlayCanvas sha256", sha256_file(playcanvas_path), playcanvas["sha256"], errors)

    da360_dir = ROOT / runtime["da360"]["local_path"]
    try:
        revision = subprocess.check_output(
            ["git", "-C", str(da360_dir), "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL
        ).strip()
        require_equal("DA360 revision", revision, runtime["da360"]["commit"], errors)
    except (OSError, subprocess.CalledProcessError):
        errors.append(f"DA360 Git checkout unavailable: {da360_dir}")

    yopo_config = runtime["yopo_config"]
    yopo_config_path = ROOT / yopo_config["local_path"]
    if not yopo_config_path.is_file():
        errors.append(f"YOPO config missing: {yopo_config_path}")
    else:
        require_equal("YOPO config size", yopo_config_path.stat().st_size, yopo_config["size_bytes"], errors)
        require_equal("YOPO config sha256", sha256_file(yopo_config_path), yopo_config["sha256"], errors)

    if not skip_checkpoints:
        checkpoints = lock["model_checkpoints"]
        model_paths = {
            "da360_large": Path(da360_model) if da360_model else ROOT / checkpoints["da360_large"]["default_host_path"],
            "yopo_epoch10": Path(yopo_model) if yopo_model else Path(checkpoints["yopo_epoch10"]["default_host_path"]),
        }
        for name, path in model_paths.items():
            expected = checkpoints[name]
            if not path.is_file():
                errors.append(f"{name} checkpoint missing: {path}")
                continue
            require_equal(f"{name} size", path.stat().st_size, expected["size_bytes"], errors)
            require_equal(f"{name} sha256", sha256_file(path), expected["sha256"], errors)
    return errors


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lock", type=Path, default=LOCK_PATH)
    parser.add_argument("--da360-model", type=Path)
    parser.add_argument("--yopo-model", type=Path)
    parser.add_argument("--skip-checkpoints", action="store_true")
    args = parser.parse_args(argv)
    with args.lock.open(encoding="utf-8") as stream:
        lock = json.load(stream)
    errors = verify(lock, args.da360_model, args.yopo_model, args.skip_checkpoints)
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print("Dependency lock verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
