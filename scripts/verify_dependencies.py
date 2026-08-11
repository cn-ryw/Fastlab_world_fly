#!/usr/bin/env python3
"""Verify public runtime assets and optional local checkpoints."""

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
    digest = hashlib.sha256()
    for path in sorted(item for item in Path(directory).rglob("*") if item.is_file()):
        relative_display = path.relative_to(ROOT).as_posix()
        digest.update(f"{sha256_file(path)}  {relative_display}\n".encode())
    return digest.hexdigest()


def require_equal(label, actual, expected, errors):
    if actual != expected:
        errors.append(f"{label}: {actual!r} != expected {expected!r}")


def verify_file(label, path, metadata, errors):
    if not path.is_file():
        errors.append(f"{label} missing: {path}")
        return
    if "size_bytes" in metadata:
        require_equal(f"{label} size", path.stat().st_size, metadata["size_bytes"], errors)
    if "sha256" in metadata:
        require_equal(f"{label} sha256", sha256_file(path), metadata["sha256"], errors)


def verify(lock, skip_web=False, skip_cesium=False, skip_playcanvas=False,
           skip_source=False, skip_checkpoints=False,
           da360_model=None, yopo_model=None):
    errors = []
    runtime = lock["runtime_dependencies"]

    if not skip_web and not skip_cesium:
        cesium = runtime["cesium"]
        cesium_entry = ROOT / cesium["entrypoint"]
        verify_file(
            "Cesium entrypoint",
            cesium_entry,
            {
                "size_bytes": cesium["entrypoint_size_bytes"],
                "sha256": cesium["entrypoint_sha256"],
            },
            errors,
        )
        if cesium_entry.is_file():
            require_equal(
                "Cesium tree manifest sha256",
                cesium_tree_manifest_sha256(ROOT / cesium["local_path"]),
                cesium["tree_manifest_sha256"],
                errors,
            )
    if not skip_web and not skip_playcanvas:
        playcanvas = runtime["playcanvas"]
        verify_file("PlayCanvas", ROOT / playcanvas["local_path"], playcanvas, errors)

    if not skip_source:
        da360 = runtime["da360"]
        da360_dir = ROOT / da360["local_path"]
        try:
            revision = subprocess.check_output(
                ["git", "-C", str(da360_dir), "rev-parse", "HEAD"],
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
            require_equal("DA360 revision", revision, da360["commit"], errors)
        except (OSError, subprocess.CalledProcessError):
            errors.append(f"DA360 checkout unavailable: {da360_dir}")

    yopo_config = runtime["yopo_config"]
    verify_file("YOPO config", ROOT / yopo_config["local_path"], yopo_config, errors)
    verify_file(
        "YOPO base config",
        ROOT / yopo_config["base_local_path"],
        {"size_bytes": yopo_config["base_size_bytes"], "sha256": yopo_config["base_sha256"]},
        errors,
    )

    if not skip_checkpoints:
        checkpoints = lock["model_checkpoints"]
        paths = {
            "DA360 checkpoint": Path(da360_model) if da360_model else ROOT / checkpoints["da360_large"]["default_host_path"],
            "YOPO checkpoint": Path(yopo_model) if yopo_model else ROOT / checkpoints["yopo_epoch10"]["default_host_path"],
        }
        verify_file("DA360 checkpoint", paths["DA360 checkpoint"], checkpoints["da360_large"], errors)
        verify_file("YOPO checkpoint", paths["YOPO checkpoint"], checkpoints["yopo_epoch10"], errors)
    return errors


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lock", type=Path, default=LOCK_PATH)
    parser.add_argument("--skip-web", action="store_true")
    parser.add_argument("--skip-cesium", action="store_true")
    parser.add_argument("--skip-playcanvas", action="store_true")
    parser.add_argument("--skip-source", action="store_true")
    parser.add_argument("--skip-checkpoints", action="store_true")
    parser.add_argument("--da360-model", type=Path)
    parser.add_argument("--yopo-model", type=Path)
    args = parser.parse_args(argv)
    with args.lock.open(encoding="utf-8") as stream:
        lock = json.load(stream)
    errors = verify(
        lock,
        skip_web=args.skip_web,
        skip_cesium=args.skip_cesium,
        skip_playcanvas=args.skip_playcanvas,
        skip_source=args.skip_source,
        skip_checkpoints=args.skip_checkpoints,
        da360_model=args.da360_model,
        yopo_model=args.yopo_model,
    )
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print("Dependency manifest verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
