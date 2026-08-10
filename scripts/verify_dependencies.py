#!/usr/bin/env python3
"""Verify ignored runtime assets and model files against dependencies.lock.json."""

import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LOCK_PATH = ROOT / "dependencies.lock.json"
COMBINED_DOCKERFILE = ROOT / "Dockerfile.da360-yopo"


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


def _docker_arg_default(dockerfile_text, name):
    match = re.search(
        rf"^ARG[ \t]+{re.escape(name)}=([^\s#]+)[ \t]*$",
        dockerfile_text,
        flags=re.MULTILINE,
    )
    return match.group(1) if match else None


def verify(
    lock,
    da360_model=None,
    yopo_model=None,
    skip_checkpoints=False,
    lock_path=LOCK_PATH,
):
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

    yopo_base_config_path = ROOT / yopo_config["base_local_path"]
    if not yopo_base_config_path.is_file():
        errors.append(f"YOPO base config missing: {yopo_base_config_path}")
    else:
        require_equal(
            "YOPO base config size",
            yopo_base_config_path.stat().st_size,
            yopo_config["base_size_bytes"],
            errors,
        )
        require_equal(
            "YOPO base config sha256",
            sha256_file(yopo_base_config_path),
            yopo_config["base_sha256"],
            errors,
        )

    timm_wheel = runtime["timm_wheel"]
    timm_wheel_path = ROOT / timm_wheel["local_path"]
    if not timm_wheel_path.is_file():
        errors.append(f"timm wheel missing: {timm_wheel_path}")
    else:
        require_equal("timm wheel size", timm_wheel_path.stat().st_size, timm_wheel["size_bytes"], errors)
        require_equal("timm wheel sha256", sha256_file(timm_wheel_path), timm_wheel["sha256"], errors)

    base_image = lock["container_images"]["yopo_base"]
    base_reference = base_image["reference"]
    if not re.fullmatch(r"[^\s@]+@sha256:[0-9a-f]{64}", base_reference):
        errors.append(f"YOPO base image is not digest-pinned: {base_reference!r}")
    if not re.fullmatch(r"sha256:[0-9a-f]{64}", base_image.get("local_image_id", "")):
        errors.append("YOPO base local_image_id is not a sha256 image ID")
    if not COMBINED_DOCKERFILE.is_file():
        errors.append(f"combined Dockerfile missing: {COMBINED_DOCKERFILE}")
    else:
        dockerfile_text = COMBINED_DOCKERFILE.read_text(encoding="utf-8")
        require_equal(
            "Dockerfile YOPO_BASE_IMAGE",
            _docker_arg_default(dockerfile_text, "YOPO_BASE_IMAGE"),
            base_reference,
            errors,
        )
        require_equal(
            "Dockerfile dependency lock sha256",
            _docker_arg_default(dockerfile_text, "MINDCLOUD_DEPENDENCY_LOCK_SHA256"),
            sha256_file(lock_path),
            errors,
        )
        for label in (
            'mindcloud.yopo_base_image="${YOPO_BASE_IMAGE}"',
            'mindcloud.dependencies_lock_sha256="${MINDCLOUD_DEPENDENCY_LOCK_SHA256}"',
            'mindcloud.image_recipe_sha256="${MINDCLOUD_IMAGE_RECIPE_SHA256}"',
        ):
            if label not in dockerfile_text:
                errors.append(f"Dockerfile runtime identity label missing: {label}")

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
    errors = verify(
        lock,
        args.da360_model,
        args.yopo_model,
        args.skip_checkpoints,
        args.lock,
    )
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print("Dependency lock verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
