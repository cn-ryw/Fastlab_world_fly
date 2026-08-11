#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${MINDCLOUD_API_IMAGE:-mindcloud-da360-yopo:latest}"
LOCK="$PROJECT_ROOT/dependencies.lock.json"
RECIPE="$PROJECT_ROOT/Dockerfile.da360-yopo"

command -v docker >/dev/null 2>&1 || { echo "ERROR: docker is required" >&2; exit 1; }
python3 "$PROJECT_ROOT/scripts/verify_dependencies.py" --skip-web --skip-checkpoints

lock_fingerprint="$(sha256sum "$LOCK" | awk '{print $1}')"
recipe_fingerprint="$(sha256sum "$RECIPE" | awk '{print $1}')"
base_image="$(python3 - "$LOCK" <<'PY'
import json, sys
with open(sys.argv[1], encoding="utf-8") as stream:
    print(json.load(stream)["container_image"]["base"])
PY
)"

docker build \
  --file "$RECIPE" \
  --tag "$IMAGE" \
  --build-arg "AUTONOMY_BASE_IMAGE=$base_image" \
  --build-arg "MINDCLOUD_DEPENDENCY_LOCK_SHA256=$lock_fingerprint" \
  --build-arg "MINDCLOUD_IMAGE_RECIPE_SHA256=$recipe_fingerprint" \
  "$PROJECT_ROOT"

echo "Built $IMAGE"
