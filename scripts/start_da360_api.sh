#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${DA360_IMAGE:-mindcloud-da360:latest}"
NAME="${DA360_CONTAINER_NAME:-mindcloud-da360-api}"
PORT="${DA360_PORT:-5688}"
MODEL_PATH="${DA360_MODEL_PATH_HOST:-$PROJECT_ROOT/models/da360/DA360_large.pth}"
DEPTH_MODE="${DA360_DEPTH_MODE:-da360-relative}"
CALIBRATION_PATH="${DA360_DEPTH_CALIB_PATH_HOST:-}"

die() { echo "ERROR: $*" >&2; exit 1; }
command -v docker >/dev/null 2>&1 || die "docker is required"
docker info >/dev/null 2>&1 || die "Docker daemon is unavailable"
[[ -s "$MODEL_PATH" ]] || die "DA360 checkpoint missing; run scripts/bootstrap.sh autonomy"
MODEL_PATH="$(readlink -f "$MODEL_PATH")"

case "$DEPTH_MODE" in
  relative|da360-relative) DEPTH_MODE=da360-relative ;;
  metric|da360-metric)
    DEPTH_MODE=da360-metric
    [[ -s "$CALIBRATION_PATH" ]] || die "metric mode requires DA360_DEPTH_CALIB_PATH_HOST"
    CALIBRATION_PATH="$(readlink -f "$CALIBRATION_PATH")"
    ;;
  *) die "DA360_DEPTH_MODE must be da360-relative or da360-metric" ;;
esac

python3 "$PROJECT_ROOT/scripts/verify_dependencies.py" \
  --skip-web --skip-checkpoints \
  || die "DA360 source verification failed"

base_image="$(python3 - "$PROJECT_ROOT/dependencies.lock.json" <<'PY'
import json, sys
with open(sys.argv[1], encoding="utf-8") as stream:
    print(json.load(stream)["container_image"]["base"])
PY
)"
docker build \
  --file "$PROJECT_ROOT/Dockerfile.da360" \
  --tag "$IMAGE" \
  --build-arg "DA360_BASE_IMAGE=$base_image" \
  "$PROJECT_ROOT"

model_sha="$(sha256sum "$MODEL_PATH" | awk '{print $1}')"
docker rm -f "$NAME" >/dev/null 2>&1 || true
run_args=(
  --rm -d --init --name "$NAME" --gpus "${DA360_GPUS:-all}"
  -p "127.0.0.1:$PORT:5688"
  -v "$MODEL_PATH:/models/DA360_large.pth:ro"
  -e "DA360_MODEL_SHA256=$model_sha"
  -e "DA360_DEPTH_MODE=$DEPTH_MODE"
  -e "DA360_INPUT_SCALE=${DA360_INPUT_SCALE:-0.46}"
  -e "DA360_RESAMPLE=${DA360_RESAMPLE:-bicubic}"
  -e "DA360_ALLOWED_ORIGINS=${DA360_ALLOWED_ORIGINS:-http://127.0.0.1:8080,http://localhost:8080}"
)
if [[ -n "$CALIBRATION_PATH" ]]; then
  run_args+=(
    -v "$CALIBRATION_PATH:/opt/calibration/depth_calibration.json:ro"
    -e "DA360_DEPTH_CALIB_PATH=/opt/calibration/depth_calibration.json"
  )
fi
docker run "${run_args[@]}" "$IMAGE" >/dev/null
echo "DA360 API: http://127.0.0.1:$PORT/health"
