#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MODE="${DA360_MODE:-docker}"
IMAGE="${DA360_IMAGE:-mindcloud-da360:latest}"
NAME="${DA360_CONTAINER_NAME:-mindcloud-da360-api}"
PORT="${DA360_PORT:-5688}"
MODEL_NAME="${1:-${DA360_MODEL:-large}}"
MODEL_PATH="${DA360_MODEL_PATH:-$PROJECT_ROOT/third_party/DA360/checkpoints/DA360_${MODEL_NAME}.pth}"
BASE_IMAGE="${DA360_BASE_IMAGE:-pytorch/pytorch:2.1.1-cuda12.1-cudnn8-runtime}"
BUILD_NETWORK="${DA360_BUILD_NETWORK:-host}"
BUILD_RETRIES="${DA360_BUILD_RETRIES:-3}"
RESAMPLE="${DA360_RESAMPLE:-bicubic}"
MOUNT_SERVER="${DA360_MOUNT_SERVER:-1}"
if ! [[ "$BUILD_RETRIES" =~ ^[0-9]+$ ]] || (( BUILD_RETRIES < 1 )); then
    BUILD_RETRIES=1
fi
SERVER_SHA="$(sha256sum "$SCRIPT_DIR/da360_server.py" | awk '{print $1}')"

# Docker's default bridge cannot reach a localhost proxy on the host.
build_args=(
    --pull=false
)
FORWARDED_PROXY_BUILD_ARGS=0
if [[ -n "$BUILD_NETWORK" ]]; then
    build_args+=(--network "$BUILD_NETWORK")
fi

add_proxy_build_arg() {
    local name="$1"
    local value="$2"

    if [[ -n "$value" ]]; then
        build_args+=(--build-arg "$name=$value")
        FORWARDED_PROXY_BUILD_ARGS=1
    fi
}

add_proxy_build_arg_pair() {
    local upper_name="$1"
    local lower_name="$2"
    local upper_value="${!upper_name:-}"
    local lower_value="${!lower_name:-}"

    add_proxy_build_arg "$upper_name" "${upper_value:-$lower_value}"
    add_proxy_build_arg "$lower_name" "${lower_value:-$upper_value}"
}

add_proxy_build_arg_pair_if_unset() {
    local upper_name="$1"
    local lower_name="$2"
    local value="$3"
    local upper_value="${!upper_name:-}"
    local lower_value="${!lower_name:-}"

    if [[ -z "$upper_value" && -z "$lower_value" ]]; then
        add_proxy_build_arg "$upper_name" "$value"
        add_proxy_build_arg "$lower_name" "$value"
    fi
}

detect_git_proxy() {
    command -v git >/dev/null 2>&1 || return 0

    git config --get http.https://github.com.proxy 2>/dev/null \
        || git config --get http.proxy 2>/dev/null \
        || true
}

add_proxy_build_arg_pair HTTP_PROXY http_proxy
add_proxy_build_arg_pair HTTPS_PROXY https_proxy
add_proxy_build_arg_pair FTP_PROXY ftp_proxy
add_proxy_build_arg_pair ALL_PROXY all_proxy
add_proxy_build_arg_pair NO_PROXY no_proxy

HOST_PROXY="${DA360_BUILD_PROXY:-}"
if [[ -z "$HOST_PROXY" ]]; then
    HOST_PROXY="$(detect_git_proxy)"
fi
if [[ -n "$HOST_PROXY" ]]; then
    add_proxy_build_arg_pair_if_unset HTTP_PROXY http_proxy "$HOST_PROXY"
    add_proxy_build_arg_pair_if_unset HTTPS_PROXY https_proxy "$HOST_PROXY"
    add_proxy_build_arg_pair_if_unset ALL_PROXY all_proxy "$HOST_PROXY"
fi

image_label() {
    docker image inspect --format "{{ index .Config.Labels \"$1\" }}" "$IMAGE" 2>/dev/null || true
}

image_server_sha_from_file() {
    docker run --rm --entrypoint sha256sum "$IMAGE" /opt/mindcloud-da360/scripts/da360_server.py 2>/dev/null \
        | awk '{print $1}' \
        || true
}

if [[ ! -s "$MODEL_PATH" ]]; then
    if [[ -n "${DA360_MODEL_PATH:-}" ]]; then
        echo "DA360_MODEL_PATH does not exist: $MODEL_PATH" >&2
        exit 1
    fi
    "$SCRIPT_DIR/download_da360_model.sh" "$MODEL_NAME"
fi

MODEL_PATH="$(readlink -f "$MODEL_PATH")"
MODEL_BASENAME="$(basename "$MODEL_PATH")"
MODEL_SHA256="${DA360_MODEL_SHA256:-$(sha256sum "$MODEL_PATH" | awk '{print $1}')}"
CALIBRATION_PATH="${DA360_DEPTH_CALIB_PATH_HOST:-${DA360_DEPTH_CALIB_PATH:-}}"
if [[ -n "$CALIBRATION_PATH" ]]; then
    [[ -s "$CALIBRATION_PATH" ]] || {
        echo "DA360 calibration does not exist: $CALIBRATION_PATH" >&2
        exit 1
    }
    CALIBRATION_PATH="$(readlink -f "$CALIBRATION_PATH")"
fi
case "${DA360_DEPTH_MODE:-da360-relative}" in
    metric|da360-metric)
        [[ -n "$CALIBRATION_PATH" ]] || {
            echo "DA360 metric mode requires DA360_DEPTH_CALIB_PATH_HOST" >&2
            exit 1
        }
        ;;
    relative|da360-relative) ;;
    *)
        echo "DA360_DEPTH_MODE must be da360-relative or da360-metric" >&2
        exit 1
        ;;
esac

if [[ "$MODE" == "local" ]]; then
    PYTHON_BIN="${DA360_PYTHON:-python3}"
    DA360_RESAMPLE="$RESAMPLE" \
    DA360_MODEL_SHA256="$MODEL_SHA256" \
    DA360_DEPTH_CALIB_PATH="$CALIBRATION_PATH" \
    exec "$PYTHON_BIN" "$SCRIPT_DIR/da360_server.py" --model-path "$MODEL_PATH" --port "$PORT"
fi

command -v docker >/dev/null 2>&1 || {
    echo "Docker is required for DA360_MODE=docker." >&2
    exit 1
}

docker info >/dev/null 2>&1 || {
    echo "Cannot access Docker daemon." >&2
    exit 1
}

build_ok=0
if [[ "${DA360_FORCE_BUILD:-0}" != "1" && "$MOUNT_SERVER" == "1" ]] &&
    docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Using existing DA360 image $IMAGE and mounting the current server script; skipping rebuild."
    build_ok=1
else
    existing_server_sha="$(image_label "mindcloud.da360.server_sha")"
    if [[ "${DA360_FORCE_BUILD:-0}" != "1" && "$existing_server_sha" != "$SERVER_SHA" ]] &&
        docker image inspect "$IMAGE" >/dev/null 2>&1; then
        existing_server_sha="$(image_server_sha_from_file)"
    fi
    if [[ "${DA360_FORCE_BUILD:-0}" != "1" && "$existing_server_sha" == "$SERVER_SHA" ]]; then
        echo "DA360 image $IMAGE already contains the current server script; skipping rebuild."
        build_ok=1
    else
        if [[ -n "$BUILD_NETWORK" ]]; then
            echo "Using Docker build network: $BUILD_NETWORK"
        fi
        if [[ "$FORWARDED_PROXY_BUILD_ARGS" == "1" ]]; then
            echo "Forwarding host proxy environment to Docker build."
        fi
        for ((attempt = 1; attempt <= BUILD_RETRIES; attempt++)); do
            if docker build "${build_args[@]}" \
                --build-arg "DA360_BASE_IMAGE=$BASE_IMAGE" \
                --build-arg "DA360_SERVER_SHA=$SERVER_SHA" \
                -f "$PROJECT_ROOT/Dockerfile.da360" \
                -t "$IMAGE" \
                "$PROJECT_ROOT"; then
                build_ok=1
                break
            fi
            if (( attempt < BUILD_RETRIES )); then
                echo "WARNING: DA360 image build failed; retrying ($attempt/$BUILD_RETRIES)..." >&2
                sleep 2
            fi
        done
        if [[ "$build_ok" != "1" ]] && docker image inspect "$IMAGE" >/dev/null 2>&1; then
            existing_server_sha="$(image_server_sha_from_file)"
            if [[ "$existing_server_sha" == "$SERVER_SHA" ]]; then
                echo "WARNING: failed to rebuild $IMAGE, but the existing local image contains the current server script; using it." >&2
                build_ok=1
            fi
        fi
    fi
fi

if [[ "$build_ok" != "1" ]]; then
    if [[ "${DA360_ALLOW_STALE_IMAGE:-0}" == "1" ]] && docker image inspect "$IMAGE" >/dev/null 2>&1; then
        echo "WARNING: failed to rebuild $IMAGE; DA360_ALLOW_STALE_IMAGE=1, using the existing local image." >&2
    else
        echo "ERROR: failed to build $IMAGE from $BASE_IMAGE." >&2
        echo "Not starting a stale DA360 container because the frontend/server protocol may be incompatible." >&2
        echo "If Docker Hub timed out, retry this command after the network recovers, or set DA360_BASE_IMAGE to a local/mirror image." >&2
        docker rm -f "$NAME" >/dev/null 2>&1 || true
        exit 1
    fi
fi
docker rm -f "$NAME" >/dev/null 2>&1 || true

gpu_args=()
if [[ "${DA360_GPUS:-all}" != "none" ]]; then
    gpu_args=(--gpus "${DA360_GPUS:-all}")
fi

run_args=(
    --rm
    --name "$NAME"
    -p "127.0.0.1:$PORT:5688"
    -e "DA360_NO_WARMUP=${DA360_NO_WARMUP:-0}"
    -e "DA360_INPUT_SCALE=${DA360_INPUT_SCALE:-0.46}"
    -e "DA360_INPUT_WIDTH=${DA360_INPUT_WIDTH:-0}"
    -e "DA360_INPUT_HEIGHT=${DA360_INPUT_HEIGHT:-0}"
    -e "DA360_RESAMPLE=$RESAMPLE"
    -e "DA360_OUTPUT_FORMAT=${DA360_OUTPUT_FORMAT:-jpeg}"
    -e "DA360_JPEG_QUALITY=${DA360_JPEG_QUALITY:-72}"
    -e "DA360_AMP=${DA360_AMP:-1}"
    -e "DA360_CHANNELS_LAST=${DA360_CHANNELS_LAST:-0}"
    -e "DA360_ALLOWED_ORIGINS=${DA360_ALLOWED_ORIGINS:-http://127.0.0.1:8080,http://localhost:8080}"
    -e "DA360_MAX_CONTENT_LENGTH=${DA360_MAX_CONTENT_LENGTH:-8388608}"
    -e "DA360_DEPTH_MODE=${DA360_DEPTH_MODE:-da360-relative}"
    -e "DA360_MODEL_SHA256=$MODEL_SHA256"
    -e "DA360_TORCH_COMPILE=${DA360_TORCH_COMPILE:-0}"
    -v "$MODEL_PATH:/models/$MODEL_BASENAME:ro"
)

if [[ -n "$CALIBRATION_PATH" ]]; then
    run_args+=(
        -v "$CALIBRATION_PATH:/opt/calibration/depth_calibration.json:ro"
        -e "DA360_DEPTH_CALIB_PATH=/opt/calibration/depth_calibration.json"
    )
fi

if [[ "$MOUNT_SERVER" == "1" ]]; then
    run_args+=(-v "$SCRIPT_DIR/da360_server.py:/opt/mindcloud-da360/scripts/da360_server.py:ro")
fi

if [[ "${DA360_DETACH:-1}" == "1" ]]; then
    run_args=(-d "${run_args[@]}")
fi

exec docker run "${gpu_args[@]}" "${run_args[@]}" "$IMAGE" \
    python /opt/mindcloud-da360/scripts/da360_server.py \
        --model-path "/models/$MODEL_BASENAME" \
        --host 0.0.0.0 \
        --port 5688
