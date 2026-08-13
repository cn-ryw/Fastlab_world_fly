#!/usr/bin/env bash
# MindCloud World Fly — local-only service launcher
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PID_FILE="$SCRIPT_DIR/.dev-web.pid"
WEB_LOG="$SCRIPT_DIR/.dev-web.log"
WEB_HOST="${MINDCLOUD_WEB_HOST:-127.0.0.1}"
WEB_PORT="${MINDCLOUD_WEB_PORT:-8080}"
API_PORT="${MINDCLOUD_API_PORT:-5688}"
API_CONTAINER="${MINDCLOUD_API_CONTAINER:-mindcloud-da360-yopo}"
LEGACY_WEB_CONTAINER="google-tiles-flight"
API_IMAGE="${MINDCLOUD_API_IMAGE:-mindcloud-da360-yopo:latest}"
DA360_MODEL="${DA360_MODEL_PATH_HOST:-$SCRIPT_DIR/third_party/DA360/checkpoints/DA360_large.pth}"
YOPO_STRATEGY="${MINDCLOUD_YOPO_STRATEGY:-baseline}"
YOPO_BASE_CONFIG_PATH="$SCRIPT_DIR/third_party/YOPO/config/traj_opt.yaml"
DEPENDENCY_MANIFEST="$SCRIPT_DIR/dependencies.versions.json"
STARTUP_TIMEOUT="${MINDCLOUD_STARTUP_TIMEOUT:-180}"
DEFAULT_CALIBRATION_FILE="$SCRIPT_DIR/../experiment_data/depth_calibration.json"
DEPTH_MODE="${DA360_DEPTH_MODE:-da360-metric}"
RESAMPLE="${DA360_RESAMPLE:-bicubic}"
INPUT_SCALE="${DA360_INPUT_SCALE:-0.46}"
START_COMPLETE=0
API_STARTED=0
WEB_STARTED=0
WEB_PID=""

die() {
    echo "ERROR: $*" >&2
    exit 1
}

require_command() {
    command -v "$1" >/dev/null 2>&1 || die "missing required command: $1"
}

resolve_yopo_strategy() {
    local resolved default_model default_config
    if ! resolved="$(python3 - "$DEPENDENCY_MANIFEST" "$YOPO_STRATEGY" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    manifest = json.load(stream)
strategies = manifest.get("yopo_strategies", {})
strategy = strategies.get(sys.argv[2])
if strategy is None:
    available = ", ".join(sorted(strategies))
    raise SystemExit(f"unknown YOPO strategy {sys.argv[2]!r}; expected one of: {available}")
print(strategy["default_host_path"] + "\t" + strategy["config"])
PY
    )"; then
        die "failed to resolve YOPO strategy: $YOPO_STRATEGY"
    fi
    IFS=$'\t' read -r default_model default_config <<<"$resolved"
    if [[ "$default_model" != /* ]]; then
        default_model="$SCRIPT_DIR/$default_model"
    fi
    YOPO_MODEL="${YOPO_MODEL_PATH_HOST:-$default_model}"
    YOPO_CONFIG_NAME="${YOPO_CONFIG:-$default_config}"
    YOPO_CONFIG_PATH="$SCRIPT_DIR/third_party/YOPO/config/$YOPO_CONFIG_NAME"
}

is_owned_web_pid() {
    local pid="${1:-}"
    [[ "$pid" =~ ^[0-9]+$ ]] || return 1
    [[ -r "/proc/$pid/cmdline" ]] || return 1
    local argument process_cwd found=0
    while IFS= read -r -d '' argument; do
        [[ "$argument" == "$SCRIPT_DIR/scripts/serve.py" ]] && found=1
    done < "/proc/$pid/cmdline" 2>/dev/null || true
    process_cwd="$(readlink -f "/proc/$pid/cwd" 2>/dev/null || true)"
    [[ "$found" == "1" && "$process_cwd" == "$SCRIPT_DIR" ]]
}

find_owned_web_pids() {
    local proc pid
    for proc in /proc/[0-9]*; do
        pid="${proc##*/}"
        if is_owned_web_pid "$pid"; then
            echo "$pid"
        fi
    done
}

stop_owned_web_pid() {
    local pid="$1"
    is_owned_web_pid "$pid" || return 1
    kill -TERM "$pid"
    for _ in $(seq 1 50); do
        is_owned_web_pid "$pid" || return 0
        sleep 0.1
    done
    if is_owned_web_pid "$pid"; then
        echo "  Web PID $pid did not stop after 5s; sending KILL" >&2
        kill -KILL "$pid"
    fi
}

port_is_free() {
    python3 - "$1" "$2" <<'PY'
import socket
import sys

host, port = sys.argv[1], int(sys.argv[2])
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    sock.bind((host, port))
except OSError:
    raise SystemExit(1)
finally:
    sock.close()
PY
}

container_owns_port() {
    local name="$1" port="$2"
    docker inspect "$name" >/dev/null 2>&1 || return 1
    docker port "$name" 2>/dev/null | grep -Eq "(^|:)$port($|[^0-9])"
}

da360_health_ok() {
    curl --noproxy '*' --fail --silent --max-time 2 \
        "http://127.0.0.1:$API_PORT/health" \
        | python3 -c '
import json, sys
try:
    p = json.load(sys.stdin)
except (json.JSONDecodeError, OSError, TypeError):
    raise SystemExit(1)
expected_model, expected_mode, expected_resample, expected_channels, expected_scale = sys.argv[1:]
valid = (
    p.get("ok") is True
    and p.get("api_version") == 2
    and p.get("model") == expected_model
    and p.get("depth_mode") == expected_mode
    and (
        expected_mode != "da360-metric"
        or (
            p.get("calibration", {}).get("loaded") is True
            and bool(p.get("calibration", {}).get("id"))
        )
    )
    and p.get("resample") == expected_resample
    and p.get("channels_last") is (
        expected_channels.strip().lower() not in {"0", "false", "no", "off", ""}
    )
    # Input dimensions are rounded to 14-pixel patches, so the reported
    # effective scale is intentionally approximate.
    and abs(float(p.get("input_scale")) - float(expected_scale)) < 0.02
)
raise SystemExit(0 if valid else 1)
' "$(basename "$DA360_MODEL" .pth)" "$DEPTH_MODE" \
          "$RESAMPLE" "${DA360_CHANNELS_LAST:-0}" \
          "$INPUT_SCALE"
}

yopo_health_ok() {
    curl --noproxy '*' --fail --silent --max-time 2 \
        "http://127.0.0.1:$API_PORT/yopo/health" \
        | python3 -c '
import json, sys
try:
    p = json.load(sys.stdin)
except (json.JSONDecodeError, OSError, TypeError):
    raise SystemExit(1)
valid = (
    p.get("ok") is True
    and p.get("api_version") == 2
    and p.get("model") == sys.argv[1]
    and p.get("config") == sys.argv[2]
    and p.get("base_config") == sys.argv[3]
    and p.get("strategy") == sys.argv[5]
    and bool(p.get("service_session_id"))
    and p.get("planning_authorized") is (sys.argv[4] == "da360-metric")
)
raise SystemExit(0 if valid else 1)
' "$(basename "$YOPO_MODEL")" "$YOPO_CONFIG_NAME" "$(basename "$YOPO_BASE_CONFIG_PATH")" "$DEPTH_MODE" "$YOPO_STRATEGY"
}

web_health_ok() {
    curl --noproxy '*' --fail --silent --show-error --max-time 2 \
        --output /dev/null "http://$WEB_HOST:$WEB_PORT/"
}

cleanup_failed_start() {
    local status=$?
    if (( status != 0 )) && (( START_COMPLETE == 0 )); then
        echo "启动未完成，清理本次启动的服务..." >&2
        if (( WEB_STARTED == 1 )) && [[ -n "$WEB_PID" ]] && is_owned_web_pid "$WEB_PID"; then
            stop_owned_web_pid "$WEB_PID" || true
        fi
        if (( WEB_STARTED == 1 )) && [[ -f "$PID_FILE" ]]; then
            rm -f "$PID_FILE"
        fi
        if (( API_STARTED == 1 )); then
            docker rm -f "$API_CONTAINER" >/dev/null 2>&1 || true
        fi
    fi
    trap - EXIT
    exit "$status"
}
trap cleanup_failed_start EXIT

echo "========================================="
echo " MindCloud World Fly — 安全启动"
echo "========================================="

echo ""
echo "=== 1/4 预检 ==="
for command_name in docker curl python3 nvidia-smi readlink nohup setsid; do
    require_command "$command_name"
done
resolve_yopo_strategy
[[ "$WEB_PORT" =~ ^[0-9]+$ ]] || die "invalid web port: $WEB_PORT"
[[ "$API_PORT" =~ ^[0-9]+$ ]] || die "invalid API port: $API_PORT"
[[ "$STARTUP_TIMEOUT" =~ ^[0-9]+$ ]] || die "invalid startup timeout: $STARTUP_TIMEOUT"
[[ "$YOPO_CONFIG_NAME" =~ ^[A-Za-z0-9._-]+\.yaml$ ]] \
    || die "YOPO_CONFIG must be a YAML filename inside third_party/YOPO/config"
case "$DEPTH_MODE" in
    relative|da360-relative) DEPTH_MODE="da360-relative" ;;
    metric|da360-metric) DEPTH_MODE="da360-metric" ;;
    *) die "DA360_DEPTH_MODE must be da360-relative or da360-metric" ;;
esac
RESAMPLE="${RESAMPLE,,}"
[[ "$RESAMPLE" == "bicubic" || "$RESAMPLE" == "bilinear" ]] \
    || die "DA360_RESAMPLE must be bicubic or bilinear"
python3 -c 'import sys; value=float(sys.argv[1]); assert 0.2 <= value <= 1.0' "$INPUT_SCALE" \
    2>/dev/null || die "DA360_INPUT_SCALE must be a number in [0.2, 1.0]"
[[ -s "$DA360_MODEL" ]] || die "DA360 checkpoint missing or empty: $DA360_MODEL"
[[ -s "$YOPO_MODEL" ]] || die "YOPO checkpoint missing or empty: $YOPO_MODEL"
[[ -s "$YOPO_CONFIG_PATH" ]] || die "YOPO config missing or empty: $YOPO_CONFIG_PATH"
[[ -s "$YOPO_BASE_CONFIG_PATH" ]] || die "YOPO base config missing or empty: $YOPO_BASE_CONFIG_PATH"
[[ -s "$DEPENDENCY_MANIFEST" ]] || die "dependency version manifest missing or empty: $DEPENDENCY_MANIFEST"
CALIBRATION_FILE=""
if [[ -n "${DA360_DEPTH_CALIB_PATH_HOST:-}" ]]; then
    [[ -s "$DA360_DEPTH_CALIB_PATH_HOST" ]] \
        || die "calibration file missing or empty: $DA360_DEPTH_CALIB_PATH_HOST"
    CALIBRATION_FILE="$(readlink -f "$DA360_DEPTH_CALIB_PATH_HOST")"
elif [[ "$DEPTH_MODE" == "da360-metric" ]]; then
    [[ -s "$DEFAULT_CALIBRATION_FILE" ]] \
        || die "default metric calibration missing or empty: $DEFAULT_CALIBRATION_FILE"
    CALIBRATION_FILE="$(readlink -f "$DEFAULT_CALIBRATION_FILE")"
fi
if [[ "$DEPTH_MODE" == "da360-metric" && -z "$CALIBRATION_FILE" ]]; then
    die "da360-metric mode requires a calibration file"
fi
docker info >/dev/null 2>&1 || die "Docker daemon is unavailable"
docker image inspect "$API_IMAGE" >/dev/null 2>&1 \
    || die "Docker image missing: $API_IMAGE (build Dockerfile.da360-yopo first)"
nvidia-smi -L >/dev/null 2>&1 || die "NVIDIA GPU/driver is unavailable"

DA360_MODEL="$(readlink -f "$DA360_MODEL")"
YOPO_MODEL="$(readlink -f "$YOPO_MODEL")"
python3 "$SCRIPT_DIR/scripts/verify_dependencies.py" \
    --da360-model "$DA360_MODEL" --yopo-model "$YOPO_MODEL" \
    --yopo-strategy "$YOPO_STRATEGY" \
    || die "runtime dependency version verification failed"
echo "  DA360: $(basename "$DA360_MODEL")"
echo "  strategy: $YOPO_STRATEGY"
echo "  YOPO:  $(basename "$YOPO_MODEL")"
echo "  config: $(basename "$YOPO_BASE_CONFIG_PATH") + $YOPO_CONFIG_NAME"
echo "  image:  $API_IMAGE"

OWNED_WEB_PIDS=()
if [[ -f "$PID_FILE" ]]; then
    PID_FILE_VALUE="$(tr -d '[:space:]' < "$PID_FILE")"
    if [[ -n "$PID_FILE_VALUE" ]] && is_owned_web_pid "$PID_FILE_VALUE"; then
        OWNED_WEB_PIDS+=("$PID_FILE_VALUE")
    else
        if [[ "$PID_FILE_VALUE" =~ ^[0-9]+$ ]] && kill -0 "$PID_FILE_VALUE" 2>/dev/null; then
            die "PID file points to a non-project process ($PID_FILE_VALUE); refusing to kill it"
        fi
        echo "  Removing stale Web PID file"
        rm -f "$PID_FILE"
    fi
fi

while IFS= read -r discovered_pid; do
    already_listed=0
    for known_pid in "${OWNED_WEB_PIDS[@]:-}"; do
        [[ "$known_pid" == "$discovered_pid" ]] && already_listed=1
    done
    (( already_listed == 1 )) || OWNED_WEB_PIDS+=("$discovered_pid")
done < <(find_owned_web_pids)

if ! port_is_free "$WEB_HOST" "$WEB_PORT"; then
    if (( ${#OWNED_WEB_PIDS[@]} > 0 )); then
        : # safe to replace below
    elif container_owns_port "$LEGACY_WEB_CONTAINER" "$WEB_PORT"; then
        : # known project container; safe to replace below
    else
        die "$WEB_HOST:$WEB_PORT is occupied by a process not owned by this project"
    fi
fi
if ! port_is_free 127.0.0.1 "$API_PORT" \
    && ! container_owns_port "$API_CONTAINER" "$API_PORT"; then
    die "127.0.0.1:$API_PORT is occupied by a process not owned by $API_CONTAINER"
fi

echo ""
echo "=== 2/4 DA360 + YOPO 推理服务 ==="
docker rm -f "$API_CONTAINER" >/dev/null 2>&1 || true
YOPO_CONTAINER_MODEL="/models/$(basename "$YOPO_MODEL")"

run_args=(
    --rm -d --init
    --name "$API_CONTAINER"
    --gpus all
    -p "127.0.0.1:$API_PORT:5688"
    -v "$DA360_MODEL:/models/DA360_large.pth:ro"
    -v "$YOPO_MODEL:$YOPO_CONTAINER_MODEL:ro"
    -v "$SCRIPT_DIR/third_party/DA360:/opt/DA360:ro"
    -v "$SCRIPT_DIR/third_party/YOPO:/opt/YOPO_360/YOPO:ro"
    -v "$SCRIPT_DIR/scripts:/opt/server:ro"
    -e "YOPO_CONFIG=$YOPO_CONFIG_NAME"
    -e "YOPO_MODEL_PATH=$YOPO_CONTAINER_MODEL"
    -e "MINDCLOUD_YOPO_STRATEGY=$YOPO_STRATEGY"
    -e "DA360_INPUT_SCALE=$INPUT_SCALE"
    -e "DA360_DEPTH_SCALE=${DA360_DEPTH_SCALE:-2.0}"
    -e "DA360_DEPTH_MODE=$DEPTH_MODE"
    -e "DA360_RESAMPLE=$RESAMPLE"
    -e "DA360_CHANNELS_LAST=${DA360_CHANNELS_LAST:-0}"
    -e "YOPO_CHANNELS_LAST=${YOPO_CHANNELS_LAST:-0}"
    -e "DA360_MAX_CONTENT_LENGTH=${DA360_MAX_CONTENT_LENGTH:-8388608}"
    -e "DA360_ALLOWED_ORIGINS=http://127.0.0.1:$WEB_PORT,http://localhost:$WEB_PORT"
)

if [[ -n "$CALIBRATION_FILE" ]]; then
    run_args+=(
        -v "$CALIBRATION_FILE:/opt/calibration/depth_calibration.json:ro"
        -e "DA360_DEPTH_CALIB_PATH=/opt/calibration/depth_calibration.json"
    )
fi

docker run "${run_args[@]}" "$API_IMAGE" \
    python3 /opt/server/combined_server.py --host 0.0.0.0 --port 5688 >/dev/null
API_STARTED=1
echo "  容器已启动，仅发布到 127.0.0.1:$API_PORT"

echo ""
echo "=== 3/4 Web 服务 ==="
for old_web_pid in "${OWNED_WEB_PIDS[@]:-}"; do
    [[ -n "$old_web_pid" ]] || continue
    stop_owned_web_pid "$old_web_pid" || die "refused to stop unowned Web PID $old_web_pid"
done
rm -f "$PID_FILE"
docker rm -f "$LEGACY_WEB_CONTAINER" >/dev/null 2>&1 || true
port_is_free "$WEB_HOST" "$WEB_PORT" \
    || die "$WEB_HOST:$WEB_PORT is still occupied after stopping the previous project service"
# Detach from the caller's terminal so closing the launch shell does not take
# the local Web service down. Runtime output is kept outside Git.
setsid nohup python3 "$SCRIPT_DIR/scripts/serve.py" "$WEB_PORT" "$WEB_HOST" \
    </dev/null >>"$WEB_LOG" 2>&1 &
WEB_PID=$!
WEB_STARTED=1
echo "$WEB_PID" > "$PID_FILE"
sleep 0.3
if ! is_owned_web_pid "$WEB_PID"; then
    tail -n 40 "$WEB_LOG" >&2 || true
    die "Web service failed to start"
fi
echo "  Web 已启动 http://$WEB_HOST:$WEB_PORT (PID $WEB_PID)"

echo ""
echo "=== 4/4 验证健康状态 ==="
deadline=$((SECONDS + STARTUP_TIMEOUT))
ready=0
while (( SECONDS < deadline )); do
    if web_health_ok && da360_health_ok && yopo_health_ok; then
        ready=1
        break
    fi
    sleep 2
done
if (( ready == 0 )); then
    echo "  服务在 ${STARTUP_TIMEOUT}s 内未同时通过 Web/DA360/YOPO 健康检查" >&2
    tail -n 80 "$WEB_LOG" >&2 || true
    docker logs --tail 80 "$API_CONTAINER" >&2 || true
    exit 1
fi
echo "  Web、DA360 与 YOPO 均已就绪"

if [[ "${MINDCLOUD_FIX_CLASH:-0}" == "1" ]]; then
    echo "  MINDCLOUD_FIX_CLASH=1：执行显式 Clash 修复"
    "$SCRIPT_DIR/fix-clash-rules.sh" || echo "WARNING: Clash 修复失败" >&2
fi

START_COMPLETE=1
trap - EXIT
echo ""
echo "========================================="
echo " 全部就绪"
echo "  Web:    http://$WEB_HOST:$WEB_PORT"
echo "  DA360:  http://127.0.0.1:$API_PORT/health"
echo "  YOPO:   http://127.0.0.1:$API_PORT/yopo/health"
echo "  策略:   $YOPO_STRATEGY"
echo "========================================="
echo "  启动 Firefox: ./launch-firefox-gpu.sh"
echo "  停止全部:     ./stop-all.sh"
