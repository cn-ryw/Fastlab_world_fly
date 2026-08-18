#!/usr/bin/env bash
# FASTLab World Fly — stop only processes owned by this project
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="$SCRIPT_DIR/.dev-web.pid"
API_CONTAINER="${MINDCLOUD_API_CONTAINER:-mindcloud-da360-yopo}"
LEGACY_WEB_CONTAINER="google-tiles-flight"
status=0

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
        is_owned_web_pid "$pid" && echo "$pid"
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
        echo "Web PID $pid did not stop after 5s; sending KILL" >&2
        kill -KILL "$pid"
    fi
}

OWNED_WEB_PIDS=()
if [[ -f "$PID_FILE" ]]; then
    WEB_PID="$(tr -d '[:space:]' < "$PID_FILE")"
    if is_owned_web_pid "$WEB_PID"; then
        OWNED_WEB_PIDS+=("$WEB_PID")
    elif [[ "$WEB_PID" =~ ^[0-9]+$ ]] && kill -0 "$WEB_PID" 2>/dev/null; then
        echo "ERROR: PID file points to non-project process $WEB_PID; refusing to kill or remove it" >&2
        status=1
    else
        rm -f "$PID_FILE"
        echo "Web PID 文件已过期并清理"
    fi
else
    echo "Web PID 文件不存在"
fi

while IFS= read -r discovered_pid; do
    already_listed=0
    for known_pid in "${OWNED_WEB_PIDS[@]:-}"; do
        [[ "$known_pid" == "$discovered_pid" ]] && already_listed=1
    done
    (( already_listed == 1 )) || OWNED_WEB_PIDS+=("$discovered_pid")
done < <(find_owned_web_pids)

for web_pid in "${OWNED_WEB_PIDS[@]:-}"; do
    [[ -n "$web_pid" ]] || continue
    if stop_owned_web_pid "$web_pid"; then
        echo "Web (PID $web_pid) 已停止"
    else
        echo "ERROR: refusing to stop unowned Web PID $web_pid" >&2
        status=1
    fi
done
if (( status == 0 )) || [[ ! -f "$PID_FILE" ]]; then
    rm -f "$PID_FILE"
fi

if command -v docker >/dev/null 2>&1; then
    docker rm -f "$LEGACY_WEB_CONTAINER" >/dev/null 2>&1 \
        && echo "Web (Docker) 已停止" || true
    docker rm -f "$API_CONTAINER" >/dev/null 2>&1 \
        && echo "DA360+YOPO 已停止" || echo "DA360+YOPO 未运行"
else
    echo "WARNING: Docker 不可用，无法检查推理容器" >&2
    status=1
fi

(( status == 0 )) && echo "全部停止"
exit "$status"
