#!/usr/bin/env bash
# MindCloud World Fly — 全部服务停止脚本
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="$SCRIPT_DIR/.dev-web.pid"

# ── dev-web 本地 Python 服务 ──
if [ -f "$PID_FILE" ]; then
    PID=$(cat "$PID_FILE")
    if kill -0 "$PID" 2>/dev/null; then
        kill "$PID" && echo "Web (dev-web, PID $PID) 已停止"
    else
        echo "Web (dev-web) 未运行"
    fi
    rm -f "$PID_FILE"
else
    echo "Web (dev-web) PID 文件不存在"
fi

# ── Docker Web 容器（如果存在）──
docker rm -f google-tiles-flight 2>/dev/null && echo "Web (Docker) 已停止" || true

# ── DA360+YOPO 容器 ──
docker rm -f mindcloud-da360-yopo 2>/dev/null && echo "DA360+YOPO 已停止" || echo "DA360+YOPO 未运行"

echo "全部停止"
