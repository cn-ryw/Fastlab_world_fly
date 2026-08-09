#!/usr/bin/env bash
# MindCloud World Fly — 全部服务启动脚本
# Usage: ./start-all.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
PID_FILE="$SCRIPT_DIR/.dev-web.pid"

echo "========================================="
echo " MindCloud World Fly — 启动"
echo "========================================="

# ── 1. DA360 + YOPO 推理服务 (GPU, 端口 5688) ──
echo ""
echo "=== 1/3 DA360 + YOPO 推理服务 (GPU) ==="
docker rm -f mindcloud-da360-yopo 2>/dev/null || true
docker run --rm -d --init \
  --name mindcloud-da360-yopo \
  --gpus all -p 5688:5688 \
  -v "$SCRIPT_DIR/third_party/DA360/checkpoints/DA360_large.pth:/models/DA360_large.pth:ro" \
  -v /home/ykx/ros1/YOPO_360_v15/YOPO/saved/YOPO_55/epoch10.pth:/models/epoch10.pth:ro \
  -v "$SCRIPT_DIR/scripts:/opt/server:ro" \
  -e DA360_INPUT_SCALE=0.46 \
  -e DA360_DEPTH_SCALE=2.0 \
  mindcloud-da360-yopo:latest
echo "  DA360+YOPO 启动中 (端口 5688)..."

# ── 2. Web 服务（本地 Python，从磁盘直读最新 JS，端口 8080）──
echo ""
echo "=== 2/3 Web 服务（本地开发模式，实时加载最新 JS）==="
# 释放 8080 端口：停旧 Docker 容器 + 杀残留 Python 进程
docker rm -f google-tiles-flight 2>/dev/null || true
if [ -f "$PID_FILE" ]; then
    kill "$(cat "$PID_FILE")" 2>/dev/null || true
    rm -f "$PID_FILE"
fi
# 兜底：fuser 杀掉任何占着 8080 的进程
fuser -k 8080/tcp 2>/dev/null || true
sleep 0.3
python3 "$SCRIPT_DIR/scripts/serve.py" 8080 &
WEB_PID=$!
echo "$WEB_PID" > "$PID_FILE"
sleep 0.3
kill -0 "$WEB_PID" 2>/dev/null || { echo "  Web 服务启动失败"; exit 1; }
echo "  Web 已启动 http://127.0.0.1:8080 (PID $WEB_PID)"

# ── 3. 等待模型加载 ──
echo ""
echo "=== 3/3 等待模型加载 ==="
for i in $(seq 1 30); do
  if curl -s http://127.0.0.1:5688/health 2>/dev/null | grep -q '"ok":true'; then
    echo "  DA360+YOPO 就绪 (${i}s)"
    break
  fi
  sleep 2
done

# ── Clash 规则 ──
./fix-clash-rules.sh 2>/dev/null || true

echo ""
echo "========================================="
echo " 全部就绪"
echo "  Web:    http://127.0.0.1:8080"
echo "  DA360:  http://127.0.0.1:5688/health"
echo "  YOPO:   http://127.0.0.1:5688/yopo/health"
echo "========================================="
echo ""
echo "  启动 Firefox: ./launch-firefox-gpu.sh"
echo "  停止全部:     ./stop-all.sh"
echo ""
