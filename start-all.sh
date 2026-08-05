#!/usr/bin/env bash
# MindCloud World Fly — 全部服务启动脚本
# Usage: ./start-all.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== 1/3 DA360 + YOPO 推理服务 (GPU) ==="
docker rm -f mindcloud-da360-yopo 2>/dev/null || true
docker run --rm -d --init \
  --name mindcloud-da360-yopo \
  --gpus all -p 5688:5688 \
  -v "$SCRIPT_DIR/third_party/DA360/checkpoints/DA360_large.pth:/models/DA360_large.pth:ro" \
  -v /home/ykx/ros1/YOPO_360_v15/YOPO/saved/YOPO_55/epoch10.pth:/models/epoch10.pth:ro \
  -e DA360_INPUT_SCALE=0.65 \
  -e DA360_DEPTH_SCALE=3.0 \
  mindcloud-da360-yopo:latest
echo "  DA360+YOPO 启动中 (端口 5688)..."

echo ""
echo "=== 2/3 Web 服务 (Cesium) ==="
docker rm -f google-tiles-flight 2>/dev/null || true
docker build -q -f Dockerfile.cesium -t google-tiles-flight:latest . >/dev/null 2>&1
docker run --rm -d --init \
  --name google-tiles-flight -p 8080:8000 \
  google-tiles-flight:latest
echo "  Web 已启动 http://127.0.0.1:8080"

echo ""
echo "=== 3/3 等待模型加载 ==="
for i in $(seq 1 30); do
  if curl -s http://127.0.0.1:5688/health 2>/dev/null | grep -q '"ok":true'; then
    echo "  DA360+YOPO 就绪 (${i}s)"
    break
  fi
  sleep 2
done

echo ""
echo "=== 全部就绪 ==="
echo "  Web:    http://127.0.0.1:8080"
echo "  DA360:  http://127.0.0.1:5688/health"
echo "  YOPO:   http://127.0.0.1:5688/yopo/health"
# 修复 Clash 规则
./fix-clash-rules.sh

echo ""
echo "  启动 Firefox: ./launch-firefox-gpu.sh"
