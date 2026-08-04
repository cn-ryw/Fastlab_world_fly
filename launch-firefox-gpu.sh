#!/usr/bin/env bash
# Firefox + NVIDIA GPU 渲染 (用于 Google 3D Tiles Flight)
# 前提: Clash TUN 模式已开启，规则已配好（tile.googleapis.com → GLOBAL）
# Usage: ./launch-firefox-gpu.sh [url]

set -euo pipefail
URL="${1:-http://127.0.0.1:8080/}"

# TUN 模式下不需要系统代理，清理干净
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY

# NVIDIA prime offload（RTX 5070 Ti）
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export MOZ_X11_EGL=0

echo "启动 Firefox (NVIDIA) → ${URL}"
firefox "${URL}" &
