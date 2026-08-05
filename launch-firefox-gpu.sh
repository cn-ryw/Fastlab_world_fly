#!/usr/bin/env bash
# Firefox + NVIDIA RTX 5070 Ti GPU 渲染
# Usage: ./launch-firefox-gpu.sh [url]
set -euo pipefail

URL="${1:-http://127.0.0.1:8080/?panoPreloadRequired=0}"

# 清理代理环境变量 (TUN模式不需要)
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY

# NVIDIA GPU 渲染
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export MOZ_X11_EGL=0

echo "Launching Firefox (NVIDIA GPU) → ${URL}"
firefox "${URL}" &
