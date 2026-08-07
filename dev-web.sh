#!/usr/bin/env bash
# 前端本地开发模式：Python 从磁盘直读，改完刷新即生效。
# 前提：DA360+YOPO 容器已在运行（另一个终端 ./start-all.sh 或手动 docker run）。
# 停止：Ctrl+C 即可。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${1:-8080}"

echo "========================================="
echo " Web 本地开发模式"
echo " 地址: http://127.0.0.1:${PORT}"
echo " 改 src/*.js 或 index.html 后刷新浏览器即可"
echo " 无需 docker build / docker restart"
echo "========================================="
echo ""

exec python3 "$SCRIPT_DIR/scripts/serve.py" "$PORT"
