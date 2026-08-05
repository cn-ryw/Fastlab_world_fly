#!/usr/bin/env bash
# MindCloud World Fly — 全部服务停止脚本
set -euo pipefail

echo "停止 Web..."
docker rm -f google-tiles-flight 2>/dev/null && echo "  Web 已停止" || echo "  Web 未运行"

echo "停止 DA360+YOPO..."
docker rm -f mindcloud-da360-yopo 2>/dev/null && echo "  DA360+YOPO 已停止" || echo "  DA360+YOPO 未运行"

echo "全部停止"
