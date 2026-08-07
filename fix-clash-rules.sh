#!/usr/bin/env bash
# 检查并修复 Clash Cesium/Google Tiles 规则 — DIRECT → 代理
# 仅在规则错误时才修改并重启
set -euo pipefail

PROFILE_DIR="/home/ykx/.local/share/io.github.clash-verge-rev.clash-verge-rev/profiles"

FOUND_BAD=0
for f in "$PROFILE_DIR"/*.yaml; do
    if grep -q "DOMAIN-SUFFIX,cesium.com,DIRECT\|DOMAIN-SUFFIX,tile.googleapis.com,DIRECT\|DOMAIN-SUFFIX,googleapis.com,DIRECT" "$f" 2>/dev/null; then
        FOUND_BAD=1
        break
    fi
done

if [ "$FOUND_BAD" -eq 0 ]; then
    echo "Clash 规则正确, 无需修复"
    exit 0
fi

echo "检测到错误的 DIRECT 规则, 正在修复..."
for f in "$PROFILE_DIR"/*.yaml; do
    if grep -q "DOMAIN-SUFFIX,cesium.com,DIRECT\|DOMAIN-SUFFIX,tile.googleapis.com,DIRECT\|DOMAIN-SUFFIX,googleapis.com,DIRECT" "$f" 2>/dev/null; then
        sed -i "s/'DOMAIN-SUFFIX,cesium.com,DIRECT'/'DOMAIN-SUFFIX,cesium.com,良心云'/" "$f"
        sed -i "s/'DOMAIN-SUFFIX,tile.googleapis.com,DIRECT'/'DOMAIN-SUFFIX,tile.googleapis.com,良心云'/" "$f"
        sed -i "s/'DOMAIN-SUFFIX,googleapis.com,DIRECT'/'DOMAIN-SUFFIX,googleapis.com,良心云'/" "$f"
        echo "已修复: $f"
    fi
done

echo "正在重启 Clash..."
sudo systemctl restart clash-verge-service 2>/dev/null && echo "Clash 已重启, 规则生效" || echo "请手动重启 Clash Verge"
