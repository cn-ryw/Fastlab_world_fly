#!/usr/bin/env bash
# 修复 Clash Cesium/Google Tiles 规则 — DIRECT → 代理
# 直接修改订阅 profile, 重启 Clash 生效
set -euo pipefail

PROFILE_DIR="/home/ykx/.local/share/io.github.clash-verge-rev.clash-verge-rev/profiles"

for f in "$PROFILE_DIR"/*.yaml; do
    if grep -q "DOMAIN-SUFFIX,cesium.com,DIRECT\|DOMAIN-SUFFIX,tile.googleapis.com,DIRECT\|DOMAIN-SUFFIX,googleapis.com,DIRECT" "$f" 2>/dev/null; then
        sed -i "s/'DOMAIN-SUFFIX,cesium.com,DIRECT'/'DOMAIN-SUFFIX,cesium.com,良心云'/" "$f"
        sed -i "s/'DOMAIN-SUFFIX,tile.googleapis.com,DIRECT'/'DOMAIN-SUFFIX,tile.googleapis.com,良心云'/" "$f"
        sed -i "s/'DOMAIN-SUFFIX,googleapis.com,DIRECT'/'DOMAIN-SUFFIX,googleapis.com,良心云'/" "$f"
        echo "已修复: $f"
    fi
done

echo "Clash 规则已修复, 重启 Clash Verge 生效"
