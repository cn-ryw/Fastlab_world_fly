#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCK="$PROJECT_ROOT/dependencies.lock.json"
MODE="${1:-}"

die() { echo "ERROR: $*" >&2; exit 1; }
require() { command -v "$1" >/dev/null 2>&1 || die "missing command: $1"; }
lock_value() {
  python3 - "$LOCK" "$1" <<'PY'
import json, sys
value = json.load(open(sys.argv[1], encoding="utf-8"))
for part in sys.argv[2].split('.'):
    value = value[part]
print(value)
PY
}
verify_download() {
  local path="$1" expected_size="$2" expected_hash="$3"
  [[ "$(stat -c '%s' "$path")" == "$expected_size" ]] || die "unexpected size: $path"
  [[ "$(sha256sum "$path" | awk '{print $1}')" == "$expected_hash" ]] || die "checksum mismatch: $path"
}

bootstrap_web() {
  require curl; require unzip; require python3; require sha256sum
  local cesium_dir="$PROJECT_ROOT/third_party/Cesium"
  local playcanvas="$PROJECT_ROOT/asset/vendor/playcanvas.min.js"
  if python3 "$PROJECT_ROOT/scripts/verify_dependencies.py" --skip-source --skip-checkpoints >/dev/null 2>&1; then
    echo "Web dependencies already verified"
    return
  fi
  [[ ! -e "$cesium_dir" ]] || die "$cesium_dir exists but failed verification; move it aside and rerun"
  [[ ! -e "$playcanvas" ]] || die "$playcanvas exists but failed verification; move it aside and rerun"

  local temp_dir
  temp_dir="$(mktemp -d /tmp/mindcloud-web.XXXXXX)"
  trap 'rm -rf "$temp_dir"' RETURN
  curl --fail --location --retry 3 "$(lock_value runtime_dependencies.cesium.download_url)" -o "$temp_dir/cesium.zip"
  unzip -q "$temp_dir/cesium.zip" -d "$temp_dir/cesium"
  local source_dir
  source_dir="$(find "$temp_dir/cesium" -type f -path '*/Build/Cesium/Cesium.js' -printf '%h\n' -quit)"
  [[ -n "$source_dir" ]] || die "Cesium archive does not contain Build/Cesium"
  mkdir -p "$(dirname "$cesium_dir")" "$PROJECT_ROOT/asset/vendor"
  cp -a "$source_dir" "$cesium_dir"
  curl --fail --location --retry 3 "$(lock_value runtime_dependencies.playcanvas.download_url)" -o "$playcanvas"
  python3 "$PROJECT_ROOT/scripts/verify_dependencies.py" --skip-source --skip-checkpoints
  rm -rf -- "$temp_dir"
  trap - RETURN
}

bootstrap_autonomy() {
  bootstrap_web
  require git; require curl; require tar; require sha256sum; require python3
  local da360_root="$PROJECT_ROOT/third_party/DA360"
  local da360_commit
  da360_commit="$(lock_value runtime_dependencies.da360.commit)"
  if [[ ! -d "$da360_root/.git" ]]; then
    [[ ! -e "$da360_root" ]] || die "$da360_root exists but is not a Git checkout"
    git clone "$(lock_value runtime_dependencies.da360.repository)" "$da360_root"
  fi
  git -C "$da360_root" fetch --depth 1 origin "$da360_commit"
  git -C "$da360_root" checkout --detach "$da360_commit"

  local da360_model="$PROJECT_ROOT/$(lock_value model_checkpoints.da360_large.default_host_path)"
  if [[ ! -s "$da360_model" ]]; then
    python3 -m gdown --version >/dev/null 2>&1 || die "install gdown to download the official DA360 checkpoint"
    mkdir -p "$(dirname "$da360_model")"
    python3 -m gdown --continue "https://drive.google.com/uc?id=1cWEUZP-uBuk6WlUi0KJF3zdd05ckHKuR" -O "$da360_model"
  fi
  verify_download "$da360_model" \
    "$(lock_value model_checkpoints.da360_large.size_bytes)" \
    "$(lock_value model_checkpoints.da360_large.sha256)"

  local yopo_model="$PROJECT_ROOT/$(lock_value model_checkpoints.yopo_epoch10.default_host_path)"
  if [[ ! -s "$yopo_model" ]]; then
    local temp_dir archive extracted
    temp_dir="$(mktemp -d /tmp/mindcloud-yopo.XXXXXX)"
    trap 'rm -rf "$temp_dir"' RETURN
    archive="$temp_dir/yopo.tar.gz"
    curl --fail --location --retry 3 "$(lock_value model_checkpoints.yopo_epoch10.download_url)" -o "$archive"
    tar -xzf "$archive" -C "$temp_dir"
    extracted="$(find "$temp_dir" -type f -name epoch10.pth -print -quit)"
    [[ -n "$extracted" ]] || die "YOPO release archive does not contain epoch10.pth"
    mkdir -p "$(dirname "$yopo_model")"
    cp "$extracted" "$yopo_model"
    rm -rf -- "$temp_dir"
    trap - RETURN
  fi
  verify_download "$yopo_model" \
    "$(lock_value model_checkpoints.yopo_epoch10.size_bytes)" \
    "$(lock_value model_checkpoints.yopo_epoch10.sha256)"
  python3 "$PROJECT_ROOT/scripts/verify_dependencies.py"
}

case "$MODE" in
  web) bootstrap_web ;;
  autonomy) bootstrap_autonomy ;;
  *) die "usage: scripts/bootstrap.sh web|autonomy" ;;
esac
