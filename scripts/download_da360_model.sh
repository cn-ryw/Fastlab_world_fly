#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DA360_ROOT="$PROJECT_ROOT/third_party/DA360"
CHECKPOINT_DIR="$DA360_ROOT/checkpoints"
MODEL_NAME="${1:-${DA360_MODEL:-large}}"

case "$MODEL_NAME" in
    small)
        MODEL_ID="1NYF4yJR83HEtxzOURLdmONeUe413auP6"
        MODEL_MIN_BYTES=350000000
        ;;
    base)
        MODEL_ID="17CEsiWRvGPKVrEOonGLXYcxobLK8idId"
        MODEL_MIN_BYTES=1
        ;;
    large)
        MODEL_ID="1cWEUZP-uBuk6WlUi0KJF3zdd05ckHKuR"
        MODEL_MIN_BYTES=1200000000
        ;;
    *)
        echo "ERROR: unknown DA360 model '$MODEL_NAME'. Use small, base, or large." >&2
        exit 1
        ;;
esac

MODEL_PATH="$CHECKPOINT_DIR/DA360_${MODEL_NAME}.pth"

die() {
    echo "ERROR: $*" >&2
    exit 1
}

command -v git >/dev/null 2>&1 || die "git is required to clone DA360."

clone_da360() {
    git clone --depth 1 https://github.com/Insta360-Research-Team/DA360.git "$DA360_ROOT"
}

gdown_cmd=()
if command -v gdown >/dev/null 2>&1; then
    gdown_cmd=(gdown)
elif python3 -m gdown --version >/dev/null 2>&1; then
    gdown_cmd=(python3 -m gdown)
else
    die "gdown is required. Install it with: python3 -m pip install --user gdown"
fi

mkdir -p "$(dirname "$DA360_ROOT")"

if [[ ! -d "$DA360_ROOT" ]]; then
    clone_da360
elif [[ ! -f "$DA360_ROOT/networks/__init__.py" ]]; then
    if [[ -z "$(find "$DA360_ROOT" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
        rmdir "$DA360_ROOT"
        clone_da360
    else
        die "$DA360_ROOT exists but does not look like a DA360 checkout. Remove it and rerun this script."
    fi
fi

mkdir -p "$CHECKPOINT_DIR"

if [[ -s "$MODEL_PATH" ]]; then
    current_size="$(stat -c '%s' "$MODEL_PATH" 2>/dev/null || echo 0)"
    if ((current_size < MODEL_MIN_BYTES)); then
        echo "DA360 model looks incomplete: $MODEL_PATH ($current_size bytes). Resuming download..."
    else
        echo "DA360 model already exists: $MODEL_PATH"
        exit 0
    fi
fi

echo "Downloading DA360_${MODEL_NAME}: $MODEL_PATH"
"${gdown_cmd[@]}" --continue "https://drive.google.com/uc?id=$MODEL_ID" -O "$MODEL_PATH"
