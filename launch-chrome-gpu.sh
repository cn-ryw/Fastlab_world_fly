#!/usr/bin/env bash
# Launch MindCloud in a persistent, non-incognito Chrome flight profile.
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly DEFAULT_URL='http://127.0.0.1:8080/?panoProfile=flight&panoPreloadRequired=0'
readonly DEFAULT_LOG_FILE="${SCRIPT_DIR}/.dev-chrome.log"

info() { printf '[chrome-gpu] %s\n' "$*"; }
warn() { printf '[chrome-gpu] WARNING: %s\n' "$*" >&2; }
die()  { printf '[chrome-gpu] ERROR: %s\n' "$*" >&2; exit 1; }

if [[ "${1:-}" == '-h' || "${1:-}" == '--help' ]]; then
    cat <<EOF
Usage: ./launch-chrome-gpu.sh [URL]

Environment:
  CHROME_BIN          Chrome executable override
  CHROME_LOG_FILE     Browser log override
  CHROME_PROFILE_DIR  Persistent flight profile directory override
EOF
    exit 0
fi
(( $# <= 1 )) || die 'expected at most one URL argument'

readonly TARGET_URL="${1:-${DEFAULT_URL}}"
case "${TARGET_URL}" in http://*|https://*) ;; *) die 'URL must use http:// or https://' ;; esac

chrome_executable=''
if [[ -n "${CHROME_BIN:-}" ]]; then
    chrome_executable="$(command -v -- "${CHROME_BIN}" 2>/dev/null || true)"
    [[ -n "${chrome_executable}" ]] || [[ -x "${CHROME_BIN}" ]] || die "Chrome is not executable: ${CHROME_BIN}"
    [[ -n "${chrome_executable}" ]] || chrome_executable="${CHROME_BIN}"
else
    for candidate in google-chrome-stable google-chrome /opt/google/chrome/chrome; do
        chrome_executable="$(command -v -- "${candidate}" 2>/dev/null || true)"
        [[ -n "${chrome_executable}" ]] && break
    done
    [[ -n "${chrome_executable}" ]] || die 'Google Chrome was not found'
fi

readonly LOG_FILE="${CHROME_LOG_FILE:-${DEFAULT_LOG_FILE}}"
readonly PROFILE_DIR="${CHROME_PROFILE_DIR:-${SCRIPT_DIR}/.chrome-mindcloud-flight}"
[[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || die 'no graphical display detected'

# Chrome keeps a profile process alive after its last window closes. Reusing a
# process also reuses its original command-line flags, which previously left
# this flight profile running with an unlimited RAF rate. Stop only processes
# carrying this launcher's exact user-data-dir before creating the new window.
profile_pids=()
for proc in /proc/[0-9]*; do
    [[ -O "${proc}" && -r "${proc}/cmdline" ]] || continue
    pid="${proc##*/}"
    while IFS= read -r -d '' argument; do
        if [[ "${argument}" == "--user-data-dir=${PROFILE_DIR}" ]]; then
            profile_pids+=("${pid}")
            break
        fi
    done < "${proc}/cmdline"
done
if (( ${#profile_pids[@]} > 0 )); then
    info "Stopping previous MindCloud Chrome profile (${#profile_pids[@]} process(es))..."
    kill -TERM "${profile_pids[@]}" 2>/dev/null || true
    for _ in $(seq 1 50); do
        survivors=()
        for pid in "${profile_pids[@]}"; do
            kill -0 "${pid}" 2>/dev/null && survivors+=("${pid}")
        done
        (( ${#survivors[@]} == 0 )) && break
        sleep 0.1
    done
    if (( ${#survivors[@]} > 0 )); then
        warn 'old dedicated Chrome processes did not exit cleanly; forcing this profile closed'
        kill -KILL "${survivors[@]}" 2>/dev/null || true
    fi
fi

merge_no_proxy() {
    local value="${1:-}" entry result=''
    for entry in ${value//,/ } 127.0.0.1 localhost ::1; do
        [[ ",${result}," == *",${entry},"* ]] && continue
        result="${result:+${result},}${entry}"
    done
    printf '%s' "${result}"
}
no_proxy="$(merge_no_proxy "${no_proxy:-}")"
NO_PROXY="$(merge_no_proxy "${NO_PROXY:-}")"
export no_proxy NO_PROXY

# Chrome/ANGLE on this X11 workstation fails EGL initialization when NVIDIA
# PRIME or Vulkan is forced ("Invalid visual ID requested"). Let Chrome use
# the desktop session's working GPU selection instead.
unset __NV_PRIME_RENDER_OFFLOAD
unset __GLX_VENDOR_LIBRARY_NAME
unset __VK_LAYER_NV_optimus
unset DRI_PRIME
unset LIBGL_ALWAYS_SOFTWARE

if command -v curl >/dev/null 2>&1 \
    && ! curl --noproxy '*' --fail --silent --max-time 2 "${TARGET_URL}" >/dev/null 2>&1; then
    warn 'local web server is not responding yet; run ./start-all.sh first'
fi

mkdir -p -- "${PROFILE_DIR}" "$(dirname -- "${LOG_FILE}")"
chrome_version="$("${chrome_executable}" --version 2>/dev/null || true)"
info "Chrome: ${chrome_version:-unknown}"
info "Profile: ${PROFILE_DIR} (persistent, non-incognito)"
info "Log: ${LOG_FILE}"

nohup "${chrome_executable}" \
    --user-data-dir="${PROFILE_DIR}" \
    --new-window \
    --ozone-platform=x11 \
    --use-angle=gl \
    --disable-background-mode \
    --disable-extensions \
    "${TARGET_URL}" </dev/null >>"${LOG_FILE}" 2>&1 &
chrome_pid=$!
disown "${chrome_pid}" 2>/dev/null || true
info "Chrome started in the background (PID ${chrome_pid})."
