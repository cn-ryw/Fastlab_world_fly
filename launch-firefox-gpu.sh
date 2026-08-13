#!/usr/bin/env bash
# Launch the local MindCloud simulation in Firefox with NVIDIA PRIME offload.
#
# Usage:
#   ./launch-firefox-gpu.sh [URL]
#
# Optional environment overrides:
#   FIREFOX_BIN       Firefox executable (default: first `firefox` in PATH)
#   FIREFOX_LOG_FILE  Browser stdout/stderr log

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly DEFAULT_URL='http://127.0.0.1:8080/?panoProfile=flight&perfProfile=demo30&tileRequestsPerServer=12&panoramaTileSse=256&panoPreloadRequired=0&flightPreloadRadius=300&flightPreloadViewTimeoutMs=30000&flightPreloadViewAttempts=2'
readonly DEFAULT_LOG_FILE="${SCRIPT_DIR}/.dev-firefox.log"

info() { printf '[firefox-gpu] %s\n' "$*"; }
warn() { printf '[firefox-gpu] WARNING: %s\n' "$*" >&2; }
die()  { printf '[firefox-gpu] ERROR: %s\n' "$*" >&2; exit 1; }

# Produce a diagnostic-safe URL without evaluating any of its contents.
# Credentials, query parameters and fragments can contain API keys or tokens,
# so only scheme, host/port and path are retained for terminal/log output.
redact_url_for_display() {
    local url="${1:-}" scheme remainder authority path display

    case "${url}" in
        http://*)  scheme='http';  remainder="${url#http://}" ;;
        https://*) scheme='https'; remainder="${url#https://}" ;;
        *) printf '%s' '<invalid-url>'; return ;;
    esac

    remainder="${remainder%%\#*}"
    remainder="${remainder%%\?*}"
    if [[ "${remainder}" == */* ]]; then
        authority="${remainder%%/*}"
        path="/${remainder#*/}"
    else
        authority="${remainder}"
        path=''
    fi
    authority="${authority##*@}"
    display="${scheme}://${authority}${path}"

    # Prevent control characters in an untrusted URL from forging log lines or
    # terminal escape sequences. Parameter expansion never executes the text.
    while [[ "${display}" == *[[:cntrl:]]* ]]; do
        display="${display/[[:cntrl:]]/?}"
    done
    printf '%s' "${display}"
}

readonly DEFAULT_DISPLAY_URL="$(redact_url_for_display "${DEFAULT_URL}")"

usage() {
    cat <<EOF
Usage: ./launch-firefox-gpu.sh [URL]

Launch Firefox in a private window with NVIDIA PRIME render offload.

Arguments:
  URL                 Page to open (default: ${DEFAULT_DISPLAY_URL})

Environment:
  FIREFOX_BIN         Firefox executable override
  FIREFOX_LOG_FILE    Browser log override (default: ${DEFAULT_LOG_FILE})
EOF
}

if [[ "${1:-}" == '-h' || "${1:-}" == '--help' ]]; then
    usage
    exit 0
fi
(( $# <= 1 )) || die 'expected at most one URL argument'

readonly TARGET_URL="${1:-${DEFAULT_URL}}"
case "${TARGET_URL}" in
    http://*|https://*) ;;
    *) die 'URL must use http:// or https://' ;;
esac
readonly DISPLAY_URL="$(redact_url_for_display "${TARGET_URL}")"

firefox_candidate="${FIREFOX_BIN:-firefox}"
if [[ "${firefox_candidate}" == */* ]]; then
    [[ -x "${firefox_candidate}" ]] || die "Firefox is not executable: ${firefox_candidate}"
    firefox_executable="${firefox_candidate}"
else
    firefox_executable="$(command -v -- "${firefox_candidate}" || true)"
    [[ -n "${firefox_executable}" ]] || die "Firefox was not found in PATH (requested: ${firefox_candidate})"
fi
readonly FIREFOX_EXECUTABLE="${firefox_executable}"
readonly LOG_FILE="${FIREFOX_LOG_FILE:-${DEFAULT_LOG_FILE}}"

[[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] \
    || die 'no graphical display detected (DISPLAY and WAYLAND_DISPLAY are both unset)'

# Keep the caller's proxy configuration so Firefox can reach external map/tile
# services. Only local simulation endpoints bypass it. Rebuild each no_proxy
# variant independently: existing entries keep their order, duplicates and
# empty fields are removed, and the required loopback hosts are appended.
merge_no_proxy() {
    local existing="${1:-}"
    local entry normalized retained duplicate
    local -a entries=() unique_entries=()

    IFS=',' read -r -a entries <<<"${existing}"
    entries+=('127.0.0.1' 'localhost' '::1')

    for entry in "${entries[@]}"; do
        normalized="${entry#"${entry%%[![:space:]]*}"}"
        normalized="${normalized%"${normalized##*[![:space:]]}"}"
        [[ -n "${normalized}" ]] || continue

        duplicate=false
        for retained in "${unique_entries[@]}"; do
            if [[ "${retained}" == "${normalized}" ]]; then
                duplicate=true
                break
            fi
        done
        [[ "${duplicate}" == false ]] || continue
        unique_entries+=("${normalized}")
    done

    local IFS=','
    printf '%s' "${unique_entries[*]}"
}

no_proxy="$(merge_no_proxy "${no_proxy:-}")"
NO_PROXY="$(merge_no_proxy "${NO_PROXY:-}")"
export no_proxy NO_PROXY

proxy_environment='not set'
for proxy_variable in http_proxy https_proxy all_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY; do
    if [[ -n "${!proxy_variable:-}" ]]; then
        proxy_environment='inherited'
        break
    fi
done
readonly proxy_environment

# Firefox uses the desktop/system proxy on Linux, while this probe uses the
# inherited shell proxy. Both should resolve to the same local Clash/ShellCrash
# path on this workstation. A plain host request contains no API key; any HTTP
# status proves DNS/TLS/transport are working (the root commonly returns 404).
google_tiles_transport='not checked'
if command -v curl >/dev/null 2>&1; then
    google_tiles_http_code="$({
        curl --silent --show-error --output /dev/null --write-out '%{http_code}' \
            --connect-timeout 3 --max-time 6 'https://tile.googleapis.com/'
    } 2>/dev/null || true)"
    if [[ "${google_tiles_http_code}" =~ ^[1-5][0-9][0-9]$ ]]; then
        google_tiles_transport="reachable (HTTP ${google_tiles_http_code})"
    else
        google_tiles_transport='unreachable'
        warn 'Google Tiles transport check failed; verify the active system proxy before flight.'
    fi
fi

# Modern GTK provides the accessibility bridge itself. Ubuntu/Xubuntu may
# export GTK_MODULES=gail:atk-bridge globally, which only produces a harmless
# startup warning in current Firefox.
unset GTK_MODULES

# NVIDIA PRIME render offload. Keep the existing X11/GLX path because it is
# stable on this workstation and is confirmed by `nvidia-smi pmon`.
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export MOZ_X11_EGL=0
export LIBGL_ALWAYS_SOFTWARE=0

firefox_version="$("${FIREFOX_EXECUTABLE}" --version 2>/dev/null | head -n 1 || true)"
firefox_packaging='system'
if [[ "${FIREFOX_EXECUTABLE}" == /usr/bin/firefox ]] \
    && grep -q '/snap/bin/firefox' "${FIREFOX_EXECUTABLE}" 2>/dev/null; then
    firefox_packaging='snap'
elif [[ "${FIREFOX_EXECUTABLE}" == /snap/* ]]; then
    firefox_packaging='snap'
fi

gpu_name='unavailable'
if command -v nvidia-smi >/dev/null 2>&1; then
    gpu_name="$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -n 1 || true)"
    [[ -n "${gpu_name}" ]] || gpu_name='driver unavailable'
else
    warn 'nvidia-smi is unavailable; GPU offload cannot be verified automatically'
fi

# A missing local web server should not prevent opening Firefox, but make the
# likely cause obvious before the browser shows a connection-error page.
if [[ "${TARGET_URL}" == http://127.0.0.1:* || "${TARGET_URL}" == http://localhost:* ]]; then
    if command -v curl >/dev/null 2>&1 \
        && ! curl --noproxy '*' --fail --silent --show-error --max-time 2 "${TARGET_URL}" >/dev/null 2>&1; then
        warn "local web server is not responding yet: ${DISPLAY_URL}"
    fi
fi

mkdir -p -- "$(dirname -- "${LOG_FILE}")"
{
    printf '\n[%s] launch\n' "$(date --iso-8601=seconds)"
    printf 'executable=%s\nversion=%s\npackaging=%s\ngpu=%s\nurl=%s\nproxy_environment=%s\ngoogle_tiles_transport=%s\n' \
        "${FIREFOX_EXECUTABLE}" "${firefox_version:-unknown}" "${firefox_packaging}" \
        "${gpu_name}" "${DISPLAY_URL}" "${proxy_environment}" "${google_tiles_transport}"
} >>"${LOG_FILE}"

info "Firefox: ${firefox_version:-unknown} (${firefox_packaging})"
info "GPU: ${gpu_name}"
info "Proxy environment: ${proxy_environment}; local loopback addresses bypass it."
info "Google Tiles transport: ${google_tiles_transport}."
info "URL: ${DISPLAY_URL}"
info "Log: ${LOG_FILE}"

existing_firefox_pids="$(pgrep -u "$(id -u)" -x firefox 2>/dev/null | paste -sd, - || true)"
if [[ -n "${existing_firefox_pids}" ]]; then
    warn "Firefox is already running (PID(s) ${existing_firefox_pids}); the new window may reuse that process, so its original GPU environment remains authoritative."
fi

nohup "${FIREFOX_EXECUTABLE}" --private-window "${TARGET_URL}" \
    </dev/null >>"${LOG_FILE}" 2>&1 &
firefox_pid=$!

# Give the launcher a brief chance to report a real startup error. A zero exit
# here normally means the URL was handed to an already-running Firefox.
sleep 0.5
if ! kill -0 "${firefox_pid}" 2>/dev/null; then
    set +e
    wait "${firefox_pid}"
    launch_status=$?
    set -e
    if (( launch_status != 0 )); then
        warn "Firefox exited with status ${launch_status}; recent log output follows"
        tail -n 20 -- "${LOG_FILE}" >&2 || true
        exit "${launch_status}"
    fi
    info 'Launch request was handed to an existing Firefox process.'
else
    disown "${firefox_pid}" 2>/dev/null || true
    info "Firefox started in the background (PID ${firefox_pid})."
fi

if command -v nvidia-smi >/dev/null 2>&1; then
    if nvidia-smi pmon -c 1 2>/dev/null | grep -q '[[:space:]]firefox[[:space:]]*$'; then
        info 'NVIDIA graphics process detected.'
    else
        warn 'Firefox is not visible in NVIDIA process monitoring yet; verify about:support → Graphics if rendering is slow.'
    fi
fi
