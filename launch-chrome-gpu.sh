#!/usr/bin/env bash
# Launch MindCloud in a persistent, non-incognito Chrome flight profile.
# Prefer verified NVIDIA PRIME rendering and fall back to the desktop GPU.
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly DEFAULT_URL='http://127.0.0.1:8080/?panoProfile=flight&perfProfile=demo30&tileRequestsPerServer=12&panoramaTileSse=256&panoPreloadRequired=0&flightPreloadRadius=400&flightPreloadViewTimeoutMs=60000&flightPreloadViewAttempts=3'
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
  CHROME_GPU_MODE     auto (default), nvidia, or desktop
EOF
    exit 0
fi
(( $# <= 1 )) || die 'expected at most one URL argument'

readonly TARGET_URL="${1:-${DEFAULT_URL}}"
case "${TARGET_URL}" in http://*|https://*) ;; *) die 'URL must use http:// or https://' ;; esac
readonly GPU_MODE="${CHROME_GPU_MODE:-auto}"
case "${GPU_MODE}" in
    auto|nvidia|desktop) ;;
    *) die 'CHROME_GPU_MODE must be auto, nvidia, or desktop' ;;
esac

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
readonly GPU_PROBE_SECONDS=8
[[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || die 'no graphical display detected'

# Chrome keeps profile processes alive after the last window closes. Reusing
# one would also reuse its original GPU flags, so manage only processes that
# carry this launcher's exact user-data-dir or own its SingletonLock. Chrome
# rewrites the stable browser process argv after startup, so the lock PID is
# the durable identity while command-line matching covers the launch window.
profile_lock_pid() {
    local lock_target pid executable
    lock_target="$(readlink -- "${PROFILE_DIR}/SingletonLock" 2>/dev/null || true)"
    pid="${lock_target##*-}"
    [[ "${pid}" =~ ^[0-9]+$ && -O "/proc/${pid}" ]] || return 1
    executable="$(readlink -f -- "/proc/${pid}/exe" 2>/dev/null || true)"
    [[ "${executable}" == */chrome ]] || return 1
    printf '%s\n' "${pid}"
}

find_profile_pids() {
    local proc pid argument owner_pid
    local -A seen=()
    owner_pid="$(profile_lock_pid || true)"
    if [[ -n "${owner_pid}" ]]; then
        seen["${owner_pid}"]=1
        printf '%s\n' "${owner_pid}"
    fi
    for proc in /proc/[0-9]*; do
        [[ -O "${proc}" && -r "${proc}/cmdline" ]] || continue
        pid="${proc##*/}"
        while IFS= read -r -d '' argument; do
            if [[ "${argument}" == "--user-data-dir=${PROFILE_DIR}" ]]; then
                if [[ -z "${seen[${pid}]:-}" ]]; then
                    seen["${pid}"]=1
                    printf '%s\n' "${pid}"
                fi
                break
            fi
        done < "${proc}/cmdline"
    done
}

profile_tree_pids() {
    local root pid child
    local -a queue=() children=()
    local -A seen=()
    root="$(profile_lock_pid || true)"
    [[ -n "${root}" ]] || return 1
    queue+=("${root}")
    while (( ${#queue[@]} > 0 )); do
        pid="${queue[0]}"
        queue=("${queue[@]:1}")
        [[ -z "${seen[${pid}]:-}" ]] || continue
        seen["${pid}"]=1
        printf '%s\n' "${pid}"
        mapfile -t children < <(pgrep -P "${pid}" 2>/dev/null || true)
        for child in "${children[@]}"; do
            [[ -n "${child}" ]] && queue+=("${child}")
        done
    done
}

clear_stale_profile_singletons() {
    local -a pids=()
    mapfile -t pids < <(find_profile_pids)
    (( ${#pids[@]} == 0 )) || return 1
    rm -f -- \
        "${PROFILE_DIR}/SingletonLock" \
        "${PROFILE_DIR}/SingletonCookie" \
        "${PROFILE_DIR}/SingletonSocket"
}

stop_profile_processes() {
    local pid
    local -a pids=() survivors=()
    mapfile -t pids < <(find_profile_pids)
    if (( ${#pids[@]} == 0 )); then
        clear_stale_profile_singletons
        return
    fi

    info "Stopping previous MindCloud Chrome profile (${#pids[@]} process(es))..."
    kill -TERM "${pids[@]}" 2>/dev/null || true
    for _ in $(seq 1 50); do
        mapfile -t survivors < <(find_profile_pids)
        if (( ${#survivors[@]} == 0 )); then
            sleep 0.2
            mapfile -t survivors < <(find_profile_pids)
            if (( ${#survivors[@]} == 0 )); then
                clear_stale_profile_singletons
                return
            fi
        else
            kill -TERM "${survivors[@]}" 2>/dev/null || true
        fi
        sleep 0.1
    done
    warn 'dedicated Chrome processes did not exit cleanly; forcing this profile closed'
    kill -KILL "${survivors[@]}" 2>/dev/null || true
    sleep 0.2
    mapfile -t survivors < <(find_profile_pids)
    (( ${#survivors[@]} == 0 )) || return 1
    clear_stale_profile_singletons
}

wait_for_profile_process() {
    local -a pids=()
    for _ in $(seq 1 20); do
        mapfile -t pids < <(find_profile_pids)
        (( ${#pids[@]} > 0 )) && return 0
        sleep 0.1
    done
    return 1
}

stop_profile_processes || die 'could not stop the previous dedicated Chrome profile'

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

detect_nvidia_provider() {
    command -v xrandr >/dev/null 2>&1 || return 1
    xrandr --listproviders 2>/dev/null \
        | sed -n 's/.*name:\(NVIDIA-G[0-9][0-9]*\).*/\1/p' \
        | head -n 1
}

configure_nvidia_gpu() {
    local provider="$1"
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __NV_PRIME_RENDER_OFFLOAD_PROVIDER="${provider}"
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    unset __VK_LAYER_NV_optimus DRI_PRIME LIBGL_ALWAYS_SOFTWARE
}

configure_desktop_gpu() {
    unset __NV_PRIME_RENDER_OFFLOAD
    unset __NV_PRIME_RENDER_OFFLOAD_PROVIDER
    unset __GLX_VENDOR_LIBRARY_NAME
    unset __VK_LAYER_NV_optimus DRI_PRIME LIBGL_ALWAYS_SOFTWARE
}

if command -v curl >/dev/null 2>&1 \
    && ! curl --noproxy '*' --fail --silent --max-time 2 "${TARGET_URL}" >/dev/null 2>&1; then
    warn 'local web server is not responding yet; run ./start-all.sh first'
fi

mkdir -p -- "${PROFILE_DIR}" "$(dirname -- "${LOG_FILE}")"
chrome_version="$("${chrome_executable}" --version 2>/dev/null || true)"
info "Chrome: ${chrome_version:-unknown}"
info "Profile: ${PROFILE_DIR} (persistent, non-incognito)"
info "Log: ${LOG_FILE}"

COMMON_CHROME_ARGS=(
    "--user-data-dir=${PROFILE_DIR}"
    --new-window
    --ozone-platform=x11
    --use-gl=angle
    --use-angle=gl
    --disable-background-mode
    --disable-extensions
    --disable-sync
    --disable-default-apps
    --no-first-run
    --no-default-browser-check
    --enable-logging=stderr
    --log-level=1
)

LAST_CHROME_PID=''
LAST_LOG_SCAN_START=1
launch_chrome() {
    local mode="$1" provider="${2:-}"
    if [[ "${mode}" == 'nvidia' ]]; then
        configure_nvidia_gpu "${provider}"
    else
        configure_desktop_gpu
    fi

    {
        printf '\n[%s] launch mode=%s provider=%s\n' \
            "$(date --iso-8601=seconds)" "${mode}" "${provider:-desktop}"
    } >>"${LOG_FILE}"
    LAST_LOG_SCAN_START=$(( $(wc -l < "${LOG_FILE}") + 1 ))

    nohup "${chrome_executable}" "${COMMON_CHROME_ARGS[@]}" "${TARGET_URL}" \
        </dev/null >>"${LOG_FILE}" 2>&1 &
    LAST_CHROME_PID=$!
    disown "${LAST_CHROME_PID}" 2>/dev/null || true
}

log_has_gpu_failure() {
    local first_line="$1"
    tail -n "+${first_line}" -- "${LOG_FILE}" 2>/dev/null \
        | grep -Eqi 'Invalid visual ID requested|GLDisplayEGL::Initialize failed|GPU process (isn.t usable|exited unexpectedly)|Exiting GPU process|use-gl=disabled|SwiftShader'
}

profile_pids_on_nvidia() {
    command -v nvidia-smi >/dev/null 2>&1 || return 1
    local pmon monitored pid
    local -a pids=()
    mapfile -t pids < <(profile_tree_pids)
    (( ${#pids[@]} > 0 )) || return 1
    pmon="$(nvidia-smi pmon -c 1 2>/dev/null || true)"
    monitored="$(awk 'NR > 2 && $2 ~ /^[0-9]+$/ {print $2}' <<<"${pmon}")"
    for pid in "${pids[@]}"; do
        grep -qx -- "${pid}" <<<"${monitored}" && return 0
    done
    return 1
}

PROBE_FAILURE_REASON=''
probe_nvidia_launch() {
    local first_line="$1" second
    local -a pids=()
    for (( second = 1; second <= GPU_PROBE_SECONDS; second++ )); do
        sleep 1
        if log_has_gpu_failure "${first_line}"; then
            PROBE_FAILURE_REASON='ANGLE/EGL/GPU initialization error in Chrome log'
            return 1
        fi
        mapfile -t pids < <(find_profile_pids)
        if (( second >= 2 && ${#pids[@]} == 0 )); then
            PROBE_FAILURE_REASON='dedicated Chrome profile exited during startup'
            return 1
        fi
        if profile_pids_on_nvidia; then
            return 0
        fi
    done
    PROBE_FAILURE_REASON="no dedicated Chrome process appeared on NVIDIA within ${GPU_PROBE_SECONDS}s"
    return 1
}

nvidia_provider=''
if [[ "${GPU_MODE}" != 'desktop' ]]; then
    nvidia_provider="$(detect_nvidia_provider || true)"
fi

final_gpu_mode=''
if [[ "${GPU_MODE}" == 'desktop' ]]; then
    info 'GPU request: desktop (explicit)'
    launch_chrome desktop
    wait_for_profile_process || die 'Chrome exited during desktop-GPU startup'
    final_gpu_mode='desktop'
elif [[ -z "${nvidia_provider}" ]]; then
    if [[ "${GPU_MODE}" == 'nvidia' ]]; then
        die 'NVIDIA-G0 PRIME provider was not found; refusing strict NVIDIA launch'
    fi
    warn 'NVIDIA-G0 PRIME provider was not found; using the desktop GPU'
    launch_chrome desktop
    wait_for_profile_process || die 'Chrome exited during desktop-GPU startup'
    final_gpu_mode='desktop fallback'
else
    info "GPU request: NVIDIA PRIME (${nvidia_provider}, ANGLE OpenGL)"
    launch_chrome nvidia "${nvidia_provider}"
    nvidia_log_start="${LAST_LOG_SCAN_START}"
    if probe_nvidia_launch "${nvidia_log_start}"; then
        final_gpu_mode="NVIDIA PRIME (${nvidia_provider})"
    else
        stop_profile_processes || die 'could not stop the failed NVIDIA Chrome profile'
        if [[ "${GPU_MODE}" == 'nvidia' ]]; then
            die "NVIDIA Chrome verification failed: ${PROBE_FAILURE_REASON}"
        fi
        warn "NVIDIA Chrome verification failed: ${PROBE_FAILURE_REASON}; restarting with the desktop GPU"
        launch_chrome desktop
        desktop_log_start="${LAST_LOG_SCAN_START}"
        wait_for_profile_process || die 'Chrome exited during desktop-GPU fallback'
        sleep 1
        if log_has_gpu_failure "${desktop_log_start}"; then
            stop_profile_processes || true
            die 'Chrome desktop-GPU fallback also reported a GPU initialization failure'
        fi
        final_gpu_mode='desktop fallback'
    fi
fi

info "Final GPU mode: ${final_gpu_mode}."
info "Chrome started in the background (launcher PID ${LAST_CHROME_PID})."
