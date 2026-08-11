#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${IMAGE:-google-tiles-flight:latest}"
NAME="${NAME:-google-tiles-flight}"
PORT="${PORT:-8080}"
DETACH="${DETACH:-0}"
MODE="docker"
OPEN_BROWSER=1
LOOPBACK_HOST="127.0.0.1"

RULE_FILE="/etc/udev/rules.d/99-google-tiles-flight-input.rules"
LOCAL_PID=""
LOG_PID=""
CONTAINER_STARTED=0
_CLEANED=0

usage() {
    cat <<EOF
Usage:
  ./launch.sh                 Build/run Docker, then open the browser
  ./launch.sh --no-open       Start the server only
  ./launch.sh --local         Use scripts/serve.py for local development
  ./launch.sh --setup-input   Install Linux udev rules for RC/WebHID access

Options:
  --docker                    Use Docker mode (default)
  --local                     Use local Python server
  --no-open, no-open          Do not open Chrome/Chromium
  --detach                    Keep Docker container running in the background
  --port PORT                 Host port, same as PORT=18081 ./launch.sh
  --image IMAGE               Docker image name
  --name NAME                 Docker container name
  --input-status              Print controller/HID status and exit
  -h, --help                  Show this help

Environment:
  PORT=18081 ./launch.sh
  DETACH=1 ./launch.sh
  CHROME_BIN=/path/to/chrome ./launch.sh
  USE_NVIDIA=1 ./launch.sh
EOF
}

die() {
    echo "ERROR: $*" >&2
    exit 1
}

truthy() {
    case "${1:-}" in
        1|true|TRUE|yes|YES|y|Y) return 0 ;;
        *) return 1 ;;
    esac
}

find_port_listeners() {
    local port="$1"

    if command -v ss >/dev/null 2>&1; then
        ss -ltnp "sport = :$port" 2>/dev/null | awk 'NR > 1 { print }'
        return 0
    fi

    if command -v lsof >/dev/null 2>&1; then
        lsof -nP -iTCP:"$port" -sTCP:LISTEN 2>/dev/null | awk 'NR > 1 { print }'
        return 0
    fi

    return 0
}

find_repo_local_server_pids() {
    local port="${1:-}"
    local pid arg cwd has_serve has_port
    local serve_path="$SCRIPT_DIR/scripts/serve.py"

    while IFS= read -r pid; do
        [[ -n "$pid" && "$pid" != "$$" ]] || continue
        [[ -r "/proc/$pid/cmdline" ]] || continue

        has_serve=0
        if [[ -n "$port" ]]; then
            has_port=0
        else
            has_port=1
        fi
        cwd=""

        while IFS= read -r -d '' arg; do
            if [[ "$arg" == "$serve_path" ]]; then
                has_serve=1
            elif [[ "$arg" == "scripts/serve.py" || "$arg" == "./scripts/serve.py" ]]; then
                [[ -n "$cwd" ]] || cwd="$(readlink "/proc/$pid/cwd" 2>/dev/null || true)"
                [[ "$cwd" == "$SCRIPT_DIR" ]] && has_serve=1
            fi

            if [[ -n "$port" && "$arg" == "$port" ]]; then
                has_port=1
            fi
        done < "/proc/$pid/cmdline"

        if [[ "$has_serve" == "1" && "$has_port" == "1" ]]; then
            printf '%s\n' "$pid"
        fi
    done < <(pgrep -f 'serve.py' 2>/dev/null || true)
}

stop_repo_local_servers() {
    local port="${1:-}"
    local -a pids=()
    local -a survivors=()
    local pid

    mapfile -t pids < <(find_repo_local_server_pids "$port")
    ((${#pids[@]} > 0)) || return 1

    echo "Stopping previous local server..."
    kill -TERM "${pids[@]}" 2>/dev/null || true
    sleep 0.3

    for pid in "${pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            survivors+=("$pid")
        fi
    done

    if ((${#survivors[@]} > 0)); then
        kill -KILL "${survivors[@]}" 2>/dev/null || true
    fi
}

port_is_listening() {
    local port="$1"

    if command -v ss >/dev/null 2>&1; then
        ss -ltn "sport = :$port" 2>/dev/null | awk 'NR > 1 { found = 1 } END { exit found ? 0 : 1 }'
        return $?
    fi

    if command -v lsof >/dev/null 2>&1; then
        lsof -nP -iTCP:"$port" -sTCP:LISTEN >/dev/null 2>&1
        return $?
    fi

    command -v python3 >/dev/null 2>&1 || return 1
    python3 - "$port" <<'PY' >/dev/null 2>&1
import socket
import sys

port = int(sys.argv[1])
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(0.2)
try:
    sys.exit(0 if sock.connect_ex(("127.0.0.1", port)) == 0 else 1)
finally:
    sock.close()
PY
}

suggest_free_port() {
    local start="$1"
    local port="$start"
    local limit=$((start + 30))

    while ((port <= limit)); do
        if ! port_is_listening "$port"; then
            echo "$port"
            return 0
        fi
        port=$((port + 1))
    done

    echo "$((start + 1))"
}

print_port_conflict_help() {
    local port="$1"
    local free_port
    free_port="$(suggest_free_port "$((port + 1))")"

    echo
    echo "Port $port is already in use, so the simulator cannot bind http://$LOOPBACK_HOST:$port."
    echo
    echo "What is using it:"
    local listeners
    listeners="$(find_port_listeners "$port" || true)"
    if [[ -n "$listeners" ]]; then
        echo "$listeners" | sed 's/^/  /'
    else
        echo "  Could not identify the listener. Try: ss -ltnp 'sport = :$port'"
    fi
    echo
    echo "Fix options:"
    echo "  1. Stop an old simulator container:"
    echo "     docker rm -f $NAME"
    echo "  2. Stop a local dev server from this repo:"
    echo "     pkill -f 'scripts/serve.py[[:space:]]+$port'"
    echo "  3. See the process using the port:"
    echo "     ss -ltnp 'sport = :$port'"
    echo "     sudo ss -ltnp 'sport = :$port'   # if the process name/PID is hidden"
    echo "  4. Start on another port:"
    echo "     PORT=$free_port ./launch.sh"
    echo "     ./launch.sh --port $free_port"
    echo
}

ensure_port_available() {
    local port="$1"
    if port_is_listening "$port"; then
        print_port_conflict_help "$port" >&2
        exit 1
    fi
}

cleanup() {
    local code=$?
    [[ "$_CLEANED" == "1" ]] && exit "$code"
    _CLEANED=1
    trap - INT TERM EXIT

    if [[ -n "$LOCAL_PID" ]]; then
        echo
        echo "Stopping local server..."
        kill -TERM "$LOCAL_PID" 2>/dev/null || true
        sleep 0.3
        kill -KILL "$LOCAL_PID" 2>/dev/null || true
    fi

    if [[ "$CONTAINER_STARTED" == "1" ]] && ! truthy "$DETACH"; then
        echo
        echo "Stopping Docker container $NAME and releasing port $PORT..."
        docker rm -f "$NAME" >/dev/null 2>&1 || true
    fi

    if [[ -n "$LOG_PID" ]]; then
        wait "$LOG_PID" 2>/dev/null || true
    fi

    exit "$code"
}

js_device_name() {
    local js_base="${1##*/}"
    [[ -r /proc/bus/input/devices ]] || return 0
    awk -v target="$js_base" '
        BEGIN { RS=""; FS="\n" }
        $0 ~ ("Handlers=.*" target) {
            for (i = 1; i <= NF; i++) {
                if ($i ~ /^N: Name=/) {
                    name = $i
                    sub(/^N: Name="/, "", name)
                    sub(/"$/, "", name)
                    print name
                    exit
                }
            }
        }
    ' /proc/bus/input/devices
}

print_input_status() {
    local js_devices=()
    local hid_devices=()
    local blocked_hid=()
    local dev

    shopt -s nullglob
    js_devices=(/dev/input/js*)
    hid_devices=(/dev/hidraw*)
    shopt -u nullglob

    echo "Input devices:"

    if ((${#js_devices[@]} > 0)); then
        echo "  Gamepad API: found ${#js_devices[@]} joystick device(s)."
        for dev in "${js_devices[@]}"; do
            local name
            name="$(js_device_name "$dev" || true)"
            if [[ -n "$name" ]]; then
                echo "    - $dev ($name)"
            else
                echo "    - $dev"
            fi
        done
    else
        echo "  Gamepad API: no /dev/input/js* joystick detected."
    fi

    if ((${#hid_devices[@]} > 0)); then
        for dev in "${hid_devices[@]}"; do
            [[ -r "$dev" && -w "$dev" ]] || blocked_hid+=("$dev")
        done
        if ((${#blocked_hid[@]} > 0)); then
            echo "  WebHID: ${#blocked_hid[@]} hidraw device(s) need permission."
            echo "          Run: ./launch.sh --setup-input"
        else
            echo "  WebHID: hidraw permissions look usable."
        fi
    else
        echo "  WebHID: no hidraw device detected yet."
    fi

    if ((${#js_devices[@]} == 0 && ${#hid_devices[@]} == 0)); then
        echo "  No controller is connected. Keyboard control is available."
        echo "  You can plug in a gamepad later and refresh, or use Settings -> Connect HID."
    fi
}

setup_input_rules() {
    local target_user="${TARGET_USER:-${SUDO_USER:-${USER:-}}}"

    if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
        command -v sudo >/dev/null 2>&1 || die "sudo is required to install udev rules."
        [[ -n "$target_user" ]] || target_user="$(id -un)"
        echo "Installing input permission rules with sudo..."
        exec sudo env TARGET_USER="$target_user" "$SCRIPT_DIR/launch.sh" --setup-input
    fi

    if ! getent group plugdev >/dev/null 2>&1; then
        groupadd plugdev 2>/dev/null || true
    fi

    local tmp
    tmp="$(mktemp)"
    cat > "$tmp" <<'EOF'
# Google 3D Tiles Flight input access.
# Allows Chrome Gamepad API and WebHID to read common RC transmitter devices.
ACTION=="add|change", SUBSYSTEM=="input", ENV{ID_INPUT_JOYSTICK}=="1", TAG+="uaccess", MODE="0660"

# RadioMaster / OpenTX / EdgeTX / common STM32 HID devices.
ACTION=="add|change", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="1209", TAG+="uaccess", MODE="0660"
ACTION=="add|change", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0483", TAG+="uaccess", MODE="0660"

# FrSky and Jumper-style USB HID.
ACTION=="add|change", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="2341", TAG+="uaccess", MODE="0660"

# Fallback for systems that rely on the plugdev group.
ACTION=="add|change", SUBSYSTEM=="hidraw", GROUP="plugdev", MODE="0660"
EOF
    install -m 0644 "$tmp" "$RULE_FILE"
    rm -f "$tmp"

    if [[ -n "$target_user" ]] && id "$target_user" >/dev/null 2>&1 && getent group plugdev >/dev/null 2>&1; then
        usermod -aG plugdev "$target_user" || true
    fi

    if command -v udevadm >/dev/null 2>&1; then
        udevadm control --reload-rules || true
        udevadm trigger || true
    fi

    echo "Installed: $RULE_FILE"
    if [[ -n "$target_user" ]]; then
        echo "User $target_user is in plugdev if the group exists."
    fi
    echo "Reconnect the RC transmitter/gamepad. If group membership changed, log out and back in."
}

find_browser() {
    if [[ -n "${CHROME_BIN:-}" ]]; then
        [[ -x "$CHROME_BIN" || -n "$(command -v "$CHROME_BIN" 2>/dev/null)" ]] && echo "$CHROME_BIN"
        return 0
    fi

    local candidate
    for candidate in /opt/google/chrome/chrome google-chrome google-chrome-stable chromium chromium-browser; do
        if command -v "$candidate" >/dev/null 2>&1; then
            command -v "$candidate"
            return 0
        elif [[ -x "$candidate" ]]; then
            echo "$candidate"
            return 0
        fi
    done
}

open_browser() {
    local url="$1"
    [[ "$OPEN_BROWSER" == "1" ]] || {
        echo "Browser launch skipped. Open $url"
        return 0
    }

    local browser
    browser="$(find_browser || true)"
    if [[ -z "$browser" ]]; then
        echo "Chrome/Chromium was not found. Open $url manually."
        return 0
    fi

    echo "Opening $url"
    if truthy "${USE_NVIDIA:-0}" || [[ -e /proc/driver/nvidia/version ]] || command -v nvidia-smi >/dev/null 2>&1; then
        __NV_PRIME_RENDER_OFFLOAD=1 \
        __GLX_VENDOR_LIBRARY_NAME=nvidia \
        __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
            nohup "$browser" \
                --enable-gpu-rasterization \
                --ignore-gpu-blocklist \
                "$url" >/dev/null 2>&1 &
    else
        nohup "$browser" \
            --enable-gpu-rasterization \
            --ignore-gpu-blocklist \
            "$url" >/dev/null 2>&1 &
    fi
    disown || true
}

run_docker() {
    command -v docker >/dev/null 2>&1 || die "Docker is not installed."
    docker info >/dev/null 2>&1 || die "Cannot access Docker daemon. Start Docker or run with sudo if your setup requires it."

    mkdir -p "$SCRIPT_DIR/asset/gate-paths"

    trap cleanup INT TERM EXIT

    docker rm -f "$NAME" >/dev/null 2>&1 || true
    stop_repo_local_servers "$PORT" || true
    ensure_port_available "$PORT"

    echo "Building Docker image: $IMAGE"
    docker build -f "$SCRIPT_DIR/Dockerfile.cesium" -t "$IMAGE" "$SCRIPT_DIR"

    local url="http://$LOOPBACK_HOST:$PORT"

    echo "Starting Docker container $NAME at $url"
    local docker_output
    if ! docker_output="$(docker run --rm -d --init \
        --name "$NAME" \
        -p "127.0.0.1:$PORT:8000" \
        -e "CESIUM_ION_TOKEN=${CESIUM_ION_TOKEN:-}" \
        -v "$SCRIPT_DIR/asset/gate-paths:/var/www/google-tiles-flight/asset/gate-paths" \
        "$IMAGE" 2>&1)"; then
        echo "$docker_output" >&2
        if [[ "$docker_output" == *"address already in use"* || "$docker_output" == *"port is already allocated"* || "$docker_output" == *"failed to bind"* ]]; then
            print_port_conflict_help "$PORT" >&2
        fi
        exit 1
    fi
    CONTAINER_STARTED=1

    open_browser "$url"

    cat <<EOF

Simulator: $url
Stop:      Ctrl+C
Input:     keyboard works; gamepad/WebHID is optional.
EOF

    if truthy "$DETACH"; then
        echo "Docker container $NAME is running in the background."
        trap - EXIT
        exit 0
    fi

    docker logs -f "$NAME" &
    LOG_PID=$!
    wait "$LOG_PID"
}

run_local() {
    command -v python3 >/dev/null 2>&1 || die "python3 is not installed."

    trap cleanup INT TERM EXIT

    stop_repo_local_servers "$PORT" || true

    ensure_port_available "$PORT"

    local url="http://$LOOPBACK_HOST:$PORT"

    echo "Starting local server at $url"
    python3 "$SCRIPT_DIR/scripts/serve.py" "$PORT" &
    LOCAL_PID=$!
    sleep 0.4
    kill -0 "$LOCAL_PID" 2>/dev/null || die "Local server failed to start."

    open_browser "$url"

    cat <<EOF

Simulator: $url
Stop:      Ctrl+C
Input:     keyboard works; gamepad/WebHID is optional.
EOF

    wait "$LOCAL_PID"
}

while (($# > 0)); do
    case "$1" in
        --docker)
            MODE="docker"
            ;;
        --local|local)
            MODE="local"
            ;;
        --no-open|no-open)
            OPEN_BROWSER=0
            ;;
        --detach)
            DETACH=1
            ;;
        --port)
            (($# >= 2)) || die "--port requires a value."
            PORT="$2"
            shift
            ;;
        --port=*)
            PORT="${1#*=}"
            ;;
        --image)
            (($# >= 2)) || die "--image requires a value."
            IMAGE="$2"
            shift
            ;;
        --image=*)
            IMAGE="${1#*=}"
            ;;
        --name)
            (($# >= 2)) || die "--name requires a value."
            NAME="$2"
            shift
            ;;
        --name=*)
            NAME="${1#*=}"
            ;;
        --setup-input)
            setup_input_rules
            exit 0
            ;;
        --input-status)
            print_input_status
            exit 0
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            usage >&2
            die "Unknown option: $1"
            ;;
    esac
    shift
done

print_input_status
echo

case "$MODE" in
    docker) run_docker ;;
    local) run_local ;;
    *) die "Unknown mode: $MODE" ;;
esac
