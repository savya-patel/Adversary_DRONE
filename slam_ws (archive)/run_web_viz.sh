#!/bin/bash
# Helper script to run web visualization for SLAM
# This combines SLAM + rosbridge + web server

set -e
set -o pipefail

echo "╔════════════════════════════════════════════════════════════╗"
echo "║  SICK TIM561 Web Polar Visualization Setup                ║"
echo "╠════════════════════════════════════════════════════════════╣"
echo "║  This will start 3 processes in background:               ║"
echo "║  1. SLAM (slam_config slam_usb.launch.py)                  ║"
echo "║  2. Rosbridge websocket (port 9090 by default)             ║"
echo "║  3. Web server (port 8080 by default)                      ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
echo "Press Ctrl+C to stop all processes"
echo ""

# Cleanup function with guard to avoid double-run
CLEANED_UP=0
cleanup() {
    if [ "$CLEANED_UP" -eq 1 ]; then
        exit 0
    fi
    CLEANED_UP=1
    echo ""
    echo "[*] Stopping all processes..."
    # Kill web server (runs as regular user)
    [ -n "${WEB_PID:-}" ] && kill "$WEB_PID" 2>/dev/null || true
    # Kill sudo processes (requires sudo)
    if [ -n "${SLAM_PID:-}" ]; then kill "$SLAM_PID" 2>/dev/null || sudo kill "$SLAM_PID" 2>/dev/null || true; fi
    if [ -n "${BRIDGE_PID:-}" ]; then kill "$BRIDGE_PID" 2>/dev/null || sudo kill "$BRIDGE_PID" 2>/dev/null || true; fi
    # Wait for cleanup
    sleep 1
    echo "[*] All processes stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Determine ROS distro and ensure ros2 is available
ROS_DISTRO=${ROS_DISTRO:-humble}
if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  echo "[!] /opt/ros/$ROS_DISTRO/setup.bash not found. Export ROS_DISTRO or install ROS 2."
  exit 1
fi

# Basic sanity check for ros2
if ! command -v ros2 >/dev/null 2>&1; then
  # Try to source for current shell so local checks can run
  # Not exported globally; just for subshells below we use explicit source
  echo "[i] ros2 not in PATH; will source /opt/ros/$ROS_DISTRO/setup.bash in subshells."
fi

# Optional: allow user to override SLAM launch file via first arg
SLAM_LAUNCH=${1:-slam_usb.launch.py}

# Helper: check if a TCP port is in use
port_in_use() {
    local port=$1
    if command -v ss >/dev/null 2>&1; then
        ss -ltnH 2>/dev/null | awk -v p=":"${port}" '$4 ~ p { f=1 } END { exit !f }'
    elif command -v netstat >/dev/null 2>&1; then
        netstat -ltn 2>/dev/null | awk -v p=":"${port}" '$4 ~ p { f=1 } END { exit !f }'
    else
        echo "[!] Neither ss nor netstat available; cannot test port ${port}. Assuming free." >&2
        return 1
    fi
}

# Helper: find a free port from a list
pick_free_port() {
    for p in "$@"; do
        if ! port_in_use "$p"; then
            echo "$p"
            return 0
        fi
    done
    return 1
}

# Best-effort: clean up any stale rosbridge on default port
if port_in_use 9090; then
    echo "[*] Port 9090 is in use. Attempting to stop existing rosbridge/websocket on 9090..."
    if command -v lsof >/dev/null 2>&1; then
        PIDS=$(sudo lsof -nP -iTCP:9090 -sTCP:LISTEN -t 2>/dev/null)
        if [ -n "$PIDS" ]; then
            echo "[*] Killing PIDs on 9090: $PIDS"
            sudo kill $PIDS 2>/dev/null || true
            sleep 1
        fi
    fi
fi

# Decide rosbridge websocket port (prefer 9090, else 9091/9092)
BRIDGE_PORT=$(pick_free_port 9090 9091 9092)
if [ -z "$BRIDGE_PORT" ]; then
    echo "[!] No free websocket port found (9090-9092). Exiting."
    exit 2
fi

# Decide web server port (prefer 8080, else 8081/8082/8083)
if port_in_use 8080; then
    WEB_PORT=$(pick_free_port 8081 8082 8083)
    if [ -z "$WEB_PORT" ]; then
        echo "[!] No free web port found (8080-8083). Exiting."
        exit 3
    fi
else
    WEB_PORT=8080
fi

# Prepare logs
mkdir -p log
SLAM_LOG="log/slam.log"
BRIDGE_LOG="log/rosbridge.log"
WEB_LOG="log/web_viz.log"

# Start SLAM (log output for cleanliness)
echo "[1/3] Starting SLAM (launch: $SLAM_LAUNCH)..."
echo "    logs -> $SLAM_LOG"
sudo -E bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source install/setup.bash && ros2 launch slam_config $SLAM_LAUNCH" > "$SLAM_LOG" 2>&1 &
SLAM_PID=$!
sleep 5

# Wait for /scan to appear (up to 20s), using the same sudo ROS env
SCAN_READY=0
for i in $(seq 1 20); do
    if sudo -E bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source install/setup.bash && ros2 topic list | grep -qx '/scan'"; then
        SCAN_READY=1
        break
    fi
    sleep 1
done
if [ "$SCAN_READY" -ne 1 ]; then
    echo "[!] /scan not detected after 20s. Continuing anyway; web viz may show no data until it appears."
fi

# Start rosbridge
echo "[2/3] Starting rosbridge websocket on port ${BRIDGE_PORT}..."
echo "    logs -> $BRIDGE_LOG"
sudo -E bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=${BRIDGE_PORT} address:=0.0.0.0" > "$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!
sleep 1

# Wait for rosbridge to bind to the port (up to 10s)
for i in $(seq 1 20); do
    if port_in_use "$BRIDGE_PORT"; then
        break
    fi
    sleep 0.5
done

# Start web server
echo "[3/3] Starting web visualization server on port ${WEB_PORT}..."
echo "    logs -> $WEB_LOG"
python3 -u lidar_web_viz.py --port "${WEB_PORT}" > "$WEB_LOG" 2>&1 &
WEB_PID=$!
sleep 2

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
if port_in_use "$BRIDGE_PORT"; then
    echo "║  ✅ All services running!                                  ║"
else
    echo "║  ⚠️ Rosbridge may not be running (port busy).            ║"
fi
echo "╠════════════════════════════════════════════════════════════╣"
HOST_IP=$(hostname -I 2>/dev/null | awk '{print $1}' | tr -d '[:space:]')
if [ -z "$HOST_IP" ]; then HOST_IP=localhost; fi
echo "║  Open in browser: http://${HOST_IP}:${WEB_PORT}/?rosbridge_port=${BRIDGE_PORT}        ║"
echo "║                   or http://localhost:${WEB_PORT}/?rosbridge_port=${BRIDGE_PORT}     ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
echo "Logs:"
echo "  SLAM      -> $SLAM_LOG"
echo "  Rosbridge -> $BRIDGE_LOG"
echo "  Web       -> $WEB_LOG"
echo ""
echo "Waiting... (Press Ctrl+C to stop)"

# Wait for user interrupt
wait
