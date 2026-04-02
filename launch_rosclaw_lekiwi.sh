#!/bin/bash
#
# ROSClaw LeKiwi Launch Script
#
# Launches:
#   1. lekiwi_motor_bridge  — Feetech servo communication (ROS2 node)
#   2. digital_twin_viewer  — Live MuJoCo visualization (optional)
#
# The MCP server is NOT launched here — Claude Code starts its own
# instance when you connect via: claude mcp add rosclaw-lekiwi ...
#
# Usage:
#   ./launch_rosclaw_lekiwi.sh                      # motor bridge only
#   ./launch_rosclaw_lekiwi.sh --viewer             # motor bridge + digital twin
#   ./launch_rosclaw_lekiwi.sh --port /dev/ttyACM1  # custom serial port
#

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROSCLAW_DIR="$(dirname "$SCRIPT_DIR")"
SPECS_DIR="$SCRIPT_DIR/specs"
LEKIWI_ROS2_DIR="$ROSCLAW_DIR/lekiwi_ros2"
VENV="$HOME/rosclaw-venv"

# Defaults
PORT="/dev/ttyACM0"
USE_SO101="true"
LAUNCH_VIEWER=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --viewer)
            LAUNCH_VIEWER=true
            shift
            ;;
        --port)
            PORT="$2"
            shift 2
            ;;
        --base-only)
            USE_SO101="false"
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [--viewer] [--port /dev/ttyACMx] [--base-only]"
            echo ""
            echo "Options:"
            echo "  --viewer      Launch live MuJoCo digital twin viewer"
            echo "  --port PATH   Serial port (default: /dev/ttyACM0)"
            echo "  --base-only   Don't look for SO-101 arm motors"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Track PIDs for cleanup
PIDS=()

cleanup() {
    echo ""
    echo "Shutting down ROSClaw LeKiwi..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null
            wait "$pid" 2>/dev/null
        fi
    done
    echo "All processes stopped."
}
trap cleanup EXIT INT TERM

# Source environments
source /opt/ros/jazzy/setup.bash
source "$VENV/bin/activate"

if [ -f "$LEKIWI_ROS2_DIR/install/setup.bash" ]; then
    source "$LEKIWI_ROS2_DIR/install/setup.bash"
else
    echo "ERROR: lekiwi_ros2 workspace not built."
    echo "Run: cd $LEKIWI_ROS2_DIR && colcon build"
    exit 1
fi

# Check serial port
if [ ! -e "$PORT" ]; then
    echo "ERROR: Serial port $PORT not found."
    echo "Available ports:"
    ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "  (none)"
    exit 1
fi

if [ ! -r "$PORT" ] || [ ! -w "$PORT" ]; then
    echo "WARNING: No permission on $PORT — trying sudo chmod..."
    sudo chmod 666 "$PORT"
fi

echo "======================================"
echo "  ROSClaw LeKiwi Launch"
echo "======================================"
echo "  Port:    $PORT"
echo "  SO-101:  $USE_SO101"
echo "  Viewer:  $LAUNCH_VIEWER"
echo "  Specs:   $SPECS_DIR"
echo "======================================"
echo ""

# 1. Launch motor bridge
echo "[1/2] Starting motor bridge..."
python3 -m lekiwi_hw_interface.lekiwi_motor_bridge --ros-args \
    -p port:="$PORT" \
    -p use_so101:="$USE_SO101" &
PIDS+=($!)
sleep 2

# Check it started
if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
    echo "ERROR: Motor bridge failed to start."
    exit 1
fi
echo "[1/2] Motor bridge running (PID: ${PIDS[-1]})"

# 2. Launch digital twin viewer (optional)
if [ "$LAUNCH_VIEWER" = true ]; then
    echo "[2/2] Starting Digital Twin Viewer..."
    cd "$SPECS_DIR"
    python3 "$SCRIPT_DIR/src/lekiwi_digital_twin_viewer.py" &
    PIDS+=($!)
    sleep 2
    if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
        echo "WARNING: Digital Twin Viewer failed to start (continuing without it)"
    else
        echo "[2/2] Digital Twin Viewer running (PID: ${PIDS[-1]})"
    fi
else
    echo "[2/2] Digital Twin Viewer skipped (use --viewer to enable)"
fi

echo ""
echo "======================================"
echo "  ROSClaw LeKiwi is LIVE"
echo "======================================"
echo ""
echo "  ROS2 topics:"
echo "    /joint_states     — robot state (50Hz)"
echo "    /joint_commands   — arm position commands"
echo "    /cmd_vel          — base velocity commands"
echo ""
echo "  Connect Claude Code (run once in another terminal):"
echo "    claude mcp add rosclaw-lekiwi -- bash -c \\"
echo "      \"source ~/rosclaw-venv/bin/activate && \\"
echo "       source /opt/ros/jazzy/setup.bash && \\"
echo "       cd $SPECS_DIR && \\"
echo "       python3 $SCRIPT_DIR/src/lekiwi_mcp_server.py\""
echo ""
echo "  Then start Claude Code:"
echo "    claude"
echo ""
echo "  Press Ctrl+C to stop all processes."
echo ""

# Wait for any background process to exit
wait -n "${PIDS[@]}" 2>/dev/null
echo "A process exited — shutting down."
