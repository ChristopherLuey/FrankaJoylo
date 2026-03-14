#!/usr/bin/env bash
#
# FrankaJoylo Setup Script
#
# 1. Identifies USB ports by detecting unplug/replug
# 2. Runs calibration (scans motors, sets offsets)
# 3. Saves config.json
# 4. Launches MuJoCo sim teleop — tune joint signs live in the viewer
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_PATH="$SCRIPT_DIR/config.json"

echo "============================================"
echo "         FrankaJoylo Setup"
echo "============================================"
echo ""

# ------------------------------------------------------------------
# Step 0: Check dependencies
# ------------------------------------------------------------------
echo "--- Checking dependencies ---"
python -c "import franka_joylo" 2>/dev/null || {
    echo "ERROR: franka_joylo not installed. Run: pip install -e ."
    exit 1
}
python -c "import dynamixel_sdk" 2>/dev/null || {
    echo "ERROR: dynamixel-sdk not installed. Run: pip install dynamixel-sdk"
    exit 1
}
python -c "import mujoco" 2>/dev/null || {
    echo "ERROR: mujoco not installed. Run: pip install mujoco"
    exit 1
}
echo "All dependencies OK."
echo ""

# ------------------------------------------------------------------
# Step 1: Identify USB ports
# ------------------------------------------------------------------
echo "============================================"
echo "  Step 1: Identify USB Ports"
echo "============================================"
echo ""
echo "Make sure BOTH Dynamixel controllers are plugged in."
read -rp "Press Enter when ready..."

PORTS_BOTH=$(ls /dev/ttyUSB* 2>/dev/null || true)
NUM_PORTS=$(echo "$PORTS_BOTH" | grep -c "/dev/ttyUSB" || true)

if [ "$NUM_PORTS" -lt 2 ]; then
    echo ""
    echo "ERROR: Expected at least 2 /dev/ttyUSB* ports, found $NUM_PORTS"
    if [ "$NUM_PORTS" -gt 0 ]; then
        echo "Found: $PORTS_BOTH"
    fi
    echo "Check that both controllers are plugged in."
    exit 1
fi

echo ""
echo "Found ports:"
echo "$PORTS_BOTH" | while read -r p; do echo "  $p"; done
echo ""

echo "Now UNPLUG the 5V controller (XL330 — motors 2, 4, 5, 6)."
read -rp "Press Enter after unplugging it..."

PORTS_AFTER=$(ls /dev/ttyUSB* 2>/dev/null || true)

# Find the port that disappeared
PORT_5V_ORIG=""
while IFS= read -r port; do
    if ! echo "$PORTS_AFTER" | grep -qF "$port"; then
        PORT_5V_ORIG="$port"
        break
    fi
done <<< "$PORTS_BOTH"

if [ -z "$PORT_5V_ORIG" ]; then
    echo "ERROR: No port disappeared. Make sure you unplugged a controller."
    exit 1
fi

echo "Detected 5V controller was on: $PORT_5V_ORIG"

# The 12V port is whatever is still connected
PORT_12V=""
while IFS= read -r port; do
    if echo "$PORTS_AFTER" | grep -qF "$port"; then
        PORT_12V="$port"
        break
    fi
done <<< "$PORTS_BOTH"

echo "12V controller is on: $PORT_12V"
echo ""
echo "Now PLUG THE 5V CONTROLLER BACK IN."
read -rp "Press Enter after plugging it back in..."

# Wait for the device to appear
sleep 2

PORTS_FINAL=$(ls /dev/ttyUSB* 2>/dev/null || true)

# Find the new port (the one that wasn't in PORTS_AFTER)
PORT_5V=""
while IFS= read -r port; do
    if ! echo "$PORTS_AFTER" | grep -qF "$port"; then
        PORT_5V="$port"
        break
    fi
done <<< "$PORTS_FINAL"

if [ -z "$PORT_5V" ]; then
    echo "ERROR: 5V controller did not reappear. Check the connection."
    exit 1
fi

echo ""
echo "Port assignment:"
echo "  5V  controller (XL330): $PORT_5V"
echo "  12V controller (XL430): $PORT_12V"
echo ""

# ------------------------------------------------------------------
# Step 2: Calibrate
# ------------------------------------------------------------------
echo "============================================"
echo "  Step 2: Calibration"
echo "============================================"
echo ""

python "$SCRIPT_DIR/examples/calibrate.py" \
    --port-5v "$PORT_5V" \
    --port-12v "$PORT_12V" \
    --output "$CONFIG_PATH"

if [ ! -f "$CONFIG_PATH" ]; then
    echo "ERROR: Calibration did not produce config.json"
    exit 1
fi

echo ""
echo "Config saved to: $CONFIG_PATH"

# ------------------------------------------------------------------
# Step 3: Launch MuJoCo sim teleop
# ------------------------------------------------------------------
echo ""
echo "============================================"
echo "  Step 3: MuJoCo Sim Teleop + Sign Tuning"
echo "============================================"
echo ""
echo "Launching MuJoCo viewer — move the Joylo and watch the sim Franka."
echo "If a joint moves the wrong way, type its number (0-6) to flip it."
echo "Type 's' to save, 'q' to quit."
echo ""

python "$SCRIPT_DIR/examples/sim_teleop.py" --config "$CONFIG_PATH"
