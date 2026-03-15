#!/usr/bin/env bash
#
# FrankaJoylo Setup Script
#
# 1. Checks for /dev/franka_small and /dev/franka_large symlinks
#    - If missing, identifies USB ports via unplug/replug and installs udev rules
# 2. Runs calibration (scans motors, saves config.json)
# 3. Launches MuJoCo sim teleop — tune joint signs live in the viewer
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_PATH="$SCRIPT_DIR/config.json"

SYMLINK_SMALL="/dev/franka_small"
SYMLINK_LARGE="/dev/franka_large"
UDEV_RULES_FILE="/etc/udev/rules.d/99-franka-joylo.rules"

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
# Step 1: Ensure /dev/franka_small and /dev/franka_large exist
# ------------------------------------------------------------------
if [ -e "$SYMLINK_SMALL" ] && [ -e "$SYMLINK_LARGE" ]; then
    echo "Found $SYMLINK_SMALL and $SYMLINK_LARGE — skipping port setup."
    PORT_5V="$SYMLINK_SMALL"
    PORT_12V="$SYMLINK_LARGE"
else
    echo "============================================"
    echo "  Step 1: Identify USB Ports & Install udev"
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

    echo "Now UNPLUG the 5V controller (XL330 — motors 0-3)."
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

    # The 12V port is whatever is still connected
    PORT_12V_TMP=""
    while IFS= read -r port; do
        if echo "$PORTS_AFTER" | grep -qF "$port"; then
            PORT_12V_TMP="$port"
            break
        fi
    done <<< "$PORTS_BOTH"

    echo "  5V  was on: $PORT_5V_ORIG"
    echo "  12V is on:  $PORT_12V_TMP"

    echo ""
    echo "Now PLUG THE 5V CONTROLLER BACK IN."
    read -rp "Press Enter after plugging it back in..."

    # Wait for the device to appear
    sleep 2

    PORTS_FINAL=$(ls /dev/ttyUSB* 2>/dev/null || true)

    # Find the new port (the one that wasn't in PORTS_AFTER)
    PORT_5V_TMP=""
    while IFS= read -r port; do
        if ! echo "$PORTS_AFTER" | grep -qF "$port"; then
            PORT_5V_TMP="$port"
            break
        fi
    done <<< "$PORTS_FINAL"

    if [ -z "$PORT_5V_TMP" ]; then
        echo "ERROR: 5V controller did not reappear. Check the connection."
        exit 1
    fi

    echo "  5V  now on: $PORT_5V_TMP"

    # Create symlinks immediately
    echo ""
    echo "Creating symlinks (requires sudo)..."
    sudo ln -sf "$PORT_5V_TMP" "$SYMLINK_SMALL"
    sudo ln -sf "$PORT_12V_TMP" "$SYMLINK_LARGE"

    # Verify symlinks resolve correctly
    RESOLVED_SMALL=$(readlink -f "$SYMLINK_SMALL")
    RESOLVED_LARGE=$(readlink -f "$SYMLINK_LARGE")
    echo "  $SYMLINK_SMALL -> $RESOLVED_SMALL"
    echo "  $SYMLINK_LARGE -> $RESOLVED_LARGE"

    if [ ! -e "$RESOLVED_SMALL" ] || [ ! -e "$RESOLVED_LARGE" ]; then
        echo "ERROR: Symlinks don't resolve to existing devices!"
        exit 1
    fi

    # Try to install udev rules for persistence across reboots
    # (serial extraction can fail on some adapters — not fatal)
    SERIAL_5V=$(udevadm info -a -n "$PORT_5V_TMP" 2>/dev/null | grep -m1 'ATTRS{serial}' | sed 's/.*=="\(.*\)"/\1/' || true)
    SERIAL_12V=$(udevadm info -a -n "$PORT_12V_TMP" 2>/dev/null | grep -m1 'ATTRS{serial}' | sed 's/.*=="\(.*\)"/\1/' || true)

    if [ -n "$SERIAL_5V" ] && [ -n "$SERIAL_12V" ]; then
        RULES_CONTENT="# FrankaJoylo: persistent symlinks for Dynamixel controllers
SUBSYSTEM==\"tty\", ATTRS{serial}==\"$SERIAL_5V\", SYMLINK+=\"franka_small\", MODE=\"0666\", ATTR{latency_timer}=\"1\"
SUBSYSTEM==\"tty\", ATTRS{serial}==\"$SERIAL_12V\", SYMLINK+=\"franka_large\", MODE=\"0666\", ATTR{latency_timer}=\"1\""

        echo "$RULES_CONTENT" | sudo tee "$UDEV_RULES_FILE" > /dev/null
        sudo udevadm control --reload-rules
        echo "  udev rules installed at $UDEV_RULES_FILE (persistent across reboots)"
    else
        echo "  NOTE: Could not read USB serial numbers — symlinks won't survive reboot."
        echo "  Re-run setup after reconnecting to fix."
    fi

    PORT_5V="$SYMLINK_SMALL"
    PORT_12V="$SYMLINK_LARGE"

    echo ""
    echo "Port assignment:"
    echo "  5V  controller (XL330): $PORT_5V"
    echo "  12V controller (XL430): $PORT_12V"
    echo ""
fi

# ------------------------------------------------------------------
# Step 2: Calibrate
# ------------------------------------------------------------------
echo "============================================"
echo "  Step 2: Calibration"
echo "============================================"
echo ""

RUN_CALIBRATION=true
if [ -f "$CONFIG_PATH" ]; then
    echo "Existing config found at: $CONFIG_PATH"
    read -rp "Re-run calibration and overwrite? [y/N] " answer
    if [[ ! "$answer" =~ ^[Yy]$ ]]; then
        echo "Skipping calibration — using existing config."
        RUN_CALIBRATION=false
    fi
fi

if [ "$RUN_CALIBRATION" = true ]; then
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
fi

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
