#!/bin/bash
#
# Rematrice Quadcopter - PX4 SITL Installation Script
# Copies model files and airframe config into a PX4-Autopilot tree.
#
# Usage:
#   ./install.sh /path/to/PX4-Autopilot
#

set -e

if [ -z "$1" ]; then
    echo "Usage: $0 <path-to-PX4-Autopilot>"
    echo "Example: $0 ~/PX4-Autopilot"
    exit 1
fi

PX4_DIR="$1"

# Sanity check
if [ ! -f "$PX4_DIR/Makefile" ] || [ ! -d "$PX4_DIR/ROMFS" ]; then
    echo "Error: $PX4_DIR doesn't look like a PX4-Autopilot directory."
    exit 1
fi

MODELS_DIR="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
WORLDS_DIR="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds"
AIRFRAMES_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
CMAKE_MODELS="$PX4_DIR/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake"
CMAKE_AIRFRAMES="$AIRFRAMES_DIR/CMakeLists.txt"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# ── 1. Copy model ──────────────────────────────────────────
echo "[1/5] Copying Gazebo model..."
mkdir -p "$MODELS_DIR/rematrice/meshes"
cp "$SCRIPT_DIR/model/meshes/Rematrice.STL"      "$MODELS_DIR/rematrice/meshes/"
cp "$SCRIPT_DIR/model/model.config"               "$MODELS_DIR/rematrice/"
cp "$SCRIPT_DIR/model/rematrice.sdf.jinja"        "$MODELS_DIR/rematrice/"
cp "$SCRIPT_DIR/PARAMETERS.md"                     "$MODELS_DIR/rematrice/"

cp "$SCRIPT_DIR/world/rematrice.world"             "$WORLDS_DIR/"

# ── 2. Copy airframe ──────────────────────────────────────
echo "[2/5] Installing airframe config (ID 6017)..."
cp "$SCRIPT_DIR/airframe/6017_gazebo-classic_rematrice" "$AIRFRAMES_DIR/"

# ── 3. PX4 simulator patch ────────────────────────────────
SIM_SRC="$PX4_DIR/src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp"
if ! grep -q "fifo_saturate" "$SIM_SRC"; then
    echo "[3/5] Patching SimulatorMavlink.cpp..."
    if ! patch -p1 -d "$PX4_DIR" --forward --silent < "$SCRIPT_DIR/patches/simulator_mavlink_fifo_saturate.patch"; then
        echo "  ⚠ Patch failed. Apply patches/simulator_mavlink_fifo_saturate.patch by hand;"
        echo "    without it the simulated accelerometer wraps on touchdown."
    fi
else
    echo "[3/5] SimulatorMavlink.cpp already patched."
fi

# ── 4. Register model in CMake targets ────────────────────
echo "[4/5] Registering model in build system..."
if ! grep -q "rematrice" "$CMAKE_MODELS" 2>/dev/null; then
    sed -i '/typhoon_h480/a\\t\trematrice' "$CMAKE_MODELS" 2>/dev/null || \
    echo "  ⚠ Could not auto-register model target. Add 'rematrice' to the models list in:"
    echo "    $CMAKE_MODELS"
fi

# ── 5. Register airframe in CMake ─────────────────────────
echo "[5/5] Registering airframe in build system..."
if ! grep -q "6017_gazebo-classic_rematrice" "$CMAKE_AIRFRAMES" 2>/dev/null; then
    # Find a good insertion point near the 6000-series airframes
    if grep -q "6016_gazebo-classic" "$CMAKE_AIRFRAMES"; then
        sed -i '/6016_gazebo-classic.*/a\\t6017_gazebo-classic_rematrice' "$CMAKE_AIRFRAMES"
    elif grep -q "6011_gazebo-classic" "$CMAKE_AIRFRAMES"; then
        sed -i '/6011_gazebo-classic_typhoon_h480$/a\\t6017_gazebo-classic_rematrice' "$CMAKE_AIRFRAMES"
    else
        echo "  ⚠ Could not auto-register airframe. Add '6017_gazebo-classic_rematrice' to:"
        echo "    $CMAKE_AIRFRAMES"
    fi
fi

echo ""
echo "Done! You can now run:"
echo "  cd $PX4_DIR"
echo "  make px4_sitl gazebo-classic_rematrice"
echo ""
echo "The default world is the KSQL airport (worlds/rematrice.world)."
echo "To use a different world:"
echo "  make px4_sitl gazebo-classic_rematrice__empty"
echo "  make px4_sitl gazebo-classic_rematrice__warehouse"
