#!/bin/bash
# Source this file to set up the full multi-robot-fleet-ros2 environment.
# Usage: source setup.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. ROS 2 Humble base
source /opt/ros/humble/setup.bash

# 2. rmf_adapter Python bindings live in a non-standard path in Humble apt
#    They are only added to PYTHONPATH when sourcing ROS — make it explicit.
export PYTHONPATH="/opt/ros/humble/lib/python/site-packages:${PYTHONPATH}"

# 3. Workspace install (skip missing packages gracefully)
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    # Source each package individually to skip missing ones
    for pkg_setup in "$SCRIPT_DIR"/install/*/share/*/local_setup.bash; do
        [ -f "$pkg_setup" ] && source "$pkg_setup" 2>/dev/null
    done
    # Source workspace-level setup for proper overlay
    source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null || true
fi

echo "[multi-robot-fleet-ros2] Environment ready."
echo "  ROS_DISTRO : $ROS_DISTRO"
echo "  Workspace  : $SCRIPT_DIR"
