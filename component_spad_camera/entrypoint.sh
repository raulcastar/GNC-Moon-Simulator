#!/usr/bin/env bash
# ==============================================================================
# ROS 2 Docker Entrypoint Script
#
# Initializes the ROS 2 environment, compiles the local workspace, and launches
# the SPAD-camera lifecycle node.
# ==============================================================================

set -Eeuo pipefail

die(){ echo "ERROR: $*" >&2; exit 1; }

[[ -f /opt/ros/jazzy/setup.bash ]] || die "Missing /opt/ros/jazzy/setup.bash"

set +u
source /opt/ros/jazzy/setup.bash
set -u

echo "Building the local ROS 2 workspace..."
if [ -f install/setup.bash ] && [ "${FORCE_BUILD:-0}" != "1" ]; then
    echo "Workspace already built; skipping colcon build (set FORCE_BUILD=1 to force a rebuild)."
else
    colcon build
fi

if [ -f install/setup.bash ]; then
    set +u
    source install/setup.bash
    set -u
else
    die "Build failed or install/setup.bash not found."
fi

echo "Workspace initialized successfully. Launching SPAD Camera Component..."

# Kill leftover ROS processes (if any)
pkill -9 -f ros2 || true
ros2 daemon stop || true

exec ros2 launch component_spad_camera lc_component_spad_camera.launch.py
