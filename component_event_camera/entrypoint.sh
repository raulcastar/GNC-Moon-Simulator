#!/usr/bin/env bash
# ==============================================================================
# ROS 2 Docker Entrypoint Script
#
# Initializes the ROS 2 environment, compiles the local workspace, and launches
# the event-camera lifecycle node.
# ==============================================================================

set -Eeuo pipefail

die(){ echo "ERROR: $*" >&2; exit 1; }

[[ -f /opt/ros/jazzy/setup.bash ]] || die "Missing /opt/ros/jazzy/setup.bash"

set +u
source /opt/ros/jazzy/setup.bash
set -u

echo "Building the local ROS 2 workspace..."
colcon build

if [ -f install/setup.bash ]; then
    set +u
    source install/setup.bash
    set -u
else
    die "Build failed or install/setup.bash not found."
fi

echo "Workspace initialized successfully. Launching Event Camera Component..."

# Kill leftover ROS processes (if any)
pkill -9 -f ros2 || true
ros2 daemon stop || true

exec ros2 launch component_event_camera lc_component_event_camera.launch.py
