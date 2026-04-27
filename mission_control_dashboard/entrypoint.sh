#!/usr/bin/env bash
# ==============================================================================
# ROS 2 Docker Entrypoint Script
# 
# This script initializes the ROS 2 environment, compiles the local workspace,
# and launches the primary lifecycle node. It acts as the container's PID 1 
# initializer before handing off execution to the ROS 2 launch system.
# ==============================================================================

# ------------------------------------------------------------------------------
# 1. Strict Error Handling ("Bash Strict Mode")
# ------------------------------------------------------------------------------
set -Eeuo pipefail

# Utility function to print error messages to standard error (stderr) and exit safely.
die(){ echo "ERROR: $*" >&2; exit 1; }

# Verify that the core ROS 2 "Underlay" exists before attempting to source it.
[[ -f /opt/ros/jazzy/setup.bash ]] || die "Missing /opt/ros/jazzy/setup.bash"

# ------------------------------------------------------------------------------
# 2. Source the ROS 2 Underlay (System Installation)
# ------------------------------------------------------------------------------
set +u
source /opt/ros/jazzy/setup.bash
set -u

echo "Building the local ROS 2 workspace..."

# ------------------------------------------------------------------------------
# 3. Compile the Workspace (The "Overlay")
# ------------------------------------------------------------------------------
colcon build

# ------------------------------------------------------------------------------
# 4. Source the Local Overlay
# ------------------------------------------------------------------------------
if [ -f install/setup.bash ]; then
    set +u
    source install/setup.bash
    set -u
else
    die "Build failed or install/setup.bash not found."
fi

echo "Workspace initialized successfully. Launching Dashboard Component..."

# Kill leftover ROS processes (if any)
pkill -9 -f ros2 || true
ros2 daemon stop || true

# ------------------------------------------------------------------------------
# 5. Execute the Primary ROS 2 Launch Process
# ------------------------------------------------------------------------------
exec ros2 launch component_rover_leo_raph lc_component_rover_leo_raph.launch.py