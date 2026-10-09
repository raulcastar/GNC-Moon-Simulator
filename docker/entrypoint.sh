#!/usr/bin/env bash
# ==============================================================================
# Shared entrypoint of every GNC component container (baked into the base image
# as /entrypoint.sh).
#
# Sources ROS 2, builds the workspace mounted at /opt/ros/jazzy/src (only the
# first time, unless FORCE_BUILD=1), and hands PID 1 to the component's launch
# file. Each component image says what to launch:
#
#   GNC_PACKAGE   ROS 2 package to launch (required)
#   GNC_LAUNCH    launch file in it (default: lc_<GNC_PACKAGE>.launch.py)
# ==============================================================================

set -Eeuo pipefail

die() { echo "ERROR: $*" >&2; exit 1; }

[[ -n "${GNC_PACKAGE:-}" ]] || die "GNC_PACKAGE is not set: the component image must define it."
GNC_LAUNCH="${GNC_LAUNCH:-lc_${GNC_PACKAGE}.launch.py}"
[[ -f /opt/ros/jazzy/setup.bash ]] || die "Missing /opt/ros/jazzy/setup.bash"

# ROS setup scripts read unset variables, so strict mode is relaxed while sourcing.
set +u
source /opt/ros/jazzy/setup.bash
set -u

cd /opt/ros/jazzy/src
if [[ -f install/setup.bash && "${FORCE_BUILD:-0}" != "1" ]]; then
    echo "Workspace already built; skipping colcon build (set FORCE_BUILD=1 to rebuild)."
else
    echo "Building the ROS 2 workspace..."
    colcon build
fi
[[ -f install/setup.bash ]] || die "Build failed: install/setup.bash not found."

set +u
source install/setup.bash
set -u

# Leftovers of a previous run in this container (restart: unless-stopped).
pkill -9 -f ros2 || true
ros2 daemon stop || true

echo "Launching ${GNC_PACKAGE} (${GNC_LAUNCH})..."
# exec: the launch process becomes PID 1, so `docker stop` reaches it directly.
exec ros2 launch "${GNC_PACKAGE}" "${GNC_LAUNCH}"
