#!/usr/bin/env bash
# Runs each package's unit tests (<package>/test) inside its own container, on
# the mounted source: the containers have ROS, the package's dependencies and
# pytest. Start the stack first (docker compose up -d).
#
#   ./run_tests.sh                     all packages
#   ./run_tests.sh component_lidar     one package (extra args go to pytest)
#
# The scenario checks that need a running simulation are in tools/scenarios/.

set -uo pipefail
cd "$(dirname "$0")"

# package (= its compose service)
PACKAGES=(component_path_planner component_stereo_depth component_lidar component_rover_leo_raph component_imu component_robot_description mission_control_dashboard)
if [[ $# -gt 0 ]]; then
    PACKAGES=("$1"); shift
fi

failed=()
for package in "${PACKAGES[@]}"; do
    echo "=== $package"
    if ! docker compose exec -T "$package" bash -lc \
        "source /opt/ros/jazzy/setup.bash && source /opt/ros/jazzy/src/install/setup.bash && cd /opt/ros/jazzy/src/$package && python3 -m pytest -q -p no:cacheprovider test $*"; then
        failed+=("$package")
    fi
done

if [[ ${#failed[@]} -gt 0 ]]; then
    echo "FAILED: ${failed[*]}"
    exit 1
fi
echo "All tests passed."
