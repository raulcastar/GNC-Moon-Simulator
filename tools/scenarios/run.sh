#!/usr/bin/env bash
# Scenario checks against a running simulation (Unreal in Play, stack up).
# Each script is copied into the container that has what it needs, run there,
# and its output copied to tools/scenarios/results/.
#
#   tools/scenarios/run.sh traverse <goal_x> <goal_y> [seconds]   drive to a goal, log it
#   tools/scenarios/run.sh overlay [seconds] [period]             lidar overlay vs the local DEM
#   tools/scenarios/run.sh lidar_gt [seconds]                     lidar depth vs Unreal's ground truth

set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE/../.."                                 # the repository (docker-compose.yml)
RESULTS="$HERE/results"
mkdir -p "$RESULTS"

run_in() {                                       # run_in <service> <script> [args...]
    local service=$1 script=$2; shift 2
    docker compose cp "$HERE/$script" "$service:/tmp/$script" >/dev/null
    docker compose exec -T "$service" bash -c \
        "source /opt/ros/jazzy/setup.bash && source /opt/ros/jazzy/src/install/setup.bash && python3 /tmp/$script $*"
}

case "${1:-}" in
    traverse)
        [[ $# -ge 3 ]] || { echo "usage: $0 traverse <goal_x> <goal_y> [seconds]"; exit 2; }
        run_in component_path_planner traverse.py "$2" "$3" "${4:-300}"
        docker compose cp component_path_planner:/tmp/traverse_log.json "$RESULTS/traverse_log.json" >/dev/null
        echo "log: $RESULTS/traverse_log.json" ;;
    overlay)
        docker compose exec -T mission_control_dashboard rm -rf /tmp/overlay
        run_in mission_control_dashboard overlay_check.py "${2:-150}" "${3:-5}"
        rm -rf "$RESULTS/overlay"
        docker compose cp mission_control_dashboard:/tmp/overlay "$RESULTS/overlay" >/dev/null
        echo "snapshots and stats.json: $RESULTS/overlay" ;;
    lidar_gt)
        run_in component_stereo_depth lidar_vs_gt.py "${2:-10}" ;;
    *)
        sed -n '2,9p' "$0"; exit 2 ;;
esac
