#!/usr/bin/env bash
set -o pipefail
set +u

WORKSPACE="${THESIS_WORKSPACE:-$HOME/Escritorio/TESIS_2_IMPLEMENTATION}"
REPORT_DIR="$WORKSPACE/tools/realsense_d435i_test/reports"
STAMP="$(date +%Y%m%d_%H%M%S)"
REPORT="$REPORT_DIR/perception_standalone_${STAMP}.log"
RUN_RATE_PROBES="${PERCEPTION_RUN_RATE_PROBES:-false}"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/local_setup.bash"
set -u
mkdir -p "$REPORT_DIR"

run()
{
    echo
    echo "\$ $*"
    timeout 8 "$@" || true
}

{
    echo "D435i standalone perception validation"
    echo "timestamp=$(date --iso-8601=seconds)"
    echo "workspace=$WORKSPACE"

    run ros2 node list
    run ros2 topic list -t

    run ros2 topic info /camera/d435i/depth/color/points -v
    run ros2 topic echo --once \
        /camera/d435i/depth/color/points --field header

    run ros2 topic info /thesis/perception/points_filtered -v
    run ros2 topic echo --once \
        /thesis/perception/points_filtered --field header

    run ros2 topic echo --once /thesis/perception/diagnostics
    run ros2 topic echo --once \
        /thesis/perception/extraction_diagnostics
    run ros2 topic info /thesis/perception/obstacle_candidate -v
    run ros2 topic echo --once /thesis/perception/obstacle_candidate

    run ros2 param dump /pointcloud_preprocessor
    run ros2 param dump /obstacle_extractor
    run ros2 param dump /perception_health

    if [[ "$RUN_RATE_PROBES" == "true" ]]; then
        echo
        echo "WARNING: rate probes copy large clouds and can lower throughput"
        run ros2 topic hz /camera/d435i/depth/color/points
        run ros2 topic hz /thesis/perception/points_filtered
    else
        echo
        echo "PointCloud rate probes skipped; using perception_health cloud_hz"
    fi
} 2>&1 | tee "$REPORT"

echo
echo "REPORT=$REPORT"
