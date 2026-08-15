#!/usr/bin/env bash
# Build the odometry_filter_py pybind11 bindings inside the pipeline
# container and stage the compiled module under _native/ so the SLAM
# benchmark runs the REAL C++ EKF (not the Python fallback).
#
# Run this once after changing pipeline/odometry_filter, or whenever
# _native/ is missing. The benchmark (common.py) auto-mounts _native/
# into its docker run and prepends it to PYTHONPATH.
#
# Usage:  tools/sim_benchmark/build_native_filter.sh
set -euo pipefail

CONTAINER="${IFSSIM_DV_CONTAINER:-ifssim-dv_pipeline_stack-1}"
WS=/dv_pipeline_stack_ws
PKG=odometry_filter
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
NATIVE_DIR="$HERE/_native"

echo ">> syncing $PKG sources into $CONTAINER"
for f in CMakeLists.txt package.xml \
         include/odometry_filter/odometry_filter.hpp \
         src/odometry_filter.cpp src/python_bindings.cpp; do
  docker cp "$REPO_ROOT/pipeline/$PKG/$f" \
    "$CONTAINER:$WS/src/$PKG/$f"
done

echo ">> building $PKG (with pybind11 bindings)"
docker exec "$CONTAINER" bash -lc \
  "source /opt/ros/humble/setup.bash && cd $WS && \
   colcon build --packages-select $PKG --cmake-args -DCMAKE_BUILD_TYPE=Release"

echo ">> staging compiled module into _native/"
mkdir -p "$NATIVE_DIR"
SO_IN="$(docker exec "$CONTAINER" bash -lc \
  "find $WS/install/$PKG -name 'odometry_filter_py*.so' | head -1" | tr -d '\r')"
SO_NAME="$(basename "$SO_IN")"
# Dereference the colcon symlink-install symlink to copy the real file.
docker exec "$CONTAINER" bash -lc "cp -L '$SO_IN' /tmp/$SO_NAME"
docker cp "$CONTAINER:/tmp/$SO_NAME" "$NATIVE_DIR/$SO_NAME"

echo ">> done: $NATIVE_DIR/$SO_NAME"
