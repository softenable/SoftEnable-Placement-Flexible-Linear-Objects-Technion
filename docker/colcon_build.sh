#!/usr/bin/env bash

# Input arguments
: "${ROS_DISTRO:=humble}" 
PKG="${1:-displacement_planner}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Resolve dependency package paths up to the target package
mapfile -t PKG_PATHS < <(colcon list --packages-up-to "${PKG}" --paths-only)

# Install only required dependencies
rosdep update
rosdep install -y \
  --rosdistro "${ROS_DISTRO}" \
  --ignore-src \
  --from-paths "${PKG_PATHS[@]}"

# Configure parallelism
CMAKE_BUILD_PARALLEL_LEVEL="$(nproc)"
export CMAKE_BUILD_PARALLEL_LEVEL

# Build only what’s required
colcon build \
  --packages-up-to "${PKG}" \
  --packages-skip-build-finished \
  --symlink-install \
  --parallel-workers "$(nproc)" \
  --cmake-args \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_UNITY_BUILD=ON \
    -DCMAKE_UNITY_BUILD_BATCH_SIZE=16 \
    -DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=lld \
    -DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=lld
