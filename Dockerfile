# ════════════════════════════════════════════════════════════════════════════
# Sensor Fusion System — Multi-stage Docker Build
# ════════════════════════════════════════════════════════════════════════════
#
# USAGE:
#   Build:
#     docker build -t sensor_fusion_system .
#
#   Run full system (all 4 nodes):
#     docker run --rm -it sensor_fusion_system
#
#   Run a single node:
#     docker run --rm -it sensor_fusion_system \
#       ros2 run sensor_fusion_system fusion_node
#
#   Run tests:
#     docker run --rm -it sensor_fusion_system colcon test --test-result-base results
#
#   View live pose (requires host network + ROS 2 on host):
#     docker run --rm -it --network host sensor_fusion_system
#     ros2 topic echo /fusion/pose      # on host
#     ros2 topic echo /tf               # verify TF2 broadcast
#
# WHY MULTI-STAGE?
#   Stage 1 (builder) has the full compiler toolchain (~2 GB).
#   Stage 2 (runtime) copies only the built binaries — final image is ~600 MB.
# ════════════════════════════════════════════════════════════════════════════

# ── Stage 1: Builder ─────────────────────────────────────────────────────────
FROM osrf/ros:humble-desktop AS builder

# Avoid interactive prompts during apt
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /ros2_ws

# Install build-time dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-ament-cmake-gtest \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Copy source into workspace
COPY . src/sensor_fusion_system/

# Build in Release mode
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    colcon build \
      --packages-select sensor_fusion_system \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --event-handlers console_cohesion+"

# ── Stage 2: Runtime ──────────────────────────────────────────────────────────
FROM osrf/ros:humble-ros-base AS runtime

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /ros2_ws

# Install only runtime libraries (no compiler, no dev headers)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Copy built install tree from builder stage
COPY --from=builder /ros2_ws/install ./install

# Copy entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# Default: launch all 4 nodes with the shared config
CMD ["ros2", "launch", "sensor_fusion_system", "sensor_fusion_system.launch.py"]
