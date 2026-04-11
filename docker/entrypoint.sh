#!/bin/bash
# ════════════════════════════════════════════════════════════════════════════
# Docker entrypoint — sources ROS 2 and workspace, then runs the command.
# ════════════════════════════════════════════════════════════════════════════
set -e

# Source ROS 2 base environment
source /opt/ros/humble/setup.bash

# Source the built workspace
source /ros2_ws/install/setup.bash

# Hand off to the CMD (or any command passed to `docker run`)
exec "$@"
