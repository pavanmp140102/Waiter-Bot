#!/bin/bash
# Source ROS and the workspace overlay (when built), then run the requested command.
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
else
  echo "entrypoint: /ws/install not found, run colcon build in /ws first" >&2
fi
exec "$@"
