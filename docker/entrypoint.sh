#!/bin/bash
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f /home/${USER}/ros2_ws/install/setup.bash ]; then
    source /home/${USER}/ros2_ws/install/setup.bash
fi
exec "$@"
