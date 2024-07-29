#!/bin/bash

source "/opt/ros/$ROS_DISTRO/setup.bash" --
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

ulimit -n 524388
ulimit -Hn 524388

exec "$@"
