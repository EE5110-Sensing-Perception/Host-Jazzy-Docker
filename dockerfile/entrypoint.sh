#!/bin/bash

# Set up ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash

# Configure NVIDIA GPU
if [ -n "$NVIDIA_VISIBLE_DEVICES" ]; then
    export LD_LIBRARY_PATH=/usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}
fi

# Set up joystick permissions
if [ -e /dev/input/js0 ]; then
    chmod a+rw /dev/input/js0
fi

# Forward X11 for RViz
if [ -n "$DISPLAY" ]; then
    xhost +local:root
fi

exec "$@"
