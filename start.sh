#!/bin/sh

unset http_proxy
unset https_proxy
unset all_proxy

# check if ROS_DISTRO is set, if not, set it to 'humble'
if [ -z "$ROS_DISTRO" ]; then
	export ROS_DISTRO=humble
fi

. /opt/ros/${ROS_DISTRO}/setup.sh
. ./install/setup.sh

colcon build --mixin release
ros2 launch action_example fib_server.launch.py
