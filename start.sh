#!/bin/sh

unset http_proxy
unset https_proxy
unset all_proxy

. /opt/ros/${ROS_DISTRO}/setup.sh
. ./install/setup.sh
colcon build --mixin release
ros2 launch action_example fib_server.launch.py