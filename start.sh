#!/bin/sh

unset http_proxy
unset https_proxy
unset all_proxy

. /opt/ros/humble/setup.sh
. ./install/setup.sh
colcon build --mixin debug
ros2 launch action_example fib_server.launch.py