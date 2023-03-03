#!/bin/sh

# check if ROS_DISTRO is set, if not, set it to 'humble'
if [ -z "$ROS_DISTRO" ]; then
	export ROS_DISTRO=humble
fi

. /opt/ros/${ROS_DISTRO}/setup.sh

# get build type from command line
BUILD_TYPE=$1
# default to release
if [ -z "$BUILD_TYPE" ]; then
	BUILD_TYPE=release
fi

#  set http_proxy
export http_proxy=http://192.168.12.43:8888
export https_proxy=http://192.168.12.43:8888
export all_proxy=socks5://192.168.12.43:8889

# test if the second argument is set, if set, use it as the package name, else use all packages
if [ -z "$2" ]; then
	colcon build --mixin $BUILD_TYPE compile-commands --cmake-args -DTEST_485=ON
else
	colcon build --mixin $BUILD_TYPE compile-commands ccache --packages-select $2
fi

# colcon build --mixin $BUILD_TYPE compile-commands ccache #--packages-select $PACKAGE
