#!/usr/bin/env bash
set -e
course_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/jazzy/setup.bash
cat /etc/os-release
uname -m
printenv ROS_DISTRO
printenv ROS_VERSION
ros2 --help
lsusb
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true
id -nG
