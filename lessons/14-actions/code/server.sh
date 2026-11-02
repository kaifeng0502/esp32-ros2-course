#!/usr/bin/env bash
set -e
course_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$course_root/scripts/ros_env.sh"
exec ros2 run motor_action_interfaces motor_action_server.py "$@"
