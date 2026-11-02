#!/usr/bin/env bash
set -e
course_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$course_root/scripts/ros_env.sh"
ros2 param set /motor_action_server target_rpm 80
ros2 param set /motor_action_server run_seconds 5
ros2 param set /motor_action_server target_turns 0.0
mkdir -p "$course_root/results"
ros2 param dump /motor_action_server > "$course_root/results/motor.parameters.yaml"
