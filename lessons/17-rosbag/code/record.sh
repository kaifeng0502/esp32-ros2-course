#!/usr/bin/env bash
set -e
course_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$course_root/scripts/ros_env.sh"
destination="${1:-$course_root/results/bags/$(date +%Y%m%d-%H%M%S)}"
mkdir -p "$(dirname "$destination")"
exec ros2 bag record -o "$destination" /motor/requested_rpm /motor/setpoint /motor/rpm /motor/pwm /motor/rpm_valid /motor/state /parameter_events
