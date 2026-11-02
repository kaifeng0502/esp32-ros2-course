#!/usr/bin/env bash
set -e
course_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$course_root/scripts/ros_env.sh"
if [[ $# -ne 1 ]]; then echo 'Usage: bash replay.sh path/to/bag'; exit 2; fi
export ROS_DOMAIN_ID=77
echo 'Offline replay on ROS_DOMAIN_ID=77; use the same domain in a separate plot terminal.'
exec ros2 bag play "$1" --clock
