# Source from a lesson shell script. Avoid set -u: ROS setup scripts inspect unset variables.
source /opt/ros/jazzy/setup.bash
course_ws="${COURSE_ROS_WS:-$HOME/esp32_ros2_ws}"
if [[ ! -f "$course_ws/install/setup.bash" ]]; then
  echo "找不到 $course_ws/install/setup.bash；请先运行 bash scripts/ros_build.sh。" >&2
  return 1
fi
source "$course_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
