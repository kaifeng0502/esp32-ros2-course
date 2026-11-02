#!/usr/bin/env bash
set -e
course_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
course_ws="${1:-${COURSE_ROS_WS:-$HOME/esp32_ros2_ws}}"
source /opt/ros/jazzy/setup.bash
mkdir -p "$course_ws/src"
course_package="$course_ws/src/motor_action_interfaces"
if [[ -e "$course_package" || -L "$course_package" ]]; then
  if [[ ! -L "$course_package" || "$(readlink "$course_package")" != "$course_root/ros2/motor_action_interfaces" ]]; then
    echo "已有不同来源的包：$course_package；请指定一个新的工作区。" >&2
    exit 1
  fi
else
  ln -s "$course_root/ros2/motor_action_interfaces" "$course_package"
fi
cd "$course_ws"
colcon build --symlink-install --packages-select motor_action_interfaces \
  --cmake-force-configure --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
echo "构建完成。新终端先执行：source $course_ws/install/setup.bash"
