cmake_minimum_required(VERSION 3.8)
project(motor_action_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "action/MoveWheel.action" "msg/EncoderState.msg"
  DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
install(PROGRAMS scripts/motor_action_server.py scripts/motor_action_client.py scripts/motor_plot.py scripts/wheel_joint_state.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/motor_action_core.py scripts/wheel_joint_core.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch urdf rviz DESTINATION share/${PROJECT_NAME})
ament_package()
