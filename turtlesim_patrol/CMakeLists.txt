cmake_minimum_required(VERSION 3.8)
project(turtlesim_patrol)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteCircle.action"
  DEPENDENCIES builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
install(PROGRAMS
  turtlesim_patrol/circle_patrol_server.py
  turtlesim_patrol/circle_patrol_client.py
  DESTINATION lib/${PROJECT_NAME}
)
