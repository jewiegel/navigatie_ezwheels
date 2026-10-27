cmake_minimum_required(VERSION 3.8)
project(my_bot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Turn.msg"
  "msg/Waypoint.msg"
  "srv/DeleteRoute.srv"
  "srv/GetRoute.srv"
  "srv/ListRoutes.srv"
  "srv/SaveRoute.srv"
  "srv/SetRoute.srv"
  "srv/StartRoute.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
