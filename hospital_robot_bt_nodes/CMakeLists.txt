cmake_minimum_required(VERSION 3.8)
project(hospital_robot_bt_nodes)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Pure path/costmap logic: message types only, no rclcpp.
add_library(hospital_robot_path_block_check SHARED src/path_block_check.cpp)
target_include_directories(hospital_robot_path_block_check PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(hospital_robot_path_block_check nav2_msgs nav_msgs)

# BT plugins: one shared library per node (Nav2 convention), loaded via plugin_lib_names.
add_library(hospital_robot_publish_blocked_bt_node SHARED src/publish_blocked.cpp)
target_include_directories(hospital_robot_publish_blocked_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(hospital_robot_publish_blocked_bt_node
  behaviortree_cpp_v3 rclcpp std_msgs)
target_compile_definitions(hospital_robot_publish_blocked_bt_node PRIVATE BT_PLUGIN_EXPORT)

add_library(hospital_robot_is_path_blocked_bt_node SHARED src/is_path_blocked.cpp)
target_include_directories(hospital_robot_is_path_blocked_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(hospital_robot_is_path_blocked_bt_node hospital_robot_path_block_check)
ament_target_dependencies(hospital_robot_is_path_blocked_bt_node
  behaviortree_cpp_v3 nav2_msgs nav_msgs rclcpp)
target_compile_definitions(hospital_robot_is_path_blocked_bt_node PRIVATE BT_PLUGIN_EXPORT)

foreach(plugin get_next_waypoint pop_waypoint)
  add_library(hospital_robot_${plugin}_bt_node SHARED src/${plugin}.cpp)
  target_include_directories(hospital_robot_${plugin}_bt_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  ament_target_dependencies(hospital_robot_${plugin}_bt_node behaviortree_cpp_v3 geometry_msgs)
  target_compile_definitions(hospital_robot_${plugin}_bt_node PRIVATE BT_PLUGIN_EXPORT)
endforeach()

set(bt_plugin_libs
  hospital_robot_publish_blocked_bt_node
  hospital_robot_is_path_blocked_bt_node
  hospital_robot_get_next_waypoint_bt_node
  hospital_robot_pop_waypoint_bt_node)

install(TARGETS hospital_robot_path_block_check ${bt_plugin_libs}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_path_block_check test/test_path_block_check.cpp)
  target_link_libraries(test_path_block_check hospital_robot_path_block_check)

  ament_add_gtest(test_publish_blocked test/test_publish_blocked.cpp)
  target_link_libraries(test_publish_blocked hospital_robot_publish_blocked_bt_node)

  ament_add_gtest(test_is_path_blocked test/test_is_path_blocked.cpp)
  target_link_libraries(test_is_path_blocked hospital_robot_is_path_blocked_bt_node)

  ament_add_gtest(test_waypoint_nodes test/test_waypoint_nodes.cpp)
  target_link_libraries(test_waypoint_nodes
    hospital_robot_get_next_waypoint_bt_node hospital_robot_pop_waypoint_bt_node)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(behaviortree_cpp_v3 geometry_msgs nav2_msgs nav_msgs rclcpp std_msgs)
ament_package()
