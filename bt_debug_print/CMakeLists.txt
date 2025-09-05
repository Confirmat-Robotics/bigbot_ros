cmake_minimum_required(VERSION 3.10)
project(bt_debug_print)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(pluginlib REQUIRED)

add_library(bt_debug_print_node SHARED
  src/debug_print_node.cpp
)

target_include_directories(bt_debug_print_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bt_debug_print_node
  rclcpp
  behaviortree_cpp
  pluginlib
)

pluginlib_export_plugin_description_file(behaviortree_cpp plugin_description.xml)

install(TARGETS bt_debug_print_node
  EXPORT export_bt_debug_print
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  behaviortree_cpp
  pluginlib
)
ament_export_targets(export_bt_debug_print)

ament_package()
