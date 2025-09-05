cmake_minimum_required(VERSION 3.10)
project(gps_fix_simulator)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  # ROS2 and plugin includes
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}

  /opt/ros/jazzy/opt/gz_common_vendor/include/gz/common5
  /opt/ros/jazzy/opt/gz_msgs_vendor/include/gz/msgs10
  /opt/ros/jazzy/opt/gz_math_vendor/include/gz/math7
  /opt/ros/jazzy/opt/gz_plugin_vendor/include/gz/plugin2
  /opt/ros/jazzy/opt/gz_sim_vendor/include/gz/sim8
  /opt/ros/jazzy/opt/gz_gui_vendor/include/gz/gui8
  /opt/ros/jazzy/opt/gz_fuel_tools_vendor/include/gz/fuel_tools9
  /opt/ros/jazzy/opt/gz_physics_vendor/include/gz/physics7
  /opt/ros/jazzy/opt/gz_rendering_vendor/include/gz/rendering8
  /opt/ros/jazzy/opt/gz_sensors_vendor/include/gz/sensors8
  /opt/ros/jazzy/opt/gz_transport_vendor/include/gz/transport13
  /opt/ros/jazzy/opt/gz_utils_vendor/include/gz/utils2
  /opt/ros/jazzy/opt/sdformat_vendor/include/gz/sdformat14
)

add_library(${PROJECT_NAME} SHARED src/gps_fix_simulator.cpp)

target_link_libraries(${PROJECT_NAME}
  gz-plugin2::gz-plugin2
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()

