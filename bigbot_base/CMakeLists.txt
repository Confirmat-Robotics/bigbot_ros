cmake_minimum_required(VERSION 3.8)
project(bigbot_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bigbot_interfaces REQUIRED)

include_directories(include)

add_library(roboclaw SHARED
            src/roboclaw.c)
ament_export_libraries(roboclaw)

add_executable(drivernode src/DriverNode.cpp)
ament_target_dependencies(drivernode 
  rclcpp 
  geometry_msgs
  std_msgs
  bigbot_interfaces)
target_link_libraries(drivernode roboclaw)

install(TARGETS
  roboclaw
  DESTINATION lib
)

install(TARGETS
  drivernode
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
