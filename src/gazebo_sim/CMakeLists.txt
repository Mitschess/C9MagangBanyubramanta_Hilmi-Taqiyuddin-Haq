cmake_minimum_required(VERSION 3.8)
project(box_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(interfaces REQUIRED)

# Include directories
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# Build controller node
add_executable(controller_node src/controller.cpp)
ament_target_dependencies(controller_node
  rclcpp
  sensor_msgs
  interfaces
)

# Build Gazebo plugin
add_library(box_control_plugin SHARED
  src/box_control_plugin.cpp
)

ament_target_dependencies(box_control_plugin
  rclcpp
  gazebo_ros
  interfaces
)

target_link_libraries(box_control_plugin
  ${GAZEBO_LIBRARIES}
  ${rclcpp_LIBRARIES}
)

# Install targets
install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  box_control_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install models
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install worlds
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
