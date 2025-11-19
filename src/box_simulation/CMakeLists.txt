cmake_minimum_required(VERSION 3.8)
project(box_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(interfaces REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# ----- PLUGIN -----
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
)

# ----- INSTALL PLUGIN -----
install(
  TARGETS box_control_plugin
  LIBRARY DESTINATION lib
)

# ----- INSTALL OTHER FILES -----
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

ament_package()
