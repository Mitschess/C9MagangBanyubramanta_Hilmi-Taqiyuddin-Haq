cmake_minimum_required(VERSION 3.8)
project(opencvros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(openvino_node src/openvino.cpp)

ament_target_dependencies(openvino_node
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge
  image_transport
  ament_index_cpp
)

install(TARGETS openvino_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
