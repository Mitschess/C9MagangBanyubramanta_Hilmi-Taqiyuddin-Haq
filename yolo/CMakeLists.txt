cmake_minimum_required(VERSION 3.8)
project(yolo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vision_msgs REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# ============================
# Detection Node
# ============================
add_executable(detection_node src/detection.cpp)
ament_target_dependencies(detection_node
  rclcpp
  sensor_msgs
  cv_bridge
  vision_msgs
)
target_link_libraries(detection_node ${OpenCV_LIBS})

# ============================
# Mask Node
# ============================
add_executable(mask_node src/mask.cpp)
ament_target_dependencies(mask_node
  rclcpp
  sensor_msgs
  cv_bridge
  vision_msgs
)
target_link_libraries(mask_node ${OpenCV_LIBS})

# ============================
# Raw Node
# ============================
add_executable(raw_node src/raw.cpp)
ament_target_dependencies(raw_node
  rclcpp
  sensor_msgs
  cv_bridge
  vision_msgs
)
target_link_libraries(raw_node ${OpenCV_LIBS})

install(TARGETS
  detection_node
  mask_node
  raw_node
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
