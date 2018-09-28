cmake_minimum_required(VERSION 3.5)
project(object_analytics_rviz)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(object_msgs REQUIRED)
find_package(object_analytics_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(marker_publisher src/marker_publisher.cpp)
ament_target_dependencies(marker_publisher
  object_analytics_msgs
  object_msgs
  sensor_msgs
  rclcpp
  std_msgs
  message_filters
  visualization_msgs
  geometry_msgs
)

add_executable(image_publisher src/image_publisher.cpp)
ament_target_dependencies(image_publisher
  object_analytics_msgs
  object_msgs
  sensor_msgs
  rclcpp
  std_msgs
  message_filters
  visualization_msgs
  geometry_msgs
  cv_bridge
)

install(TARGETS
  marker_publisher
  image_publisher
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
