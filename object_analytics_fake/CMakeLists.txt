# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(object_analytics_fake)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(object_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 3.2 REQUIRED)

include_directories(
  ${pcl_conversions_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_IO_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

add_executable(object_analytics_fake src/object_analytics_fake.cpp)
ament_target_dependencies(object_analytics_fake ament_index_cpp rclcpp sensor_msgs object_msgs cv_bridge pcl_conversions)
target_link_libraries(object_analytics_fake
  ${pcl_conversions_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  object_analytics_fake
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  images
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
