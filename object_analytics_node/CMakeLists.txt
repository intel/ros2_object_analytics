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
project(object_analytics_node)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "set build type to Release" FORCE)
  message(${CMAKE_BUILD_TYPE})
endif()

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
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(class_loader REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(object_msgs REQUIRED)
find_package(object_analytics_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS
  common
  segmentation
  kdtree
  search
  filters
)
find_package(OpenCV 3.2 REQUIRED)

include_directories(
  include
  ${object_analytics_msgs_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_SEGMENTATION_INCLUDE_DIRS}
)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(object_analytics_common SHARED
  src/const.cpp
  src/util/file_parser.cpp
  src/model/object2d.cpp
  src/model/object3d.cpp
  src/model/object_utils.cpp
)
target_compile_definitions(object_analytics_common
  PRIVATE "OBJECT_ANALYTICS_NODE_BUILDING_DLL"
)
ament_target_dependencies(object_analytics_common
  "sensor_msgs"
  "geometry_msgs"
  "object_msgs"
  "object_analytics_msgs"
  "pcl_conversions"
  "cv_bridge"
  "class_loader"
)
target_link_libraries(object_analytics_common
  ${PCL_COMMON_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(composition src/composition.cpp)
target_link_libraries(composition ament_index_cpp::ament_index_cpp)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_libraries(composition "stdc++fs")
endif()
ament_target_dependencies(composition
  "class_loader"
  "rclcpp"
  "rcutils"
)
target_link_libraries(composition object_analytics_common)

add_executable(tracker_regression src/tracker/tracking_regression.cpp
  src/dataset/tr_dataset.cpp
  src/dataset/trimg_dataset.cpp
  src/dataset/trvid_dataset.cpp)
target_link_libraries(tracker_regression ament_index_cpp::ament_index_cpp)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_libraries(tracker_regression "stdc++fs")
endif()
ament_target_dependencies(tracker_regression
  "object_analytics_msgs"
  "cv_bridge"
  "OpenCV"
  "object_msgs"
  "class_loader"
  "rclcpp"
  "rcutils"
)
target_link_libraries(tracker_regression object_analytics_common tracking_component)

add_library(splitter_component SHARED
  src/splitter/splitter_node.cpp
  src/splitter/splitter.cpp
)
target_compile_definitions(splitter_component
  PRIVATE "OBJECT_ANALYTICS_NODE_BUILDING_DLL"
)
ament_target_dependencies(splitter_component
  "class_loader"
  "rclcpp"
  "sensor_msgs"
  "pcl_conversions"
)
target_link_libraries(splitter_component object_analytics_common)
rclcpp_register_node_plugins(splitter_component "object_analytics_node::splitter::SplitterNode")
set(node_plugins
  "${node_plugins}object_analytics_node::splitter::SplitterNode;$<TARGET_FILE:splitter_component>\n")

add_library(moving_component SHARED
  src/movement/moving_object_node.cpp
  src/movement/moving_objects.cpp
  src/movement/object_frame.cpp
  src/movement/param.cpp)

target_compile_definitions(moving_component
  PRIVATE "OBJECT_ANALYTICS_NODE_BUILDING_DLL"
)
ament_target_dependencies(moving_component
  "rclcpp"
  "object_analytics_msgs"
  "rmw_implementation"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "object_msgs"
  "message_filters"
  "ament_index_cpp"
  "class_loader"
)
target_link_libraries(moving_component object_analytics_common)
rclcpp_register_node_plugins(moving_component "object_analytics_node::movement::MovementNode")
set(node_plugins
  "${node_plugins}object_analytics_node::movement::MovementNode;$<TARGET_FILE:moving_component>\n")

add_library(tracking_component SHARED
  src/tracker/tracking_node.cpp
  src/tracker/tracking.cpp
  src/tracker/tracking_manager.cpp
)
target_compile_definitions(tracking_component
  PRIVATE "OBJECT_ANALYTICS_NODE_BUILDING_DLL"
)
ament_target_dependencies(tracking_component
  "class_loader"
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "OpenCV"
  "object_msgs"
  "object_analytics_msgs"
)
target_link_libraries(tracking_component object_analytics_common)
rclcpp_register_node_plugins(tracking_component "object_analytics_node::tracker::TrackingNode")
set(node_plugins
  "${node_plugins}object_analytics_node::tracker::TrackingNode;$<TARGET_FILE:tracking_component>\n")

add_library(segmenter_component SHARED
  src/segmenter/segmenter_node.cpp
  src/segmenter/segmenter.cpp
  src/segmenter/algorithm_provider_impl.cpp
  src/segmenter/organized_multi_plane_segmenter.cpp
)
target_compile_definitions(segmenter_component
  PRIVATE "OBJECT_ANALYTICS_NODE_BUILDING_DLL"
)
ament_target_dependencies(segmenter_component
  "class_loader"
  "rclcpp"
  "sensor_msgs"
  "object_msgs"
  "object_analytics_msgs"
  "message_filters"
)
target_link_libraries(segmenter_component object_analytics_common ${PCL_SEGMENTATION_LIBRARIES})
rclcpp_register_node_plugins(segmenter_component "object_analytics_node::segmenter::SegmenterNode")
set(node_plugins
  "${node_plugins}object_analytics_node::segmenter::SegmenterNode;$<TARGET_FILE:segmenter_component>\n")


install(TARGETS
  splitter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  composition
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  tracker_regression
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  object_analytics_common
  segmenter_component
  tracking_component
  moving_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

# Flags
if(UNIX OR APPLE)
  # Linker flags.
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    # GCC specific flags. ICC is compatible with them.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z noexecstack -z relro -z now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack -z relro -z now")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # In Clang, -z flags are not compatible, they need to be passed to linker via -Wl.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} \
      -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} \
      -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
  endif()

  # Compiler flags.
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    # GCC specific flags.
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9 OR
        CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.9)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector-strong")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
    endif()
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # Clang is compatbile with some of the flags.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    # Same as above, with exception that ICC compilation crashes with -fPIE option, even
    # though it uses -pie linker option that require -fPIE during compilation. Checksec
    # shows that it generates correct PIE anyway if only -pie is provided.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector")
  endif()

  # Generic flags.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fno-operator-names -Wformat -Wformat-security \
    -Wall -fopenmp")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  # Dot not forward c++11 flag to GPU beucause it is not supported
  set( CUDA_PROPAGATE_HOST_FLAGS OFF )
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D_FORTIFY_SOURCE=2")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(tests)
endif()

ament_package()
