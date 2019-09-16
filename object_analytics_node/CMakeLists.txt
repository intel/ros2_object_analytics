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

set(CRYSTAL_BLOOM_RELEASE FALSE)
# TODO(yechun1): always build tracking when opencv3.3 is available for debian package.
# Tracking feature depends on opencv3.3 libraries, the ubuntu 18.04 only support opencv3.2.
# to bloom release object package, we need to fix dependence issue, as there is no opencv3.3 debian packages,
# temporary removed tracking feature for bloom release object analytics as debian package.
if(NOT ${CRYSTAL_BLOOM_RELEASE})
    set(BUILD_TRACKING TRUE)
else()
    set(BUILD_TRACKING FALSE)
endif()

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
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-unused-function")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp_components REQUIRED)
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
if(${BUILD_TRACKING})
  target_link_libraries(object_analytics_common
    ${PCL_COMMON_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )
else()
  target_link_libraries(object_analytics_common
    ${PCL_COMMON_LIBRARIES}
  )
endif()

add_executable(object_analytics_node src/composition.cpp)
ament_target_dependencies(object_analytics_node
  "class_loader"
  "rclcpp"
  "rclcpp_components"
  "rcutils"
)
target_link_libraries(object_analytics_node object_analytics_common)

if(${BUILD_TRACKING})
  find_package(OpenCV 3.2 REQUIRED)
  add_executable(tracker_regression src/tracker/tracking_regression.cpp
    src/dataset/tr_dataset.cpp
    src/dataset/trimg_dataset.cpp
    src/dataset/trvid_dataset.cpp)
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
    "rclcpp_components"
    "sensor_msgs"
    "cv_bridge"
    "OpenCV"
    "object_msgs"
    "object_analytics_msgs"
  )
  target_link_libraries(tracking_component object_analytics_common)
  rclcpp_components_register_nodes(tracking_component "object_analytics_node::tracker::TrackingNode")
  set(node_plugins
    "${node_plugins}object_analytics_node::tracker::TrackingNode;$<TARGET_FILE:tracking_component>\n")
endif()

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
  "rclcpp_components"
  "sensor_msgs"
  "object_msgs"
  "object_analytics_msgs"
  "message_filters"
)
target_link_libraries(segmenter_component object_analytics_common ${PCL_SEGMENTATION_LIBRARIES} lz4)
rclcpp_components_register_nodes(segmenter_component "object_analytics_node::segmenter::SegmenterNode")
set(node_plugins
  "${node_plugins}object_analytics_node::segmenter::SegmenterNode;$<TARGET_FILE:segmenter_component>\n")

install(TARGETS
  object_analytics_node
  DESTINATION lib/${PROJECT_NAME}
)


if(${BUILD_TRACKING})
  install(TARGETS
    object_analytics_common
    segmenter_component
    tracking_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
  )

  install(TARGETS
    tracker_regression
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  install(TARGETS
    object_analytics_common
    segmenter_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
  )
endif()

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

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

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
