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
project(oa_viewer)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "set build type to Release" FORCE)
  message(${CMAKE_BUILD_TYPE})
endif()

####################################
## to use C++11
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
####################################

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
	# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(Pangolin REQUIRED)
find_package(OpenCV 3.2 REQUIRED)

include_directories(
  include
  utils
	device
	render_object
  view
  control
  data 
  data/dataset
  model
  model/stat
  model/sample
	$(CMAKE_CURRENT_SOURCE_DIR)
)

add_executable(oa_viewer 
	main.cpp
  view/view.cpp
  control/control.cpp
  control/control_ds.cpp
	device/stream_cap.cpp
	device/stream_device.cpp
	device/stream_vid.cpp
	device/stream_ds.cpp
	render_object/render_object.cpp
	render_object/render_image.cpp
	render_object/render_lines.cpp
	render_object/render_rect.cpp
	render_object/render_ellipse.cpp
  data/frame.cpp
  data/frame_obj.cpp
  data/dataset/tr_dataset.cpp
  data/dataset/trimg_dataset.cpp
  data/dataset/trvid_dataset.cpp
  data/dataset/trimg_MTdataset.cpp
  model/math_model.cpp
  model/math_sample.cpp
  model/stat/stat_model.cpp
  model/stat/gaussian_model.cpp
  model/sample/sample_model.cpp
  model/sample/uniform_sample.cpp
  model/sample/rw_sample.cpp
	)

target_link_libraries(oa_viewer
	${PCL_COMMON_LIBRARIES}
	${Pangolin_LIBRARIES}
	${OpenCV_LIBRARIES}
)
