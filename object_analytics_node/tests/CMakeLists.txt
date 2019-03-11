find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(object_msgs REQUIRED)
find_package(object_analytics_msgs REQUIRED)

ament_lint_auto_find_test_dependencies()

find_package(PCL 1.7 REQUIRED COMPONENTS
  io
)
if(${BUILD_TRACKING})
  set(UNITEST_LIBRARIES
    pthread
    ${GTEST_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    segmenter_component
    tracking_component
  )
else()
  set(UNITEST_LIBRARIES
    pthread
    ${GTEST_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    segmenter_component
  )
endif()

get_filename_component(RESOURCE_DIR "resource" ABSOLUTE)
configure_file(unittest_util.hpp.in unittest_util.hpp)
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${object_msgs_INCLUDE_DIRS})

ament_add_gtest(unittest_object2d unittest_object2d.cpp unittest_util.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
if(TARGET unittest_object2d)
  target_link_libraries(unittest_object2d ${UNITEST_LIBRARIES})
endif()

ament_add_gtest(unittest_object3d unittest_object3d.cpp unittest_util.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
if(TARGET unittest_object3d)
  target_link_libraries(unittest_object3d ${UNITEST_LIBRARIES})
endif()

ament_add_gtest(unittest_objectutils unittest_objectutils.cpp unittest_util.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
if(TARGET unittest_objectutils)
  target_link_libraries(unittest_objectutils ${UNITEST_LIBRARIES})
endif()

ament_add_gtest(unittest_segmenter unittest_segmenter.cpp unittest_util.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
if(TARGET unittest_segmenter)
  target_link_libraries(unittest_segmenter ${UNITEST_LIBRARIES})
endif()

if(${BUILD_TRACKING})
  ament_add_gtest(unittest_tracking unittest_tracking.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
  if(TARGET unittest_tracking)
    target_link_libraries(unittest_tracking ${UNITEST_LIBRARIES})
  endif()

  ament_add_gtest(unittest_trackingmanager unittest_trackingmanager.cpp unittest_util.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
  if(TARGET unittest_trackingmanager)
    target_link_libraries(unittest_trackingmanager ${UNITEST_LIBRARIES})
  endif()
endif()
