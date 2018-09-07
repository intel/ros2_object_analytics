^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package object_analytics_rviz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2018-09-07)
------------------
* update maintainer
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* fix c++ code style issue
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* add ament_cppcheck
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Merge pull request `#29 <https://github.com/intel/ros2_object_analytics/issues/29>`_ from ahuizxc/multi-obj-in-rviz
  support showing multi objects localization marks in RVIZ
* deleted these two lines and changed the clear objects code
* support showing multi objects localization marks in RVIZ
* Merge pull request `#23 <https://github.com/intel/ros2_object_analytics/issues/23>`_ from intel/perf
  Merge detected object in localization and tracking
* Merge detected object in localization and tracking
  Merged detected object_name and probability in localization and tracking object,
  so that users could only subscribe localization or tracking, needn't sub detected object
  and needn't do message fileter again.
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Revert "moving object rectangle with correct position"
  ros2_movidius_ncs have fixed roi issue, revert the patch.
  This reverts commit 17a1120a37905e2fe619225f89f0903e7fb90f97.
* Merge pull request `#18 <https://github.com/intel/ros2_object_analytics/issues/18>`_ from intel/rviz_cpp
  fix rviz line draw malposition issue
* fix rviz line draw malposition issue
  set marker.pos.position is not correct, the value should be zero, just remove it and it will use default zero value.
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* moving object rectangle with correct position
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Merge pull request `#17 <https://github.com/intel/ros2_object_analytics/issues/17>`_ from intel/rviz_cpp
  ported image_publisher work on ros2 rviz
* optimize parameter to be passed by reference
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* reduce scope of variable
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* ported image_publisher work on ros2 rviz
  enabled image_publisher on cpp and py, cpp pass test while py not work.
  image publisher used to display rectangle of tracked object on image viewer.
  this is full ros2 rviz support without ros1_bridge.
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Merge pull request `#15 <https://github.com/intel/ros2_object_analytics/issues/15>`_ from intel/rviz_cpp
  Rviz cpp
* Initialize object_analytics_rviz for ros2
  rewrite marker_publisher with cpp from python.
  subscibe detection/localization/tracking topic and publish box_3d_markers,
  to display object analytics result on rviz.
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Contributors: Chris Ye, ahuizxc

0.3.0 (2018-07-03)
------------------
* create marker_publisher.cpp, subscribe detection/localization/tracking and publish 3d box to rviz markers
* create image_publisher.cpp, subscribe detection/localization/tracking and publish 2d box to rviz image
