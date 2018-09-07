^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package object_analytics_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2018-09-07)
------------------
* update maintainer
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* remove unused cloud_segment parameter
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* fix c++ code style issue
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* add ament_cppcheck
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Merge pull request `#31 <https://github.com/intel/ros2_object_analytics/issues/31>`_ from ahuizxc/new-segmentation-alg
  new segmentation algorithm, faster and more accurate
* fixed bugs to pass CI test
* fix format bug
* Merge pull request `#32 <https://github.com/intel/ros2_object_analytics/issues/32>`_ from intel/fix_performance_issue
  Remove ncs launch code out of OA code
* Remove ncs launch code
  add ncs launch step in Readme.txt
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* new segmentation algorithm, faster and more accurate
* Merge pull request `#30 <https://github.com/intel/ros2_object_analytics/issues/30>`_ from qingmingjie/test
  update unittest_trackingmanager.cpp
* update unittest_trackingmanager.cpp
* Merge pull request `#24 <https://github.com/intel/ros2_object_analytics/issues/24>`_ from qingmingjie/add_test
  add tracking test code
* Modify unittest_trackingmanager.cpp
* add tracking test code
* Merge pull request `#23 <https://github.com/intel/ros2_object_analytics/issues/23>`_ from intel/perf
  Merge detected object in localization and tracking
* Update unittest after add object in objects_in_boxes
  unnitest also need to update after added object_name and probility in ObjectInBox3D msgs.
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Merge detected object in localization and tracking
  Merged detected object_name and probability in localization and tracking object,
  so that users could only subscribe localization or tracking, needn't sub detected object
  and needn't do message fileter again.
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Merge pull request `#22 <https://github.com/intel/ros2_object_analytics/issues/22>`_ from intel/perf
  splite pointcloud2 to rgb and xyz
* splite pointcloud2 to rgb and xyz
  Currently localization using pcl segementation api which not required rgb,
  this patch splited Pointcloud2 to XYZ, reduced half size of PointCloud2 (from 9.83M reduce to 4.91M)
  imporved pointcloud2 sub-pub transport performance.
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Merge pull request `#17 <https://github.com/intel/ros2_object_analytics/issues/17>`_ from intel/rviz_cpp
  ported image_publisher work on ros2 rviz
* optimize parameter to be passed by reference
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Merge pull request `#14 <https://github.com/intel/ros2_object_analytics/issues/14>`_ from intel/tracking_roi
  fix tracking roi not match objected roi
* fix tracking roi not match objected roi
  tracking will change objects roi because of size boundary.
  Backup objected roi before handle tracking, after tracking handled, restore objected roi,
  so that the roi of tracking and objected could match.
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Revert "add CMakeLists"
  function not stable, revert first
  This reverts commit 0275614c16c0a7a2ba8612abdb665b11e397a951.
* Merge pull request `#8 <https://github.com/intel/ros2_object_analytics/issues/8>`_ from qingmingjie/add_test
  add test
* add CMakeLists
* add message_filters build_depend
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Merge pull request `#4 <https://github.com/intel/ros2_object_analytics/issues/4>`_ from intel/for_deb_build
  add class_loader dependence for deb package build
* add class_loader dependence for deb package build
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* add cv_bridge dep in package.xml
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* add CHANGELOG.rst
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Remove useless parameter to supress warning
  Signed-off-by: Peter Han <peter9606@gmail.com>
* Change class_loader to default branch
  Signed-off-by: Peter Han <peter.han@intel.com>
* Contributors: Chris Ye, Peter Han, ahuizxc, qingmingjie, yechun1

0.3.0 (2018-05-14)
------------------
* Change class_loader to default branch
  Signed-off-by: Peter Han <peter.han@intel.com>
* ros2 object analytics package - V0.3.0 Release
  Signed-off-by: Peter Han <peter.han@intel.com>
* Contributors: Peter Han
