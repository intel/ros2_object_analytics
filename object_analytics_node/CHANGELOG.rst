^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package object_analytics_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.1 (2018-12-06)
------------------
* Merge pull request `#55 <https://github.com/intel/ros2_object_analytics/issues/55>`_ from intel/devel
  update launcher file and license
* update launcher file and license
* Merge pull request `#54 <https://github.com/intel/ros2_object_analytics/issues/54>`_ from nuclearsandwich/dependencies
  Reconcile dependencies between package.xml and CMakeLists.txt
* Merge pull request `#53 <https://github.com/intel/ros2_object_analytics/issues/53>`_ from intel/devel
  remove depth_image_proc when launch
* Add ament_index dependencies to the package manifest.
  Failures due to these missing dependencies were encountered on the ROS
  buildfarm. I added ament_index_python as an exec depend as it appears to
  be used by the launch files included with the package.
* remove depth_image_proc when launch
  depth_image_proc will has reduce image transport performance, subscribe pointcloud from camera directly, although there has blue points. To fix later with good performance.
* Merge pull request `#52 <https://github.com/intel/ros2_object_analytics/issues/52>`_ from intel/for_release
  For release
* add BUILD_TRACKING option
  Add BUILD_TRACKING option in order to disable tracking build for crystal bloom release, as opencv3.3 no provided debian installation from ubuntu18.04
* remap topic in launch instead of code hardcode
  remap topic name in launch.py instead of hardcode in code
* Contributors: Chris Ye, Steven! Ragnarök

0.5.0 (2018-11-14)
------------------
* update launch file
  * delete object_analytics_launch package and create object_analytics_node/launch folder
  * rename node name from composition to object_anaytics_node
  * fix loading MovementNode to class typo
  * update readme to simple launch example
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Merge pull request `#49 <https://github.com/yechun1/ros2_object_analytics/issues/49>`_ from ahuizxc/merged_mo
  Enable moving object feature
* changed code as recommended
* merged with moving object project
* Merge pull request `#44 <https://github.com/yechun1/ros2_object_analytics/issues/44>`_ from ahuizxc/master
  remove merger-node to simplify the code
* remove merger-node to simplify the code
* Merge pull request `#43 <https://github.com/yechun1/ros2_object_analytics/issues/43>`_ from intel/fix_build_issue
  add semicolon after RCLCPP_INFO
* add semicolon after RCLCPP_INFO
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Merge pull request `#42 <https://github.com/yechun1/ros2_object_analytics/issues/42>`_ from challen-zhou/master
  Enable tracking algorithm select in tracker_regression tool
* Merge pull request `#41 <https://github.com/yechun1/ros2_object_analytics/issues/41>`_ from qingmingjie/test
  modify unittest_trackingmanager.cpp based on tracking new code.
* Enable tracking algorithm select in tracker_regression tool
  1. Enable tracker_regression tool to select algorithms for tracking.
  2. Add tracker_regression tool usage introduction.
* modify unittest_trackingmanager.cpp
* Merge pull request `#40 <https://github.com/yechun1/ros2_object_analytics/issues/40>`_ from intel/ament_test
  Ament test
* update copyright format to pass colcon test
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* use ament_lint_auto for ros2 code style check
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Merge pull request `#39 <https://github.com/yechun1/ros2_object_analytics/issues/39>`_ from challen-zhou/master
  Enable tracking dataset support
* Tuning tracker's roburst to detection
  1. Tuning tracker's strategy to be more roburst to detection's result.
  2. Change code style to match ROS2 requirement.
* Enabled more tracking features
  1. Changed tracking tick tock to image data topic instead of detection topic.
  2. Enabled more tracking features such as KCF/TLD/BOOSTING/MEDIAN_FLOW.
* Enable tracking dataset support
  1. Add functions to enable dataset support in tracking module.
  2. Add one regression tool to track tracking precision and perf.
* Merge pull request `#38 <https://github.com/yechun1/ros2_object_analytics/issues/38>`_ from ahuizxc/remove_plane_seg
  remove the plane segment part according to the new algorithm
* remove the plane segment part according to the new algorithm
* Merge pull request `#36 <https://github.com/yechun1/ros2_object_analytics/issues/36>`_ from intel/message_filter
  update message_filter and interface
* update message_filter and interface
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Contributors: Chris Ye, ahuizxc, challen.zhou, qingmingjie

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
