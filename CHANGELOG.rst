changelog for ros2_object_analytics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.1 (2018-12-6)
------------------
* Enable bloom package build for ROS2 crystal release

0.5.0 (2018-11-14)
------------------
* Enable dataset support in tracking module.
* Add one regression tool to track tracking precision and perf
* Enabled more tracking features such as KCF/TLD/BOOSTING/MEDIAN_FLOW
* remove merger-node to simplify the code
* remove message_filter used in rviz for better display
* Upload demo screensot and video
* Enable moving object feature
* delete object_analytics_launch package and create launch file in object_analytics_node

0.4.0 (2018-09-7)
------------------
* fully support ros2 implenetation without ros1_bridge connunication
* support object_analytics_rviz on ros2
* splite pointcloud2 to xyz and rgb
* restruct tracking and localization object message type
* optimize localization segmentation implementation
* seperate ncs launch interface
* enable cppcheck and unittest
* support ubuntu 18.04

0.3.0 (2018-05-14)
------------------
* Support tracking object in 2D dimensional space.
* Support locating object in 3D dimisional space.
