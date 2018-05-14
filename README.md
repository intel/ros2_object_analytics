# ros2_object_analytics
Object Analytics (OA) is ROS2 wrapper for realtime object detection, localization and tracking.
These packages aim to provide real-time object analyses over RGB-D camera inputs, enabling ROS developer to easily create amazing robotics advanced features, like intelligent collision avoidance and semantic SLAM. It consumes [sensor_msgs::PointClould2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) data delivered by RGB-D camera, publishing topics on [object detection](https://github.com/intel/ros2_object_msgs), [object tracking](https://github.com/intel/ros2_object_analytics/tree/master/object_analytics_msgs), and [object localization](https://github.com/intel/ros2_object_analytics/object_analytics_msgs) in 3D camera coordination system.

OA keeps integrating with various "state-of-the-art" algorithms.
* Object detection offload to VPU, [ros_intel_movidius_ncs](https://github.com/intel/ros2_intel_movidius_ncs/tree/master/), with MobileNet SSD model and Caffe framework

## compiling dependencies
  ROS2 packages from [ROS2](https://github.com/ros2)
 * ament_cmake
  * std_msgs
  * sensor_msgs
  * geometry_msgs
  * rclcpp
  * rosidl_default_generators
  * rosidl_interface_packages
  * launch
  * ros2run
  * [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge)
  * [object_msgs](https://github.com/intel/ros2_object_msgs)
  * [ros2_intel_movidius_ncs](https://github.com/intel/ros2_intel_movidius_ncs)
  * [pcl_conversions](https://github.com/ros2/pcl_conversions/tree/ardent)
  * [ros2_message_filters](https://github.com/intel/ros2_message_filters)
  * [class_loader](https://github.com/ros/class_loader/tree/ardent)

  Other non-ROS packages
  * libpcl-all
  * libpcl-all-dev
  * python3-numpy
  * OpenCV3 (Minimum reqired OpenCV3.2, Kinetic comes with 3.3)

## build
  ```bash
  cd ${ros_ws} # "ros_ws" is the ament workspace root directory where this project is placed in
  ament build --symlink-install --isolated --only-packages object_analytics_node object_analytics_msgs object_analytics_launch
  ```
  You might don't need "--symlink-install" and "--isolated" opitions, it totally depends on your own build style.

## extra running dependencies
  The only supported RGB-D camera by now is Intel RealSense
  * [ros2_intel_realsense](https://github.com/intel/ros2_intel_realsense)
  ```
  realsense_ros2_camera
  ```

## command to launch object_analytics
   Movidius NCS is the only supported detection backend
   ```bash
   echo -e "param_file: mobilenetssd.yaml\ninput_topic: /object_analytics/rgb" > `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/config/default.yaml
   launch `ros2 pkg prefix object_analytics_launch`/share/object_analytics_launch/launch/analytics_movidius_ncs.py
   ```

 ## published topics
  /object_analytics/rgb ([sensor_msgs::msg::Image](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg))

  /object_analytics/pointcloud ([sensor_msgs::msg::PointCloud2](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg))

  /object_analytics/localization ([object_analytics_msgs::msg::ObjectsInBoxes3D](https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/ObjectsInBoxes3D.msg))

  /object_analytics/tracking ([object_analytics_msgs::msg::TrackedObjects](https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/TrackedObjects.msg))

  /movidius_ncs_stream/detected_objects ([object_msgs::msg::ObjectsInBoxes](https://github.com/intel/ros2_object_msgs/blob/master/msg/ObjectsInBoxes.msg))


## visualize tracking and localization results on RViz on ROS1 side over ros1_bridge
  As currently RViz2 is not ready for object analytics visualization, we use view result in RViz on the ROS1 side via ros1_bridge.
  Steps to be able visualize object analtyics result on RViz are as following.

  1. Rebuild ros1_bridge following official [guide](https://github.com/ros2/ros1_bridge) with ROS2 object_analytics_msgs has already built.

  2. Patch and rebuild ROS1's [object_analytics_visualization](https://github.com/intel/ros_object_analytics/tree/master/object_analytics_visualization) due to name changes.
  ```bash
  #in your ros1 workspace which has object_analytics_visualization source code
  cd <path/to/object_analytics/root>
  git apply ros1_bridge-patch.patch
  # you can find path following [link](https://github.com/intel/ros2_object_analytics/blob/master/patch/ros1_bridge-patch.patch)
  catkin_make --only-pkg-with-deps object_analytics_visualization
  catkin_make install
  ```
  3. Run
  ```bash
  # Shell A:
  . /opt/ros/kinetic/setup.bash
  roscore
  ```
  ```bash
  # Shell B:
  . /opt/ros/kinetic/setup.bash
  . <install-space-with-bridge>/local_setup.bash
  export ROS_MASTER_URI=http://localhost:11311
  ros2 run ros1_bridge dynamic_bridge
  ```
  ```bash
  # Shell C
  . /opt/ros/kinetic/setup.bash
  . <install-space-with-object-analytics-visualization>/setup.bash
  roslaunch object_analytics_visualization rviz.launch
  ```
  ```base
  # Shell D
  . <install-space-with-realsense-ros2-camera>/local_setup.bash
  realsense_ros2_camera
  ```
  ```base
  # Shell E
  . <install-space-with-object-analytics-launch>/local_setup.bash
  echo -e "param_file: mobilenetssd.yaml\ninput_topic: /object_analytics/rgb" > `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/config/default.yaml
  launch `ros2 pkg prefix object_analytics_launch`/share/object_analytics_launch/launch/analytics_movidius_ncs.py
  ```


## customize launch
  By default, object analytics will launch both tracking and localization features, but either tracking or localization or both can be dropped. Detailed please refer comments embedded in launch file.

###### *Any security issue should be reported using process at https://01.org/security*
