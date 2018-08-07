# ros2_object_analytics
Object Analytics (OA) is ROS2 wrapper for realtime object detection, localization and tracking.
These packages aim to provide real-time object analyses over RGB-D camera inputs, enabling ROS developer to easily create amazing robotics advanced features, like intelligent collision avoidance and semantic SLAM. It consumes [sensor_msgs::PointClould2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) data delivered by RGB-D camera, subscribing topic on [object detection](https://github.com/intel/ros2_object_msgs) by [ros2_intel_movidius_ncs](https://github.com/intel/ros2_intel_movidius_ncs), publishing topics on [object tracking](https://github.com/intel/ros2_object_analytics/tree/master/object_analytics_msgs) and [object localization](https://github.com/intel/ros2_object_analytics/object_analytics_msgs) in 3D camera coordination system.

OA keeps integrating with various "state-of-the-art" algorithms.
* Object detection offload to VPU, Intel Movidius NCS, with MobileNet SSD model and Caffe framework.

## System Requirements
We support Ubuntu Linux Bionic Beaver 18.04 on 64-bit. We not support Mac OS X and Windows.

## Dependencies
  Install ROS2 packages [ros-bouncy-desktop](https://github.com/ros2/ros2/wiki/Linux-Install-Debians)
  * ament_cmake
  * std_msgs
  * sensor_msgs
  * geometry_msgs
  * rclcpp
  * rosidl_default_generators
  * rosidl_interface_packages
  * launch
  * ros2run
  * class_loader
  * pcl_conversions
  * [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge)
  * [object_msgs](https://github.com/intel/ros2_object_msgs)
  * [ros2_message_filters](https://github.com/intel/ros2_message_filters)
  * [ros2_intel_realsense](https://github.com/intel/ros2_intel_realsense) (The only supported RGB-D camera by now is Intel RealSense)
  * [ros2_intel_movidius_ncs](https://github.com/intel/ros2_intel_movidius_ncs) (Movidius NCS is the only supported detection backend)

  Other non-ROS debian packages
  * libpcl-dev
  * python3-numpy
  * OpenCV3 (Minimum reqired OpenCV3.2, Kinetic comes with 3.3)

## Get Code
  ```bash
  mkdir ~/ros2_ws/src -p
  cd ~/ros2_ws/src

  git clone https://github.com/ros-perception/vision_opencv.git -b ros2
  git clone https://github.com/intel/ros2_object_msgs.git
  git clone https://github.com/intel/ros2_message_filters.git
  git clone https://github.com/intel/ros2_intel_realsense.git
  git clone https://github.com/intel/ros2_intel_movidius_ncs.git
  git clone https://github.com/intel/ros2_object_analytics.git
  ```
  
## Build
  ```bash
  cd ~/ros2_ws/src
  source /opt/ros/bouncy/local_setup.bash
  cd ${ros2_ws} # "ros2_ws" is the workspace root directory where this project is placed in
  colcon build --symlink-install
  ```

## Run
#### Realsense
  ```
  # Terminal 1:
  source /opt/ros/bouncy/local_setup.bash
  source ~/ros2_ws/local_setup.bash
  realsense_ros2_camera
  ```
#### NCS and OA
  ```
  # Terminal 2
  source /opt/ros/bouncy/local_setup.bash
  source ~/ros2_ws/local_setup.bash
  echo -e "param_file: mobilenetssd.yaml\ninput_topic: /object_analytics/rgb" > `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/config/default.yaml
  launch `ros2 pkg prefix object_analytics_launch`/share/object_analytics_launch/launch/analytics_movidius_ncs.py
  ```
#### OA Rviz
  ```
  # Terminal 3
  source /opt/ros/bouncy/local_setup.bash
  source ~/ros2_ws/local_setup.bash
  launch `ros2 pkg prefix object_analytics_launch`/share/object_analytics_launch/launch/object_rviz.py
  ```
## Subscribed topics
  /movidius_ncs_stream/detected_objects ([object_msgs::msg::ObjectsInBoxes](https://github.com/intel/ros2_object_msgs/blob/master/msg/ObjectsInBoxes.msg))

## Published topics
  /object_analytics/rgb ([sensor_msgs::msg::Image](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg))

  /object_analytics/pointcloud ([sensor_msgs::msg::PointCloud2](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg))

  /object_analytics/localization ([object_analytics_msgs::msg::ObjectsInBoxes3D](https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/ObjectsInBoxes3D.msg))

  /object_analytics/tracking ([object_analytics_msgs::msg::TrackedObjects](https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/TrackedObjects.msg))


## Customize launch
  By default, object analytics will launch both tracking and localization features, but either tracking or localization or both can be dropped. Detailed please refer comments embedded in launch file.

###### *Any security issue should be reported using process at https://01.org/security*
