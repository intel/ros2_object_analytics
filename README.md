DISCONTINUATION OF PROJECT

This project will no longer be maintained by Intel.

Intel has ceased development and contributions including, but not limited to, maintenance, bug fixes, new releases, or updates, to this project.  

Intel no longer accepts patches to this project.

If you have an ongoing need to use this project, are interested in independently developing it, or would like to maintain patches for the open source software community, please create your own fork of this project.  

Contact: webadmin@linux.intel.com
# ros2_object_analytics
Object Analytics (OA) is ROS2 module for real time object tracking and 3D localization.
These packages aim to provide real-time object analyses over RGB-D camera inputs, enabling ROS developer to easily create amazing robotics advanced features, like intelligent collision avoidance, people follow and semantic SLAM. It consumes [sensor_msgs::Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html) and [sensor_msgs::PointClould2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) data delivered by RGB-D camera, subscribes topic on [object detection](https://github.com/intel/ros2_object_msgs), publishes topics on [object tracking](https://github.com/intel/ros2_object_analytics/tree/master/object_analytics_msgs) in 2D RGB image and [object localization](https://github.com/intel/ros2_object_analytics/object_analytics_msgs) in 3D camera coordination system.

![OA_Architecture](https://github.com/intel/ros2_object_analytics/blob/devel/images/oa_architecture_devel.png "OA Architecture")

OA keeps integrating with various "state-of-the-art" algorithms.

## System Requirements
We support Ubuntu Linux Bionic Beaver 18.04 on 64-bit. We not support Mac OS X and Windows.

## Hardware Requirements
* Intel NUC (CPU: Intel i7-6700HQ @2.60GHz, Mem:16G)

## Dependencies
### Install ROS2 desktop packages [ros-dashing-desktop](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/)
  ```
  sudo apt-get install ros-dashing-desktop
  ```
  The ros-dashing-desktop will include below packages.
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

### Install ROS2 dependencies
  ```
  sudo apt-get install ros-dashing-cv-bridge ros-dashing-object-msgs ros-dashing-image-transport
  ```
  * [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge)
  * [object_msgs](https://github.com/intel/ros2_object_msgs)
  * [ros2_message_filters](https://github.com/ros2/message_filters)

## Install OA debian packages
  ```
  sudo apt-get install ros-dashing-object-analytics-msgs ros-dashing-object-analytics-node ros-dashing-object-analytics-rviz
  ```
  Notes: debian installed package does not support 2d tracking feature since the dependent opencv3.3 debian package is not available. For full feature, please build opencv3.3 and install object analytics from source.

## Install OA from source
### Build OpenCV3
  * OpenCV3 & opencv-contrib 3.3 (OA depends on tracking feature from OpenCV Contrib 3.3. OpenCV 3.3 is not integrated in ROS2 dashing release, need to build and install Opencv3 with contrib from source to apply tracking feature)
  ```
  # Build and Install OpenCV3 with opencv-contrib
  mkdir ${HOME}/opencv
  cd ${HOME}/opencv
  git clone https://github.com/opencv/opencv.git -b 3.3.0
  git clone https://github.com/opencv/opencv_contrib.git -b 3.3.0
  mkdir opencv/build -p
  cd opencv/build
  cmake -DOPENCV_EXTRA_MODULES_PATH=${HOME}/opencv/opencv_contrib/modules \ 
        -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_opencv_cnn_3dobj=OFF ..
  make -j8
  sudo make install
  sudo ldconfig
  ```
  
### Install OpenCV3 dependencies
  ```
  sudo apt-get install liblz4-dev
  ```

### Build ros2_object_analytics
  ```bash
  # get code
  cd ~/ros2_ws/src
  git clone https://github.com/intel/ros2_object_analytics.git -b devel (devel branch is the latest code with 2D tracking features, while master branch is stable for ros2 released distributions)

  # build
  cd ~/ros2_ws
  source /opt/ros/dashing/setup.bash
  colcon build --symlink-install
  ```

## Run
### Customize launcher
  ```
  Object Analytics Module consumes 2D image/Point cloud/Detection bounding box from outside, so you need config the sources according to your specific condition. We provided a sample launch file "object_analytics_sample.launch.py", you can customize the remapping topics to have your own launcher.

  By default, object analytics will launch both tracking and localization features, but either tracking or localization or both can be dropped. Detailed please refer arguments embedded in launch file "object_analytics_sample.launch.py".
  ```

#### Run sample launcher co-work with Realsense and OpenVINO
  ```
  # Start OA demo to co-work with Realsense and OpenVINO
  Step1: launch ROS2 Realsense and OpenVINO
    a) Please refer below links to enable ROS2 realsense and openvino.
       Realsense: https://github.com/intel/ros2_intel_realsense
       OpenVino:  https://github.com/intel/ros2_openvino_toolkit
    b) Make sure below topics works well, or please config the remapping topics in "object_analytics_sample.launch.py":
       1. /camera/color/image_raw
       2. /camera/aligned_depth_to_color/color/points
       3. /ros2_openvino_toolkit/detected_objects

  Step2: if ros2_openvino_toolkit got from Robotics_SDK
       ros2 launch object_analytics_node object_analytics_sample.launch.py
  ```

![OA_demo_video](https://github.com/intel/ros2_object_analytics/blob/master/images/oa_demo.gif "OA demo video")

## Subscribed topics

  /object_analytics/rgb ([sensor_msgs::msg::Image](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg))

  /object_analytics/detected_objects ([object_msgs::msg::ObjectsInBoxes](https://github.com/intel/ros2_object_msgs/blob/master/msg/ObjectsInBoxes.msg))

  /object_analytics/pointcloud ([sensor_msgs::msg::PointCloud2](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg))

## Published topics

  /object_analytics/localization ([object_analytics_msgs::msg::ObjectsInBoxes3D](https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/ObjectsInBoxes3D.msg))

  /object_analytics/tracking ([object_analytics_msgs::msg::TrackedObjects](https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/TrackedObjects.msg))

## Tools
To ensure the algorithms in OA components to archive best performance in ROS2, we have below tools used to examine design/development performance/accuracy/precision..., more tools are in developing progress and will publish later.

### 1. tracker_regression
The tools is used to feed tracking node with raw images from datasets within fixed time interval(33ms), also simulate detector send ground truth as detections to tracking node for rectification, then receive tracking results for precision and recall statistics. It support multiple algorithms(dynamic configure to tracking node when start).

#### * Tools usages
    # ros2 run object_analytics_node tracker_regression --options
           options: [-a algorithm] [-p dataset_path] [-t dataset_type] [-n dataset_name] [-h];
           -h : Print this help function.
           -a algorithm_name : Specify the tracking algorithm in the tracker.
              supported algorithms: KCF,TLD,BOOSTING,MEDIAN_FLOW,MIL,GOTURN.
           -p dataset_path : Specify the tracking datasets location.
           -t dataset_type : Specify the dataset type: video,image.
           -n dataset_name : Specify the dataset name
#### * Example:

    Video dataset with tracking algorithm("MEDIAN_FLOW"):
    # ros2 run object_analytics_node tracker_regression -p /your/video/datasets/root/path -t video -n dudek -a MEDIAN_FLOW

    Image dataset with default algorithm("MEDIAN_FLOW"):
    # ros2 run object_analytics_node tracker_regression -p /your/image/datasets/root/path -t image -n Biker -a MEDIAN_FLOW

#### * Dataset:

 Support both video and image dataset, but you may need to translate into below formats.

 Video dataset: ([Refer to opencv_extra tracking dataset](https://github.com/opencv/opencv_extra/tree/master/testdata/cv/tracking))

     track_vid/    (/your/video/datasets/root/path)
           ├── david
           │   ├── data
           │   │   └── david.webm
           │   ├── david.yml
           │   ├── gt.txt
           │   └── initOmit
           │       └── david.txt
           ├── dudek
           │   ├── data
           │   │   └── dudek.webm
           │   ├── dudek.yml
           │   ├── gt.txt
           │   └── initOmit
           │       └── dudek.txt
           ├── faceocc2
           │   ├── data
           │   │   └── faceocc2.webm
           │   ├── faceocc2.yml
           │   ├── gt.txt
           │   └── initOmit
           │       └── faceocc2.txt
           ├── list.txt (Note: this is manually added, list the dataset names which will be used)
           └── README.md

  Image dataset: ([Refer to database from Computer Vision Lab@HYU](http://cvlab.hanyang.ac.kr/tracker_benchmark/datasets.html))

     track_img/    (/your/video/datasets/root/path)
           ├── Biker
           ├── Bird1
           ├── Bird2
           ├── list.txt (Note: this is manually added, list the dataset names which will be used)
           ├── Man
           ├── Matrix
           └── Woman


###### *Any security issue should be reported using process at https://01.org/security*
