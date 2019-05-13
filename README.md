# ros2_object_analytics
Object Analytics (OA) is ROS2 wrapper for realtime object tracking and 3D localization.
These packages aim to provide real-time object analyses over RGB-D camera inputs, enabling ROS developer to easily create amazing robotics advanced features, like intelligent collision avoidance, people follow and semantic SLAM. It consumes [sensor_msgs::PointClould2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) data delivered by RGB-D camera, subscribs topic on [object detection](https://github.com/intel/ros2_object_msgs) by [ros2_intel_movidius_ncs](https://github.com/intel/ros2_intel_movidius_ncs) or by [ros2_openvino_toolkit](https://github.com/intel/ros2_openvino_toolkit), publishs topics on [object tracking](https://github.com/intel/ros2_object_analytics/tree/master/object_analytics_msgs) in 2D RGB image and [object localization](https://github.com/intel/ros2_object_analytics/object_analytics_msgs) in 3D camera coordination system.

![OA_Architecture](https://github.com/intel/ros2_object_analytics/blob/master/images/oa_architecture.png "OA Architecture")

OA keeps integrating with various "state-of-the-art" algorithms.
* Object detection offload to VPU, Intel Movidius NCS, with MobileNet SSD model and Caffe framework(TODO).

## System Requirements
We support Ubuntu Linux Bionic Beaver 18.04 on 64-bit. We not support Mac OS X and Windows.

## Hardware Requirements

* Intel NUC (CPU: Intel i7-6700HQ @2.60GHz, Mem:16G)
* Intel Movidius Neural Compute Stick(required by ros2_intel_movidius_ncs)
* Intel RealSense D435/D415

## Dependencies
### Install ROS2 desktop packages [ros-crystal-desktop](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/)
  ```
  sudo apt-get install ros-crystal-desktop
  ```
  The ros-crystal-desktop will include below packages.
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

### Install Intel® RealSense™ SDK 2.0 [tag v2.17.1](https://github.com/IntelRealSense/librealsense/tree/v2.17.1)
[Install from source code](https://github.com/IntelRealSense/librealsense/blob/v2.17.1/doc/installation.md)(Recommended)

```
 sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
 sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
 mkdir -p ~/code && cd ~/code
 git clone https://github.com/IntelRealSense/librealsense
 cd ~/code/librealsense
 git checkout v2.17.1
 mkdir build && cd build
 cmake ../ -DBUILD_EXAMPLES=true
 sudo make uninstall 
 make clean
 make
 sudo make install
 cd ..
 sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
 sudo udevadm control --reload-rules
 udevadm trigger
```

[Install from package](https://github.com/IntelRealSense/librealsense/blob/v2.17.1/doc/distribution_linux.md)

### Install ROS2 dependences
  ```
  sudo apt-get install ros-crystal-cv-bridge ros-crystal-object-msgs ros-crystal-image-transport ros-crystal-realsense-camera-msgs
  ```
  * [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge)
  * [object_msgs](https://github.com/intel/ros2_object_msgs)
  * [ros2_message_filters](https://github.com/ros2/message_filters)
  * [ros2_intel_realsense](https://github.com/intel/ros2_intel_realsense) (The only supported RGB-D camera by now is Intel RealSense)

#### Install ros2_intel_movidius_ncs (Install NCS or OpenVINO as demand)
  ros2_intel_movidius has not integrated in ROS2 release, so there is no debian package available for Movidius NCS installation, need to build from source, more details please referece to https://github.com/intel/ros2_intel_movidius_ncs).
  ```
  # build ncsdk
  mkdir ~/code
  cd ~/code
  git clone https://github.com/movidius/ncsdk
  git clone https://github.com/movidius/ncappzoo
  cd ~/code/ncsdk
  make install
  ln -sf ~/code/ncappzoo /opt/movidius/ncappzoo

  # build ros2_intel_movidius_ncs
  mkdir ~/ros2_ws/src -p
  cd ~/ros2_ws/src
  git clone https://github.com/intel/ros2_intel_movidius_ncs.git
  cd ~/ros2_ws
  source /opt/ros/crystal/setup.bash
  colcon build --symlink-install (Install python3-colcon-common-extensions by apt-get if colcon command not exist)

  # build CNN model (Please plugin NCS device on the host while compiling)
  cd /opt/movidius/ncappzoo/caffe/SSD_MobileNet
  make

  # Copy object label file to NCSDK installation location.
  cp ~/ros2_ws/src/ros2_intel_movidius_ncs/data/labels/* /opt/movidius/ncappzoo/data/ilsvrc12/

  ```
#### Install ros2_openvino_toolkit (Install NCS or OpenVINO as demand)
  The OpenVINO™ (Open visual inference and neural network optimization) toolkit provides a ROS-adaptered runtime framework of neural network which quickly deploys applications and solutions for vision inference. By leveraging Intel® OpenVINO™ toolkit and corresponding libraries, this runtime framework extends workloads across Intel® hardware (including accelerators) and maximizes performance. Please see [ros2_openvino_toolkit](https://github.com/intel/ros2_openvino_toolkit) for installation.

## Install OA debian packages
  ```
  sudo apt-get install ros-crystal-object-analytics-msgs ros-crystal-object-analytics-node ros-crystal-object-analytics-rviz
  ```
  The object analytics packages installation have been completed. You could jump to [Run](https://github.com/intel/ros2_object_analytics/tree/update_readme#run) for executing, you could also install OA from source for more features.
  Notes: debian installed package does not support 2d tracking feature as the dependent opencv3.3 has no debian available. For full feature, please build opencv3.3 and install object analytics from source.

## Install OA from source
### Build OpenCV3
  * OpenCV3 & opencv-contrib 3.3 (OA depends on tracking feature from OpenCV Contrib 3.3. OpenCV 3.3 is not integrated in ROS2 Crystal release, need to build and install Opencv3 with contrib from source to apply tracking feature)
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
  
### Install OpenCV3 dependences
  ```
  sudo apt-get install liblz4-dev
  ```

### Build ros2_intel_realsense
  ```
  # get code
  mkdir ~/ros2_ws/src -p
  cd ~/ros2_ws/src
  https://github.com/intel/ros2_intel_realsense.git
  
  # build
  cd ~/ros2_ws
  source /opt/ros/crystal/setup.bash
  colcon build --symlink-install
  ```

### Build ros2_object_analytics
  ```bash
  # get code
  cd ~/ros2_ws/src
  git clone https://github.com/intel/ros2_object_analytics.git -b devel (devel branch is the latest code with 2D tracking features, while master branch is stable for ros2 bloom release)

  # build
  cd ~/ros2_ws
  source /opt/ros/crystal/setup.bash
  colcon build --symlink-install
  ```

## Run
#### Object Analytics with NCS
  ```
  # Configure NCS default.yaml
  source /opt/ros/crystal/setup.bash
  source ~/ros2_ws/install/local_setup.bash
  echo -e "param_file: mobilenetssd.yaml\ninput_topic: /object_analytics/rgb" > `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/config/default.yaml

  # Start OA demo with NCS
  ros2 launch object_analytics_node object_analytics_with_ncs.launch.py
  ```

#### Object Analytics with OpenVINO
  ```
  # Start OA demo with OpenVINO
  source /opt/ros/crystal/setup.bash
  source ~/ros2_ws/install/local_setup.bash
  
  Option1: if ros2_openvino_toolkit built from source code
  ros2 launch object_analytics_node object_analytics_with_openvino_oss.launch.py

  Option2: if ros2_openvino_toolkit got from Robotics_SDK
  ros2 launch object_analytics_node object_analytics_with_openvino_sdk.launch.py
  ```

![OA_demo_video](https://github.com/intel/ros2_object_analytics/blob/master/images/oa_demo.gif "OA demo video")

## Subscribed topics
  /object_analytics/detected_objects ([object_msgs::msg::ObjectsInBoxes](https://github.com/intel/ros2_object_msgs/blob/master/msg/ObjectsInBoxes.msg))

## Published topics
  /object_analytics/rgb ([sensor_msgs::msg::Image](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg))

  /object_analytics/pointcloud ([sensor_msgs::msg::PointCloud2](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg))

  /object_analytics/localization ([object_analytics_msgs::msg::ObjectsInBoxes3D](https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/ObjectsInBoxes3D.msg))

  /object_analytics/tracking ([object_analytics_msgs::msg::TrackedObjects](https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/TrackedObjects.msg))

  /object_analytics/movement ([object_analytics_msgs::msg::MovingObjectsInFrame](https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/MovingObjectsInFrame.msg))
## Customize launch
  By default, object analytics will launch both tracking and localization features, but either tracking or localization or both can be dropped. Detailed please refer comments embedded in launch file.

## Tools
To ensure the algorims in OA components to archive best performance in ROS2, we have below tools used to examine design/development performance/accuracy/precision..., more tools are in developing progress and will publish later.

### 1. tracker_regression
The tools is used to feed tracking node with raw images from datasets within fixed time interval(33ms), also simulate detector send groundtruth as detections to tracking node for rectification, then receive tracking results for precision and recall stastics. It support multiple algorithms(dynamic configure to tracking node when start).

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
           ├── list.txt (Note: this is mannually added, list the dataset names which will be used)
           └── README.md

  Image dataset: ([Refer to database from Computer Vision Lab@HYU](http://cvlab.hanyang.ac.kr/tracker_benchmark/datasets.html))

     track_img/    (/your/video/datasets/root/path)
           ├── Biker
           ├── Bird1
           ├── Bird2
           ├── list.txt (Note: this is mannually added, list the dataset names which will be used)
           ├── Man
           ├── Matrix
           └── Woman


###### *Any security issue should be reported using process at https://01.org/security*
