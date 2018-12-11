/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#define PCL_NO_PRECOMPILE
#include <string>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <limits.h>
#include "util.hpp"

#include <iostream>
using namespace std;
#define LOOPRATE 30

void generate_dt(object_msgs::msg::ObjectsInBoxes::SharedPtr msg, rclcpp::Time stamp)
{
  msg->objects_vector.clear();
  ObjectInBox first = getObjectInBox(111, 249, 163, 145, "chair", 0.95f);
  msg->objects_vector.push_back(first);
  ObjectInBox second = getObjectInBox(373, 200, 58, 182, "bottle", 0.97f);
  msg->objects_vector.push_back(second);
  msg->header.frame_id = string("camera_depth_optical_frame");
  msg->header.stamp = stamp;
  msg->inference_time_ms = 50;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("object_analytics_fake");
  auto pub_pc = node->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/depth/color/points");
  auto pub_dt = node->create_publisher<object_msgs::msg::ObjectsInBoxes>("/movidius_ncs_stream/detected_objects");

  sensor_msgs::msg::PointCloud2::SharedPtr msg_pc = std::make_shared<sensor_msgs::msg::PointCloud2>();
  object_msgs::msg::ObjectsInBoxes::SharedPtr msg_dt = std::make_shared<object_msgs::msg::ObjectsInBoxes>();
  
  string prefix_path;
  prefix_path = ament_index_cpp::get_package_prefix("object_analytics_fake");
  prefix_path += "/share/object_analytics_fake/images/sample.pcd";
  if(access(prefix_path.c_str(), 0) == -1)
  {
    cout<<"FILE: ["<<prefix_path<<"] NOT EXIST!"<<endl;
    return 1;
  }
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (prefix_path, cloud) == -1)
  {
    cout<<"LOADPCDFILE: ["<<prefix_path<<"] ERROR!"<<endl;
    return 2;
  }
  pcl::toROSMsg(cloud, *msg_pc);
  msg_pc->header.frame_id = string("camera_depth_optical_frame");
    
  rclcpp::WallRate loop_rate(LOOPRATE);
  cout<<"Start to publish pointcloud2! -- "<<"LOOP_RATE: "<<LOOPRATE<<endl;
  
  while(1)
  {
    if(!rclcpp::ok())
        break;
    else
    {
      msg_pc->header.stamp = rclcpp::Clock().now();
      pub_pc->publish(msg_pc);
      generate_dt(msg_dt, msg_pc->header.stamp);
      pub_dt->publish(msg_dt);
      loop_rate.sleep();
      rclcpp::spin_some(node);
    }
  }

  return 0;
}
