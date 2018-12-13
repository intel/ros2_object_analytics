// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OBJECT_ANALYTICS_FAKE_HPP_
#define OBJECT_ANALYTICS_FAKE_HPP_

#include <ament_index_cpp/get_package_prefix.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <rclcpp/rclcpp.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>
#include <unistd.h>
#include <limits.h>

#include <string>
#include <memory>
#include <utility>
#include <vector>

using sensor_msgs::msg::RegionOfInterest;
using object_msgs::msg::Object;
using object_msgs::msg::ObjectInBox;

Object getObject(const std::string & name, const float probability);
RegionOfInterest getRoi(int x_offset, int y_offset, int width, int height);
ObjectInBox getObjectInBox(
  int x_offset, int y_offset, int width, int height,
  const std::string & name, const float probability);
void generate_dt(object_msgs::msg::ObjectsInBoxes::SharedPtr msg, rclcpp::Time stamp);

#endif  // OBJECT_ANALYTICS_FAKE_HPP_
