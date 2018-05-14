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
#include <vector>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include "object_analytics_node/tracker/tracking_node.hpp"
#include "object_analytics_node/const.hpp"

namespace object_analytics_node
{
namespace tracker
{
// TrackingNode class implementation
using SubscribeImg = message_filters::Subscriber<sensor_msgs::msg::Image>;
using SubscribeObjs = message_filters::Subscriber<object_msgs::msg::ObjectsInBoxes>;
using Synchronizer = message_filters::TimeSynchronizer<object_msgs::msg::ObjectsInBoxes, sensor_msgs::msg::Image>;

TrackingNode::TrackingNode() : Node("TrackingNode")
{
  auto rgb_callback = [this](const typename sensor_msgs::msg::Image::SharedPtr image) -> void { this->rgb_cb(image); };
  sub_rgb_ = create_subscription<sensor_msgs::msg::Image>(Const::kTopicRgb, rgb_callback);

  auto obj_callback = [this](const typename object_msgs::msg::ObjectsInBoxes::SharedPtr objs) -> void {
    this->obj_cb(objs);
  };
  sub_obj_ = create_subscription<object_msgs::msg::ObjectsInBoxes>(Const::kTopicDetection, obj_callback);

  pub_tracking_ = create_publisher<object_analytics_msgs::msg::TrackedObjects>(Const::kTopicTracking);
  tm_ = std::make_unique<TrackingManager>(this);
  last_detection_ = builtin_interfaces::msg::Time();
  this_detection_ = builtin_interfaces::msg::Time();
  last_obj_ = nullptr;
  this_obj_ = nullptr;
}

void TrackingNode::rgb_cb(const sensor_msgs::msg::Image::ConstSharedPtr& img)
{
  rgbs_.push_back(img);
}

bool operator<(const builtin_interfaces::msg::Time& left, const builtin_interfaces::msg::Time& right)
{
  if (left.sec < right.sec)
  {
    return true;
  }
  if (left.sec == right.sec && left.nanosec < right.nanosec)
  {
    return true;
  }
  return false;
}

void TrackingNode::obj_cb(const object_msgs::msg::ObjectsInBoxes::ConstSharedPtr& objs)
{
  last_detection_ = this_detection_;
  this_detection_ = objs->header.stamp;
  last_obj_ = this_obj_;
  this_obj_ = objs;
  if (last_detection_ == builtin_interfaces::msg::Time())
  {
    return;
  }

  std::vector<sensor_msgs::msg::Image::ConstSharedPtr>::iterator rgb = rgbs_.begin();
  while (rgb != rgbs_.end())
  {
    if ((*rgb)->header.stamp < last_detection_)
    {
      RCLCPP_DEBUG(get_logger(), "slower, dropped");
      rgb = rgbs_.erase(rgb);
      continue;
    }
    if (!((*rgb)->header.stamp < this_detection_))
    {
      RCLCPP_DEBUG(get_logger(), "faster, break");
      break;
    }
    cv::Mat mat_cv = cv_bridge::toCvShare(*rgb, "bgr8")->image;
    if ((*rgb)->header.stamp == last_detection_)
    {
      RCLCPP_DEBUG(get_logger(), "rectify!");
      tm_->detect(mat_cv, last_obj_);
    }
    else
    {
      RCLCPP_DEBUG(get_logger(), "track!");
      tm_->track(mat_cv);
    }
    tracking_publish((*rgb)->header);
    rgb = rgbs_.erase(rgb);
  }
}

void TrackingNode::tracking_publish(const std_msgs::msg::Header& header)
{
  object_analytics_msgs::msg::TrackedObjects::SharedPtr msg =
      std::make_shared<object_analytics_msgs::msg::TrackedObjects>();
  msg->header = header;
  tm_->getTrackedObjs(msg);
  pub_tracking_->publish(msg);
}

}  // namespace tracker
}  // namespace object_analytics_node

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(object_analytics_node::tracker::TrackingNode, rclcpp::Node)
