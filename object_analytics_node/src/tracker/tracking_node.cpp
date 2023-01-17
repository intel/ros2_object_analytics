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

#include "tracker/tracking_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <class_loader/register_macro.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <string>
#include <vector>

#include "tracker/tracking.hpp"
#include "const.hpp"

namespace object_analytics_node
{
// TrackingNode class implementation
using SubscribeImg = message_filters::Subscriber<sensor_msgs::msg::Image>;
using SubscribeObjs =
  message_filters::Subscriber<object_msgs::msg::ObjectsInBoxes>;
using Synchronizer =
  message_filters::TimeSynchronizer<object_msgs::msg::ObjectsInBoxes,
    sensor_msgs::msg::Image>;

TrackingNode::TrackingNode(rclcpp::NodeOptions options)
: Node("TrackingNode", options)
{
  auto rgb_callback =
    [this](const typename sensor_msgs::msg::Image::SharedPtr image) -> void {
      this->rgb_cb(image);
    };
  sub_rgb_ = create_subscription<sensor_msgs::msg::Image>(
    Const::kTopicRgb, rclcpp::SensorDataQoS(), rgb_callback);

  auto obj_callback =
    [this](const typename object_msgs::msg::ObjectsInBoxes::SharedPtr objs)
    -> void {this->obj_cb(objs);};
  sub_obj_ = create_subscription<object_msgs::msg::ObjectsInBoxes>(
    Const::kTopicDetection, rclcpp::ServicesQoS(), obj_callback);

  pub_tracking_ = create_publisher<object_analytics_msgs::msg::TrackedObjects>(
    Const::kTopicTracking, rclcpp::ServicesQoS());

  tm_ = std::make_unique<tracker::TrackingManager>();
  last_detection_.tv_sec = 0;
  last_detection_.tv_nsec = 0;
  this_detection_.tv_sec = 0;
  this_detection_.tv_nsec = 0;

  kRgbQueueSize = 5;
  rgbs_.reserve(kRgbQueueSize);
}

void TrackingNode::rgb_cb(const sensor_msgs::msg::Image::ConstSharedPtr & img)
{
  static timespec stmp;
  stmp.tv_sec = img->header.stamp.sec;
  stmp.tv_nsec = img->header.stamp.nanosec;

  uint64_t t_interval = (img->header.stamp.sec - stmp.tv_sec) * 1e3 +
    (img->header.stamp.nanosec - stmp.tv_nsec) / 1e6;
  RCUTILS_LOG_DEBUG("received rgb frame frame_id(%s), interval(%ld)",
    img->header.frame_id.c_str(), t_interval);

  struct timespec stamp;
  stamp.tv_sec = img->header.stamp.sec;
  stamp.tv_nsec = img->header.stamp.nanosec;

  cv::Mat mat_cv = cv_bridge::toCvShare(img, "bgr8")->image;

  std::shared_ptr<sFrame> frame = std::make_shared<sFrame>(mat_cv, stamp);

  if (this_detection_ == stamp) {
    RCLCPP_DEBUG(get_logger(), "rectify in rgb_cb!");
    tm_->detectRecvProcess(frame, this_obj_);
    RCUTILS_LOG_INFO("Rectify  the RGB images");
  } else {
    tm_->track(frame);
    RCUTILS_LOG_INFO("Track the RGB images");
  }

#ifndef NDEBUG
  cv::Mat mat_show = mat_cv.clone();
  const std::vector<std::shared_ptr<tracker::Tracking>> trackings =
    tm_->getTrackedObjs();
  for (auto t : trackings) {
    cv::Rect2d r = t->getTrackedRect();
    int track_id = t->getTrackingId();

    rectangle(mat_show, r, cv::Scalar(255, 0, 0), 1, cv::LINE_8);
    putText(mat_show, std::to_string(track_id), (r.tl() + r.br()) / 2,
      cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);

    std::vector<tracker::Traj> trajs = t->getTrajs();
    cv::Point2d p_start = (trajs[0].rect_.tl() + trajs[0].rect_.br()) / 2.0f;
    for (uint32_t i = 1; i < trajs.size(); i++) {
      cv::Point2d p_end = (trajs[i].rect_.tl() + trajs[i].rect_.br()) / 2.0f;
      cv::line(mat_show, p_start, p_end, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
      p_start = p_end;
    }
  }

  cv::imshow("tracking_cb", mat_show);
  cv::waitKey(1);

  if (trackings.size() <= 0) {RCUTILS_LOG_INFO("No tracking to publish");}
#endif

  tracking_publish(img->header);

  rgbs_.push_back(frame);
  if (kRgbQueueSize < rgbs_.size()) {
    rgbs_.erase(rgbs_.begin());
  }
}

bool operator<(
  const builtin_interfaces::msg::Time & left,
  const builtin_interfaces::msg::Time & right)
{
  if (left.sec < right.sec) {
    return true;
  }
  if (left.sec == right.sec && left.nanosec < right.nanosec) {
    return true;
  }
  return false;
}

void TrackingNode::obj_cb(
  const object_msgs::msg::ObjectsInBoxes::ConstSharedPtr & objs)
{
  if (objs->objects_vector.size() == 0) {
    return;
  }

  last_detection_ = this_detection_;
  this_detection_.tv_sec = objs->header.stamp.sec;
  this_detection_.tv_nsec = objs->header.stamp.nanosec;
  this_obj_.clear();

  for (uint32_t i = 0; i < objs->objects_vector.size(); i++) {
    object_msgs::msg::Object dobj = objs->objects_vector[i].object;
    sensor_msgs::msg::RegionOfInterest droi = objs->objects_vector[i].roi;
    Object c_obj;
    c_obj.Category_ = dobj.object_name;
    c_obj.Confidence_ = dobj.probability;
    c_obj.Stamp_ = this_detection_;
    c_obj.BoundBox_.x = droi.x_offset;
    c_obj.BoundBox_.y = droi.y_offset;
    c_obj.BoundBox_.width = droi.width;
    c_obj.BoundBox_.height = droi.height;

    this_obj_.push_back(c_obj);
  }

  RCUTILS_LOG_DEBUG(
    "received obj detection frame_id(%s), stamp(sec(%d),nsec(%d)), "
    "img_buff_count(%ld)!\n",
    objs->header.frame_id.c_str(), objs->header.stamp.sec,
    objs->header.stamp.nanosec, rgbs_.size());

  std::vector<std::shared_ptr<sFrame>>::iterator rgb = rgbs_.begin();
  while (rgb != rgbs_.end()) {
    RCUTILS_LOG_DEBUG("iterate queue buffer stamp(sec(%ld),nsec(%ld))!\n",
      (*rgb)->stamp.tv_sec, (*rgb)->stamp.tv_nsec);
    if ((*rgb)->stamp < this_detection_) {
      RCLCPP_DEBUG(get_logger(), "slower, dropped");
      rgb = rgbs_.erase(rgb);
      continue;
    }
    if ((*rgb)->stamp == this_detection_) {
      RCUTILS_LOG_DEBUG("rectify frame_id(%s), stamp(sec(%d),nsec(%d))\n",
        objs->header.frame_id.c_str(), objs->header.stamp.sec,
        objs->header.stamp.nanosec);

      tm_->detectRecvProcess((*rgb), this_obj_);
      break;
    }

    rgb++;
  }
}

void TrackingNode::tracking_publish(const std_msgs::msg::Header & header)
{
  object_analytics_msgs::msg::TrackedObjects msg;
  msg.header = header;

  const std::vector<std::shared_ptr<tracker::Tracking>> trackings =
    tm_->getTrackedObjs();
  if (trackings.size() > 0) {
    fillTrackedObjsMsg(msg, trackings);
    pub_tracking_->publish(msg);
  } else {
    RCUTILS_LOG_WARN("No objects to publish!");
  }
}

void TrackingNode::fillTrackedObjsMsg(
  object_analytics_msgs::msg::TrackedObjects & objs,
  std::vector<std::shared_ptr<tracker::Tracking>> trackings)
{
  for (auto t : trackings) {
    cv::Rect2d r = t->getTrackedRect();
    if (!t->isActive()) {
      RCUTILS_LOG_DEBUG("Tracked (Not detected) %s [%f %f %f %f] %.0f%%",
        t->getObjName().c_str(), r.x, r.y, r.width, r.height,
        t->getObjProbability() * 100);
    }
    object_analytics_msgs::msg::TrackedObject tobj;
    tobj.id = t->getTrackingId();
    tobj.object.object_name = t->getObjName();
    tobj.object.probability = t->getObjProbability();
    tobj.roi.x_offset = static_cast<int>(r.x);
    tobj.roi.y_offset = static_cast<int>(r.y);
    tobj.roi.width = static_cast<int>(r.width);
    tobj.roi.height = static_cast<int>(r.height);

    objs.tracked_objects.push_back(tobj);
    RCUTILS_LOG_DEBUG("Tracking publish %s [%f %f %f %f] %.0f%%",
      t->getObjName().c_str(), r.x, r.y, r.width, r.height,
      t->getObjProbability() * 100);
  }
}

}  // namespace object_analytics_node

RCLCPP_COMPONENTS_REGISTER_NODE(object_analytics_node::TrackingNode)
