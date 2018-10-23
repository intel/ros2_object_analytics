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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>

#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <algorithm>

#include "object_analytics_msgs/msg/moving_object.hpp"
#include "object_analytics_node/movement/object_frame.hpp"
#include "object_analytics_msgs/msg/moving_objects_in_frame.hpp"
#include "object_analytics_node/movement/moving_object_node.hpp"
#include "object_analytics_node/movement/remap.hpp"
namespace object_analytics_node
{
namespace movement
{
MovingObjectFrame::MovingObjectFrame(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id,
  std::shared_ptr<Param> params)
: published_(true), params_(params)
{
  /**< @todo: configuring it by params for social_filter_ */
  social_filter_.clear();
  social_filter_ = kObjectFiltering_TypeFiltering_Types;

  objects_localized_.clear();

  moving_objects_.clear();
  stamp_ = stamp;
  tf_frame_id_ = frame_id;
}
MovingObjectFrame::~MovingObjectFrame() {}

void MovingObjectFrame::addVector(
  const std::vector<LocalizationObjectInBox> & vector)
{
  objects_localized_ = vector;
}

void MovingObjectFrame::mergeObjects()
{
  for (std::vector<LocalizationObjectInBox>::iterator it =
    objects_localized_.begin();
    it != objects_localized_.end(); ++it)
  {
    if (params_->social_filtering_enabled_ &&
      it->object.object_name != "person")
    {
      continue;
    }

    ObjectRoi roi = it->roi;
    MovingObject moving_obj;
    moving_obj.min = it->min;
    moving_obj.max = it->max;
    moving_obj.type = it->object.object_name;
    moving_obj.roi = roi;
    // Give a non-sense value as the original velocity.
    moving_obj.velocity.x = moving_obj.velocity.y = moving_obj.velocity.z = 0;
    moving_objects_.push_back(moving_obj);
  }  // end of for(...)
}

bool MovingObjectFrame::findMovingObjectById(const int id, MovingObject & out)
{
  MovingObjectVector temp_objects = moving_objects_;
  for (auto t : temp_objects) {
    if (t.id == id) {
      out = t;
      return true;
    }
  }
  return false;
}

bool MovingObjectFrame::findMovingObjectByOverlap(
  const ObjectRoi & roi,
  MovingObject & out)
{
  MovingObjectVector temp_objects = moving_objects_;
  float x1 = roi.x_offset;
  float y1 = roi.y_offset;
  float width1 = roi.width;
  float height1 = roi.height;
  for (auto t : temp_objects) {
    float x2 = t.roi.x_offset;
    float y2 = t.roi.y_offset;
    float width2 = t.roi.width;
    float height2 = t.roi.height;
    float endx = std::max(x1 + width1, x2 + width2);
    float startx = std::min(x1, x2);
    float width = width1 + width2 - (endx - startx);
    float endy = std::max(y1 + height1, y2 + height2);
    float starty = std::min(y1, y2);
    float height = height1 + height2 - (endy - starty);
    float overlap_ratio;
    if (width <= 0 || height <= 0) {
      overlap_ratio = 0;
    } else {
      overlap_ratio = width * height /
        (width1 * height1 + width2 * height2 - width * height);
    }
    if (overlap_ratio >= 0.3) {
      out = t;
      return true;
    }
  }
  return false;
}

bool MovingObjectFrame::findMovingObjectByRoi(
  const ObjectRoi & roi,
  MovingObject & out)
{
  MovingObjectVector temp_objects = moving_objects_;
  for (auto t : temp_objects) {
    if (roi.x_offset == t.roi.x_offset && roi.y_offset == t.roi.y_offset &&
      roi.width == t.roi.width && roi.height == t.roi.height)
    {
      out = t;
      return true;
    }
  }

  return false;
}

bool MovingObjectFrame::publish(
  const rclcpp::Publisher<MovingObjectMsg>::SharedPtr & moving_objects_pub_)
{
  if (params_->moving_object_msg_enabled_) {
    MovingObjectMsg::SharedPtr msg = std::make_shared<MovingObjectMsg>();
    msg->header.frame_id = tf_frame_id_;
    msg->header.stamp = stamp_;
    msg->objects = moving_objects_;
    if (moving_objects_.size() == 0) {
      return false;
    }
    moving_objects_pub_->publish(msg);
  }
  setFlagPublished(true);
  return true;
}

bool MovingObjectFrame::isSocialObject(DetectionObjectInBox & ob)
{
  for (auto f : social_filter_) {
    if (ob.object.object_name.find(f) != std::string::npos) {
      return true;
    }
  }

  return false;
}

void MovingObjectFrame::setFlagPublished(bool state) {published_ = state;}
}  // namespace movement
}  // namespace object_analytics_node
