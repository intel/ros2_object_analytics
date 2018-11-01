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

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <math.h>
#include <vector>
#include <string>
#include <memory>
#include "object_analytics_node/movement/moving_objects.hpp"
#include "object_analytics_msgs/msg/moving_object.hpp"
namespace object_analytics_node
{
namespace movement
{
MovingObjects::MovingObjects(const std::shared_ptr<Param> & param)
: params_(param)
{
  frames_.clear();
}

MovingObjects::~MovingObjects() {}

double MovingObjects::durationBtwFrames(
  MovingObjectFrame & first,
  MovingObjectFrame & second)
{
  rclcpp::Time t1(first.getStamp()), t2(second.getStamp());
  if (t1 == t2) {
    return 0.0;
  }
  rclcpp::Duration d = t1 - t2;
  return (static_cast<double>(d.nanoseconds())) / 1000000000;
}

void MovingObjects::calcVelocity(std::shared_ptr<MovingObjectFrame> & frame)
{
  unsigned int size_frames = frames_.size();

  if (size_frames < 2) { /**< Velocity calcaluation needs at least 2 frames. */
    return;
  }

  for (MovingObjectVector::iterator ob = frame->getMovingObjects().begin();
    ob != frame->getMovingObjects().end(); ++ob)
  {
    geometry_msgs::msg::Point sum_vel;
    int sum_count = 0;
    sum_vel.x = sum_vel.y = sum_vel.z = 0.0;

    /**< Find the latest objects from frames (in reverse order) */
    for (unsigned int i = size_frames; i > 0; --i) {
      double duration = durationBtwFrames(*frames_[i - 1], *frame);

      if (duration == 0.0) {  /// skip the frame to be calculated itself.
        continue;
      }

      MovingObject out;
      if (frames_[i - 1]->findMovingObjectByOverlap(ob->roi, out)) {
        geometry_msgs::msg::Point32 from = MovingObjectFrame::getCentroid(*ob);
        geometry_msgs::msg::Point32 to = MovingObjectFrame::getCentroid(out);

        /**< Align data's frame id and time. */
        geometry_msgs::msg::PointStamped pt, opt;
        pt.point.x = to.x;
        pt.point.y = to.y;
        pt.point.z = to.z;
        pt.header.frame_id = frame->getTfFrameId();
        pt.header.stamp = frames_[i - 1]->getStamp();
        /**< @todo, make better velocity calculation by using tf's transform. */
        // tf_.transformPoint(frame->getTfFrameId(), frame->getStamp(), pt,
        // fixed_frame_, opt);

        double distance_x = to.x - from.x;
        double distance_y = to.y - from.y;
        double distance_z = to.z - from.z;

        sum_vel.x += distance_x / duration;
        sum_vel.y += distance_y / duration;
        sum_vel.z += distance_z / duration;
        sum_count++;
        break;
      }
    }

    if (sum_count > 0) {
      if (abs(sum_vel.x) / sum_count < 0.2) {
        ob->velocity.x = 0;
      } else {
        ob->velocity.x = sum_vel.x / sum_count;
      }
      if (abs(sum_vel.y) / sum_count < 0.2) {
        ob->velocity.y = 0;
      } else {
        ob->velocity.y = sum_vel.y / sum_count;
      }
      if (abs(sum_vel.z) / sum_count < 0.2) {
        ob->velocity.z = 0;
      } else {
        ob->velocity.z = sum_vel.z / sum_count;
      }
    }
  }
}

void MovingObjects::clearOldFrames()
{
  int olds = frames_.size() - params_->max_frames_;

  if (olds > 0) {
    frames_.erase(frames_.begin(), frames_.begin() + olds);
  }
}

void MovingObjects::processFrame(
  const object_analytics_msgs::msg::ObjectsInBoxes3D::SharedPtr & loc,
  const rclcpp::Publisher<MovingObjectMsg>::SharedPtr & moving_objects_pub_)
{
  auto stamp = loc->header.stamp;
  std::string frame_id = loc->header.frame_id;

  std::shared_ptr<MovingObjectFrame> new_frame =
    std::make_shared<MovingObjectFrame>(stamp, frame_id, params_);

  /**< make sure old frames are already cleared first. */
  clearOldFrames();

  if (loc->objects_in_boxes.size() != 0) {
    new_frame->addVector(loc->objects_in_boxes);
    new_frame->mergeObjects();

    if (params_->velocity_enabled_) {
      calcVelocity(new_frame);
    }
  }

  frames_.push_back(new_frame);
  new_frame->publish(moving_objects_pub_);
}

std::shared_ptr<MovingObjectFrame> MovingObjects::getInstance(
  const builtin_interfaces::msg::Time stamp, const std::string frame_id)
{
  try {
    int size = frames_.size();
    for (int i = 0; i < size; ++i) {
      if (frames_[i]->getTfFrameId() == frame_id &&
        frames_[i]->getStamp() == stamp)
      {
        return frames_[i];
      }
    }

    std::shared_ptr<MovingObjectFrame> new_frame =
      std::make_shared<MovingObjectFrame>(stamp, frame_id, params_);
    frames_.push_back(new_frame);

    return new_frame;
  } catch (...) {
    std::shared_ptr<MovingObjectFrame> new_frame =
      std::make_shared<MovingObjectFrame>(stamp, frame_id, params_);
    frames_.push_back(new_frame);
    return new_frame;
  }
}

std::shared_ptr<MovingObjectFrame> MovingObjects::findObjectFrame(
  const builtin_interfaces::msg::Time stamp, const std::string frame_id)
{
  int size = frames_.size();

  for (int i = 0; i < size; ++i) {
    if (frames_[i]->getTfFrameId() == frame_id &&
      frames_[i]->getStamp() == stamp)
    {
      return frames_[i];
    }
  }

  return nullptr;
}

}  // namespace movement
}  // namespace object_analytics_node
