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

#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <object_analytics_msgs/msg/tracked_objects.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>
#include <string>
#include <memory>

#include "object_analytics_node/tracker/tracking.hpp"
#include "object_analytics_node/tracker/tracking_manager.hpp"
#include "object_analytics_node/visibility_control.h"

namespace object_analytics_node
{
/** @class TrackingNode
 * ROS Node of multiple trackings, each against one object detected across
 * camera frames.
 *
 * - Subscribe topics
 *   - /object_analytics/rgb. This class listen to "sensor_msgs::Image"
 * published by an RGBD camera, see @ref rgb_cb().
 *   - /object_analytics/detection. This class also listen to
 * "object_msgs::ObjectsInBoxes" published by object detection node, see @ref
 * obj_cb().
 * - Publish topic
 *   - /object_analytics/tracking. This class publish
 * "object_analytics_msgs::TrackedObjects", see @ref tracking_publish().
 *
 * The tracking workflow is initiated by a detection frame. Roi of each detected
 * object will be used to initialize a tracker for that object. Then the tracker
 * will be kept updated with each successive frames arrived, we calling them
 * tracking frames, till next detection frame arrives. So the rhythm of the
 * workflow is to repeat this sequence:
 *
 * [detection, tracking, tracking, tracking, ..., tracking]
 *
 * It pending on the frequency of detection, several tracking frames are
 * processed in between. The faster the tracker algorithm or the less objects to
 * track, the more tracking frames being processed before the next detection
 * frame arrives.
 *
 * TrackingNode simply buffers the RGB image in @ref rgb_cb(), and waiting for
 * the arrival of the detection frame. Then in @ref obj_cb(), TrackingNode noted
 * down this object frame and last object frame. The "last" object frame is used
 * to initiate trackings, then TrackingNode will process all successive frames
 * before "this" object frame.
 *
 * TrackingNode has a @ref TrackingManager to process tracking updates from both
 * detection frames and tracking frames.
 */
class TrackingNode : public rclcpp::Node
{
public:
  OBJECT_ANALYTICS_NODE_PUBLIC TrackingNode(rclcpp::NodeOptions options);

  /**
   * @brief Set tracker manager algorithm.
   */
  void setAlgo(std::string algo) {tm_->setAlgo(algo);}

  /**
   * @brief Get tracker manager algorithm in current use.
   */
  std::string getAlgo() {return tm_->getAlgo();}

private:
  /**
   * @brief Callback from the object detection.
   *
   * @param[in] objs List of objects detected in a detection frame.
   */
  void obj_cb(const object_msgs::msg::ObjectsInBoxes::ConstSharedPtr & objs);

  /**
   * @brief Callback from the rgb image.
   *
   * @param[in] img Image frame captured by camera.
   */
  void rgb_cb(const sensor_msgs::msg::Image::ConstSharedPtr & img);

  /**
   * @brief Publish tracked objects.
   *
   * @param[in] header Message header of the tracked objects.
   */
  void tracking_publish(const std_msgs::msg::Header & header);

  /**
   * @brief Publish predict objects.
   *
   * @param[in] header Message header of the predicted objects.
   */
  void predict_publish(const std_msgs::msg::Header & header);

  /**
   * @brief Fill track message with object information.
   *
   * @param[in] ROS tracking message.
   * @param[in] objs List of objects tracked in frame.
   */
  void fillTrackedObjsMsg(
    const object_analytics_msgs::msg::TrackedObjects::SharedPtr & objs,
    std::vector<std::shared_ptr<tracker::Tracking>> trackings);

  rclcpp::Publisher<object_analytics_msgs::msg::TrackedObjects>::SharedPtr
    pub_tracking_;   /**< Tracking publisher.*/
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
    sub_rgb_;   /**< Rgb image subscriber.*/
  rclcpp::Subscription<object_msgs::msg::ObjectsInBoxes>::SharedPtr
    sub_obj_;                           /**< Object detection subscriber.*/
  std::unique_ptr<tracker::TrackingManager> tm_; /**< TrackingManager*/
//  std::vector<sensor_msgs::msg::Image::ConstSharedPtr>
  std::vector<std::shared_ptr<sFrame>> rgbs_;     /**< Rgb image buffer.*/
  std::vector<Object> this_obj_;/**< Last detection frame, and this detection frame.*/
  struct timespec last_detection_,
    this_detection_;   /**< Timestamp of last and this detection frame.*/
  uint32_t kRgbQueueSize;
};

}  // namespace object_analytics_node
