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

#ifndef OBJECT_ANALYTICS_NODE__SEGMENTER__SEGMENTER_NODE_HPP_
#define OBJECT_ANALYTICS_NODE__SEGMENTER__SEGMENTER_NODE_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>

#include "object_analytics_node/visibility_control.h"
#include "object_analytics_node/segmenter/segmenter.hpp"

namespace object_analytics_node
{
namespace segmenter
{
/** @class SegmenterNode
 * Segmenter node, segmenter implementation holder.
 */
class SegmenterNode : public rclcpp::Node
{
public:
  OBJECT_ANALYTICS_NODE_PUBLIC SegmenterNode(rclcpp::NodeOptions options);

private:
  void callback(
    const ObjectsInBoxes::ConstSharedPtr objs_2d,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr pcls);

  static const int kMsgQueueSize;

  using Objs_2d = message_filters::Subscriber<ObjectsInBoxes>;
  using Pcls = message_filters::Subscriber<sensor_msgs::msg::PointCloud2>;
  using ApproximatePolicy =
    message_filters::sync_policies::ApproximateTime<ObjectsInBoxes, sensor_msgs::msg::PointCloud2>;
  using ApproximateSynchronizer = message_filters::Synchronizer<ApproximatePolicy>;

  rclcpp::Publisher<object_analytics_msgs::msg::ObjectsInBoxes3D>::SharedPtr pub_;

  std::unique_ptr<Segmenter> impl_;
  std::unique_ptr<Objs_2d> objs_2d;
  std::unique_ptr<Pcls> pcls;
  std::unique_ptr<ApproximateSynchronizer> sub_sync_seg;
};
}  // namespace segmenter
}  // namespace object_analytics_node
#endif  // OBJECT_ANALYTICS_NODE__SEGMENTER__SEGMENTER_NODE_HPP_
