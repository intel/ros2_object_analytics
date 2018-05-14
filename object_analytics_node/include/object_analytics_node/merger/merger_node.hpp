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

#ifndef OBJECT_ANALYTICS_NODE_MERGER_MERGER_NODE_H
#define OBJECT_ANALYTICS_NODE_MERGER_MERGER_NODE_H

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "object_analytics_node/visibility_control.h"
#include "object_analytics_node/merger/merger.hpp"

namespace object_analytics_node
{
namespace merger
{
/** @class MergerNode
 * Merger node, merger implementation holder.
 */
class MergerNode : public rclcpp::Node
{
public:
  OBJECT_ANALYTICS_NODE_PUBLIC MergerNode();

private:
  void callback(const ObjectsInBoxes::ConstSharedPtr objects_in_boxes2d,
                const ObjectsInBoxes3D::ConstSharedPtr objects_in_boxes3d);

  static const int kMsgQueueSize;

  using Subscriber2D = message_filters::Subscriber<ObjectsInBoxes>;
  using Subscriber3D = message_filters::Subscriber<ObjectsInBoxes3D>;
  using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<ObjectsInBoxes, ObjectsInBoxes3D>;
  using ApproximateSynchronizer2D3D = message_filters::Synchronizer<ApproximatePolicy>;

  rclcpp::Publisher<object_analytics_msgs::msg::ObjectsInBoxes3D>::SharedPtr pub_result_;
  std::unique_ptr<Subscriber2D> sub_2d_;
  std::unique_ptr<Subscriber3D> sub_3d_;
  std::unique_ptr<ApproximateSynchronizer2D3D> sub_sync_;
};
}  // namespace merger
}  // namespace object_analytics_node
#endif  // OBJECT_ANALYTICS_NODE_MERGER_MERGER_NODE_H
