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

#ifndef OBJECT_ANALYTICS_NODE_SEGMENTER_SEGMENTER_NODE_H
#define OBJECT_ANALYTICS_NODE_SEGMENTER_SEGMENTER_NODE_H

#include <rclcpp/rclcpp.hpp>
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
  OBJECT_ANALYTICS_NODE_PUBLIC SegmenterNode();

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<object_analytics_msgs::msg::ObjectsInBoxes3D>::SharedPtr pub_;

  std::unique_ptr<Segmenter> impl_;
};
}  // namespace segmenter
}  // namespace object_analytics_node
#endif  // OBJECT_ANALYTICS_NODE_SEGMENTER_SEGMENTER_NODE_H
