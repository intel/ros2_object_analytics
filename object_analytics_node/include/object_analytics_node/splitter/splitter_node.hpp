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

#ifndef OBJECT_ANALYTICS_NODE__SPLITTER__SPLITTER_NODE_HPP_
#define OBJECT_ANALYTICS_NODE__SPLITTER__SPLITTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "object_analytics_node/visibility_control.h"
#include "object_analytics_node/splitter/splitter.hpp"

namespace object_analytics_node
{
namespace splitter
{
/** @class SplitterNode
 * Splitter node, splitter implementation holder.
 */
class SplitterNode : public rclcpp::Node
{
public:
  OBJECT_ANALYTICS_NODE_PUBLIC SplitterNode(rclcpp::NodeOptions options);

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_2d_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_3d_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc2_;
};
}  // namespace splitter
}  // namespace object_analytics_node
#endif  // OBJECT_ANALYTICS_NODE__SPLITTER__SPLITTER_NODE_HPP_
