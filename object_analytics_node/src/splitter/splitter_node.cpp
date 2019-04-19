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
#include <rclcpp_components/register_node_macro.hpp>

#include <memory>
#include "object_analytics_node/const.hpp"
#include "object_analytics_node/splitter/splitter_node.hpp"

namespace object_analytics_node
{
namespace splitter
{
SplitterNode::SplitterNode(rclcpp::NodeOptions options)
: Node("SplitterNode", options)
{
  pub_2d_ = create_publisher<sensor_msgs::msg::Image>(Const::kTopicRgb);
  pub_3d_ = create_publisher<sensor_msgs::msg::PointCloud2>(Const::kTopicPC2);

  auto callback = [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr points) -> void {
      try {
        sensor_msgs::msg::Image::SharedPtr image = std::make_shared<sensor_msgs::msg::Image>();
        Splitter::split(points, image);
        pub_2d_->publish(image);

        sensor_msgs::msg::PointCloud2::SharedPtr pointsXYZ =
          std::make_shared<sensor_msgs::msg::PointCloud2>();
        Splitter::splitPointsToXYZ(points, pointsXYZ);
        pub_3d_->publish(pointsXYZ);
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(this->get_logger(),
          "caught exception %s while splitting, skip this message", e.what());
      }
    };
  sub_pc2_ =
    create_subscription<sensor_msgs::msg::PointCloud2>(Const::kTopicRegisteredPC2, callback);
}
}  // namespace splitter
}  // namespace object_analytics_node

RCLCPP_COMPONENTS_REGISTER_NODE(object_analytics_node::splitter::SplitterNode)
