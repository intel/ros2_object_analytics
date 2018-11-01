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

#include <class_loader/register_macro.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdio>
#include <memory>
#include <string>
#include "object_analytics_node/movement/moving_object_node.hpp"
#include "object_analytics_msgs/msg/moving_object.hpp"
#include "object_analytics_node/movement/param.hpp"
#include "object_analytics_node/const.hpp"

using std::placeholders::_1;

namespace object_analytics_node
{
namespace movement
{
MovementNode::MovementNode()
: Node("MovementNode")
{
  // Force flush of the stdout buffer.
  moving_objects_pub_ =
    create_publisher<MovingObjectMsg>(Const::kTopicMoving, 10);
  auto params_ = std::make_shared<object_analytics_node::movement::Param>();
  frames_ = std::make_shared<MovingObjects>(params_);
  sub_loc = this->create_subscription<LocalizationMsg>(
    Const::kTopicLocalization,
    std::bind(&MovementNode::onObjectsReceived, this, _1));
  RCLCPP_INFO(get_logger(), "...Creating Moving Objects buffer...");
}
void MovementNode::onObjectsReceived(const LocalizationMsg::SharedPtr loc)
{
  frames_->processFrame(loc, moving_objects_pub_);
}
}  // namespace movement
}  // namespace object_analytics_node

CLASS_LOADER_REGISTER_CLASS(object_analytics_node::movement::MovementNode,
  rclcpp::Node)
