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

#ifndef OBJECT_ANALYTICS_NODE__MOVEMENT__MOVING_OBJECT_NODE_HPP_
#define OBJECT_ANALYTICS_NODE__MOVEMENT__MOVING_OBJECT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <memory>

#include "object_analytics_msgs/msg/moving_object.hpp"
#include "object_analytics_node/movement/moving_object_node.hpp"
#include "object_analytics_node/movement/moving_objects.hpp"
#include "object_analytics_node/movement/param.hpp"
#include "object_analytics_node/visibility_control.h"

namespace object_analytics_node
{
namespace movement
{
/** @brief Merging camera related messages into one, with more info(currently
 * velocity) added. When a given message is received:
 *         1. The class searches the corresponding frame by the same frame_id
 * and stamp, if no, creates a new frame.
 *         2. Add the message to the frame which filtered out in step1.
 *         3. Publish the ready frame(s).
 *         4. Clean the frames which are not in the monitoring window.
 */
class MovementNode : public rclcpp::Node
{
public:
  OBJECT_ANALYTICS_NODE_PUBLIC MovementNode();

private:
  void onObjectsReceived(const LocalizationMsg::SharedPtr loc);
  std::shared_ptr<MovingObjects> frames_;
  std::shared_ptr<Param> params_;
  rclcpp::Subscription<LocalizationMsg>::SharedPtr sub_loc;
  rclcpp::Publisher<MovingObjectMsg>::SharedPtr moving_objects_pub_;
};

}  // namespace movement
}  // namespace object_analytics_node
#endif  // OBJECT_ANALYTICS_NODE__MOVEMENT__MOVING_OBJECT_NODE_HPP_
