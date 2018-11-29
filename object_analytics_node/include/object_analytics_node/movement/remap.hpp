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

#ifndef OBJECT_ANALYTICS_NODE__MOVEMENT__REMAP_HPP_
#define OBJECT_ANALYTICS_NODE__MOVEMENT__REMAP_HPP_

#include <object_analytics_msgs/msg/moving_object.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>

#include <vector>
#include <memory>
#include <string>

#include "object_analytics_msgs/msg/moving_objects_in_frame.hpp"
#include "object_analytics_msgs/msg/objects_in_boxes3_d.hpp"

namespace object_analytics_node
{
namespace movement
{
using DetectionObject = object_msgs::msg::Object;
using DetectionObjectInBox = object_msgs::msg::ObjectInBox;
using LocalizationObjectInBox = object_analytics_msgs::msg::ObjectInBox3D;
using MovingObject = object_analytics_msgs::msg::MovingObject;

using DetectionMsg = object_msgs::msg::ObjectsInBoxes;
using LocalizationMsg = object_analytics_msgs::msg::ObjectsInBoxes3D;
using MovingObjectMsg = object_analytics_msgs::msg::MovingObjectsInFrame;

using DetectionVector = std::vector<DetectionObjectInBox>;
using LocalizationVector = std::vector<LocalizationObjectInBox>;
using MovingObjectVector = std::vector<MovingObject>;

using ObjectRoi = sensor_msgs::msg::RegionOfInterest;

#define MO_VERBOSE
#ifdef MO_VERBOSE
#define MO_VERBOSE_INFO(args) RCLCPP_INFO(args)
#else
#define MO_VERBOSE_INFO(args)
#endif

/**< Posibility Threshold, only objects with equal or higher posibility pass to
 * MO.*/
constexpr double kObjectFiltering_PosibilityThreshold = 0.7;

/**< Enable or not type filtering. */
constexpr bool kObjectFiltering_TypeFiltering_Enabling = true;

/**< If type filtering enabled, only object with the listed types pass to MO. */
// constexpr char kObjectFiltering_TypeFiltering_Types[] = "person";
const std::vector<std::string> kObjectFiltering_TypeFiltering_Types = {
  "person"};

/**< The maximum number of frames to be archived in moving object node. */
constexpr int kFrameCache_Size = 8;

/**< Enable or not velocity calculation. */
constexpr bool kVelocityCalculation_Enabling = true;

/**< The frame name used for coordination transform during velocity calculation.
 */

}  // namespace movement
}  // namespace object_analytics_node
#endif  // OBJECT_ANALYTICS_NODE__MOVEMENT__REMAP_HPP_
