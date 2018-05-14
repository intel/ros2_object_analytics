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

#ifndef OBJECT_ANALYTICS_NODE_MERGER_MERGER_H
#define OBJECT_ANALYTICS_NODE_MERGER_MERGER_H

#include <object_msgs/msg/objects_in_boxes.hpp>
#include <object_analytics_msgs/msg/objects_in_boxes3_d.hpp>
#include "object_analytics_node/model/object_utils.hpp"

namespace object_analytics_node
{
namespace merger
{
using object_analytics_msgs::msg::ObjectsInBoxes3D;

/** @class Merger
 * @brief Implementaion of merge logic.
 *
 * Merge 3d segmentation and 2d detection results together and publish the 3d localization result.
 * First using message header's timestamp, synchronize 3d segmentation result which is subscribed on "segmentation"
 * topic and 2d detection result which is subscribed on "detection" topic. Then find corresponding 3d segmentaion who
 * has the maximum overlap area in 2d for each 2d detection result. Finally publish the merged localization result.
 */
class Merger
{
public:
  /** Default constructor */
  Merger() = default;

  /** Default destructor */
  ~Merger() = default;

  /**
   * @brief Merge detection and segmentation results and return.
   *
   * @param[in] objects_in_boxes2d    Pointer to 2d detection result
   * @param[in] objects_in_boxes3d    Pointer to 3d segmentation result
   *
   * @return Shared pointer to ObjectsInBoxes3D type message
   */
  static std::shared_ptr<ObjectsInBoxes3D> merge(const ObjectsInBoxes::ConstSharedPtr& objects_in_boxes2d,
                                                 const ObjectsInBoxes3D::ConstSharedPtr& objects_in_boxes3d);

private:
  /**
   * Compose merged objects into ObjectsInBoxes3D message.
   *
   * @param[in]     relations   Pair list of 2d object and corresponding 3d object
   * @param[in,out] header      Pointer to ObjectsInBoxes3D message
   */
  static void composeResult(const RelationVector& relations, ObjectsInBoxes3D::SharedPtr& msgs);
};
}  // namespace merger
}  // namespace object_analytics_node
#endif  // OBJECT_ANALYTICS_NODE_MERGER_MERGER_H
