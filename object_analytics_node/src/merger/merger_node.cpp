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
#include <functional>
#include "object_analytics_node/const.hpp"
#include "object_analytics_node/merger/merger_node.hpp"

namespace object_analytics_node
{
namespace merger
{
const int MergerNode::kMsgQueueSize = 100;

MergerNode::MergerNode() : Node("MergerNode")
{
  pub_result_ = create_publisher<ObjectsInBoxes3D>(Const::kTopicLocalization);

  sub_2d_ = std::unique_ptr<Subscriber2D>(new Subscriber2D(this, Const::kTopicDetection));
  sub_3d_ = std::unique_ptr<Subscriber3D>(new Subscriber3D(this, Const::kTopicSegmentation));
  sub_sync_ = std::unique_ptr<ApproximateSynchronizer2D3D>(
      new ApproximateSynchronizer2D3D(ApproximatePolicy(kMsgQueueSize), *sub_2d_, *sub_3d_));
  sub_sync_->registerCallback(std::bind(&MergerNode::callback, this, std::placeholders::_1, std::placeholders::_2));
}

void MergerNode::callback(const ObjectsInBoxes::ConstSharedPtr objects_in_boxes2d,
                          const ObjectsInBoxes3D::ConstSharedPtr objects_in_boxes3d)
{
  ObjectsInBoxes3D::SharedPtr msgs = Merger::merge(objects_in_boxes2d, objects_in_boxes3d);
  pub_result_->publish(msgs);
}

}  // namespace merger
}  // namespace object_analytics_node

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(object_analytics_node::merger::MergerNode, rclcpp::Node)
