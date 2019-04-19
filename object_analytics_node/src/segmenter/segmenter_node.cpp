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
#include "object_analytics_node/segmenter/segmenter_node.hpp"
#include "object_analytics_node/segmenter/algorithm_provider_impl.hpp"

namespace object_analytics_node
{
namespace segmenter
{
#define DEFAULT_SAMPLING  10
const int SegmenterNode::kMsgQueueSize = 100;
using object_analytics_node::segmenter::AlgorithmProvider;
using object_analytics_node::segmenter::AlgorithmProviderImpl;

SegmenterNode::SegmenterNode(rclcpp::NodeOptions options)
: Node("SegmenterNode", options)
{
  pub_ = create_publisher<object_analytics_msgs::msg::ObjectsInBoxes3D>(Const::kTopicLocalization);

  rclcpp::Node::SharedPtr node = std::shared_ptr<rclcpp::Node>(this);
  pcls = std::unique_ptr<Pcls>(new Pcls(node, Const::kTopicPC2));
  objs_2d = std::unique_ptr<Objs_2d>(new Objs_2d(node, Const::kTopicDetection));

  sub_sync_seg = std::unique_ptr<ApproximateSynchronizer>(
    new ApproximateSynchronizer(ApproximatePolicy(kMsgQueueSize), *objs_2d, *pcls));
  sub_sync_seg->registerCallback(
    std::bind(&SegmenterNode::callback, this, std::placeholders::_1, std::placeholders::_2));
  impl_.reset(new Segmenter(std::unique_ptr<AlgorithmProvider>(new AlgorithmProviderImpl())));
  impl_->setSamplingStep(DEFAULT_SAMPLING);
}

void SegmenterNode::callback(
  const ObjectsInBoxes::ConstSharedPtr objs_2d,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pcls)
{
  ObjectsInBoxes3D::SharedPtr msgs = std::make_shared<ObjectsInBoxes3D>();
  impl_->segment(objs_2d, pcls, msgs);
  pub_->publish(msgs);
}
}  // namespace segmenter
}  // namespace object_analytics_node

RCLCPP_COMPONENTS_REGISTER_NODE(object_analytics_node::segmenter::SegmenterNode)
