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
#include "object_analytics_node/const.hpp"
#include "object_analytics_node/segmenter/segmenter_node.hpp"
#include "object_analytics_node/segmenter/algorithm_provider_impl.hpp"

namespace object_analytics_node
{
namespace segmenter
{
using object_analytics_node::segmenter::AlgorithmProvider;
using object_analytics_node::segmenter::AlgorithmProviderImpl;

SegmenterNode::SegmenterNode() : Node("SegmenterNode")
{
  auto callback = [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr points) -> void {
    ObjectsInBoxes3D::SharedPtr msg = std::make_shared<ObjectsInBoxes3D>();
    msg->header = points->header;

    impl_->segment(points, msg);

    pub_->publish(msg);
  };

  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(Const::kTopicPC2, callback);
  pub_ = create_publisher<object_analytics_msgs::msg::ObjectsInBoxes3D>(Const::kTopicSegmentation);

  impl_.reset(new Segmenter(std::unique_ptr<AlgorithmProvider>(new AlgorithmProviderImpl())));
}

}  // namespace segmenter
}  // namespace object_analytics_node

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(object_analytics_node::segmenter::SegmenterNode, rclcpp::Node)
