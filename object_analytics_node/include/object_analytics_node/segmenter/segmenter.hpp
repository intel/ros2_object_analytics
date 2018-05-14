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

#ifndef OBJECT_ANALYTICS_NODE_SEGMENTER_SEGMENTER_H
#define OBJECT_ANALYTICS_NODE_SEGMENTER_SEGMENTER_H

#define PCL_NO_PRECOMPILE
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "object_analytics_msgs/msg/objects_in_boxes3_d.hpp"
#include "object_analytics_node/model/object2d.hpp"
#include "object_analytics_node/model/object3d.hpp"
#include "object_analytics_node/segmenter/algorithm_provider.hpp"

namespace object_analytics_node
{
namespace segmenter
{
using object_analytics_msgs::msg::ObjectsInBoxes3D;
using object_analytics_node::model::PointT;
using object_analytics_node::model::PointCloudT;
using object_analytics_node::segmenter::AlgorithmProvider;

/** @class Segmenter
 * Segmenter implmentation. Segment the coming point cloud into individual objects and publish on segmentation topic.
 */
class Segmenter
{
public:
  using Object3DVector = std::vector<object_analytics_node::model::Object3D>;

  /**
   * Constructor
   *
   * @param[in] provider Pointer to AlgorithmProvider instance
   */
  explicit Segmenter(std::unique_ptr<AlgorithmProvider> provider);

  /** Default desctructor */
  ~Segmenter() = default;

  /**
   * @brief Do segmentation on given PointCloud2 and return ObjectsInBoxes3D message back.
   *
   * @param[in]     points  Pointer point to PointCloud2 message from sensor.
   * @param[in,out] msg     Pointer pint to ObjectsInBoxes3D message to take back.
   */
  void segment(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& points, ObjectsInBoxes3D::SharedPtr& msg);

private:
  void getPclPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&, PointCloudT&);
  void doSegment(const PointCloudT::ConstPtr&, Object3DVector&);
  void composeResult(const Object3DVector&, ObjectsInBoxes3D::SharedPtr&);

  std::unique_ptr<AlgorithmProvider> provider_;
};
}  // namespace segmenter
}  // namespace object_analytics_node
#endif  // OBJECT_ANALYTICS_NODE_SEGMENTER_SEGMENTER_H
