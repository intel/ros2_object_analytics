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

#ifndef OBJECT_ANALYTICS_NODE__SEGMENTER__SEGMENTER_HPP_
#define OBJECT_ANALYTICS_NODE__SEGMENTER__SEGMENTER_HPP_

#define PCL_NO_PRECOMPILE
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <object_msgs/msg/object_in_box.hpp>
#include <vector>
#include <memory>
#include "object_analytics_msgs/msg/objects_in_boxes3_d.hpp"
#include "object_analytics_node/model/object2d.hpp"
#include "object_analytics_node/model/object3d.hpp"
#include "object_analytics_node/segmenter/algorithm_provider.hpp"
#include "object_analytics_node/model/object_utils.hpp"

namespace object_analytics_node
{
namespace segmenter
{
using object_analytics_msgs::msg::ObjectsInBoxes3D;
using object_analytics_node::model::PointT;
using object_analytics_node::model::PointCloudT;
using object_analytics_node::segmenter::AlgorithmProvider;

/** @class Segmenter
 * Segmenter implmentation. Segment the coming point cloud into individual objects and publish
 * on segmentation topic.
 */
class Segmenter
{
public:
  using Object3DVector = std::vector<object_analytics_node::model::Object3D>;
  using Object2D = object_analytics_node::model::Object2D;
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
  void segment(
    const ObjectsInBoxes::ConstSharedPtr objs_2d,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points,
    ObjectsInBoxes3D::SharedPtr & msg);

  /**
   * @brief Set ROI cloud sampling step.
   *
   * @param[in]     step Sampling step to be used in get roi clouds.
   */
  void setSamplingStep(size_t step);

private:
  void getPclPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &, PointCloudT &);
  void getRoiPointCloud(
    const PointCloudT::ConstPtr & cloud, PointCloudT::Ptr & roi_cloud, const Object2D & obj2d);
  void getRoiPointCloud(
    const PointCloudT::ConstPtr & cloud, const pcl::PointCloud<PointXYZPixel>::Ptr & pixel_pcl,
    PointCloudT::Ptr & roi_cloud, const Object2D & obj2d);
  void getPixelPointCloud(
    const PointCloudT::ConstPtr & cloud, pcl::PointCloud<PointXYZPixel>::Ptr & pixel_pcl);
  void doSegment(
    const ObjectsInBoxes::ConstSharedPtr, const PointCloudT::ConstPtr &, RelationVector &);
  void composeResult(const RelationVector &, ObjectsInBoxes3D::SharedPtr &);

  std::unique_ptr<AlgorithmProvider> provider_;

  size_t sampling_step_ = 1;
};
}  // namespace segmenter
}  // namespace object_analytics_node
#endif  // OBJECT_ANALYTICS_NODE__SEGMENTER__SEGMENTER_HPP_
