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

#ifndef OBJECT_ANALYTICS_NODE__MODEL__OBJECT3D_HPP_
#define OBJECT_ANALYTICS_NODE__MODEL__OBJECT3D_HPP_

#define PCL_NO_PRECOMPILE
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <geometry_msgs/msg/point32.h>

#include <vector>
#include <memory>

#include "object_analytics_msgs/msg/object_in_box3_d.hpp"

namespace object_analytics_node
{
namespace model
{
using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

/** @class Object3D
 * @brief Wrapper of object_analytics_msgs::msg::ObjectInBox3D.
 *
 * There are two scenarios of using this class. One is in SegmenterNode, this class is used to
 * hold the result of 3d segmentation and calculate minimum and maximum point in 3d space.
 * Another one is in MergerNode, it's build based
 * on segmentation result.
 */
class Object3D
{
public:
  /**
   * @brief Construct a 3D object based on PointCloud segmentation result.
   *
   * Use this constructor to build Object3D objects when 3D segmentation is done.
   *
   * @param[in] cloud       PointCloud got from RGB-D sensor
   * @param[in] indices     Indices vector, each is the indices of one segmentation object
   */
  Object3D(const PointCloudT::ConstPtr & cloud, const std::vector<int> & indices);

  /**
   * @brief Construct a 3D object based on results published by segmenter.
   *
   * Use this constructor in merger to build Object3D object.
   *
   * @param[in] object3d    Result published by segmenter
   */
  explicit Object3D(const object_analytics_msgs::msg::ObjectInBox3D & object3d);

  /** Default destructor */
  ~Object3D() = default;

  /**
   * Inline method. Get the region of interest in image space.
   *
   * @return Roi in image space
   */
  inline sensor_msgs::msg::RegionOfInterest getRoi() const
  {
    return roi_;
  }

  /**
   * Inline method. Get the minimum x, y and z of underlying object in 3d space.
   *
   * @return 3D point contains minimum x, y and z
   */
  inline geometry_msgs::msg::Point32 getMin() const
  {
    return min_;
  }

  /**
   * Inline method. Get the maximum x, y and z of underlying object in 3d space.
   *
   * @return 3D point contains maximum x, y and z
   */
  inline geometry_msgs::msg::Point32 getMax() const
  {
    return max_;
  }

  /**
   * Get the underlying object_msgs::Object.
   *
   * @return The underlying object_msgs::Object
   */
  inline object_msgs::msg::Object getObject() const
  {
    return object_;
  }

  /**
   * Inline method. Reset region of interest of the 3d object.
   *
   */
  inline void setRoi(const sensor_msgs::msg::RegionOfInterest & roi_new)
  {
    roi_ = roi_new;
  }

  /**
   * Overload operator << to dump information of underlying information.
   *
   * @param[in,out] os    Standard output stream
   * @param[in]     obj   Object to be dumped
   *
   * @return Standard output stream
   */

  friend std::ostream & operator<<(std::ostream & os, const Object3D & obj);

private:
  sensor_msgs::msg::RegionOfInterest roi_;
  geometry_msgs::msg::Point32 min_;
  geometry_msgs::msg::Point32 max_;
  object_msgs::msg::Object object_;
};

using Object3DPtr = std::shared_ptr<Object3D>;
using Object3DConstPtr = std::shared_ptr<Object3D const>;
}  // namespace model
}  // namespace object_analytics_node
#endif  // OBJECT_ANALYTICS_NODE__MODEL__OBJECT3D_HPP_
