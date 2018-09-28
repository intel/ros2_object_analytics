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

#ifndef OBJECT_ANALYTICS_NODE__SPLITTER__SPLITTER_HPP_
#define OBJECT_ANALYTICS_NODE__SPLITTER__SPLITTER_HPP_

#define PCL_NO_PRECOMPILE
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace object_analytics_node
{
namespace splitter
{
/** @class Splitter
 * @brief Implementaion of splitter logic.
 *
 * Subscrib PointCloud2 type topic which contains both 3d point cloud and rgb image,
 * separate image and 3d point cloud and re-publish.
 */
class Splitter
{
public:
  /** Default constructor */
  Splitter() = default;

  /** Default destructor */
  ~Splitter() = default;

  /**
   * @brief Split PointCloud2 w/ RGB into Image.
   *
   * param[in]      points  Pointer to PointCloud2 w/ RGB
   * param[in,out]  image   Pointer to Image
   */
  static void split(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points,
    sensor_msgs::msg::Image::SharedPtr & image);

  /**
   * @brief Split PointCloud2 w/ XYZRGB to XYZ.
   *
   * param[in]      points  Pointer to PointCloud2 w/ XYZRGB
   * param[out]     points  Pointer to PointCloud2 w/ XYZ
   */
  static void splitPointsToXYZ(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points,
    sensor_msgs::msg::PointCloud2::SharedPtr & points_xyz);
};
}  // namespace splitter
}  // namespace object_analytics_node
#endif  // OBJECT_ANALYTICS_NODE__SPLITTER__SPLITTER_HPP_
