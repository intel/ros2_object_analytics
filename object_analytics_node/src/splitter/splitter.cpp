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

#define PCL_NO_PRECOMPILE
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include "object_analytics_node/model/object3d.hpp"
#include "object_analytics_node/splitter/splitter.hpp"

namespace object_analytics_node
{
namespace splitter
{
using object_analytics_node::model::PointT;

void Splitter::split(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points,
  sensor_msgs::msg::Image::SharedPtr & image)
{
  std_msgs::msg::Header header = points->header;
  pcl::toROSMsg(*points, *image);
  image->header = header;
}

void
Splitter::splitPointsToXYZ(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointsXYZRGB,
  sensor_msgs::msg::PointCloud2::SharedPtr & pointsXYZ)
{
  pointsXYZ->header.stamp = pointsXYZRGB->header.stamp;
  pointsXYZ->header.frame_id = pointsXYZRGB->header.frame_id;
  pointsXYZ->width = pointsXYZRGB->width;
  pointsXYZ->height = pointsXYZRGB->height;
  pointsXYZ->is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(*pointsXYZ);

  modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> out_x(*pointsXYZ, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(*pointsXYZ, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(*pointsXYZ, "z");

  sensor_msgs::PointCloud2ConstIterator<float> in_x(*pointsXYZRGB, "x");
  sensor_msgs::PointCloud2ConstIterator<float> in_y(*pointsXYZRGB, "y");
  sensor_msgs::PointCloud2ConstIterator<float> in_z(*pointsXYZRGB, "z");

  for (size_t i = 0; i < pointsXYZ->height * pointsXYZ->width; ++i,
    ++out_x, ++out_y, ++out_z, ++in_x, ++in_y, ++in_z)
  {
    *out_x = *in_x;
    *out_y = *in_y;
    *out_z = *in_z;
  }
}

}  // namespace splitter
}  // namespace object_analytics_node
