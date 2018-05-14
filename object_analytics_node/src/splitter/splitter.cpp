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
#define PCL_NO_PRECOMPILE
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include "object_analytics_node/model/object3d.hpp"
#include "object_analytics_node/splitter/splitter.hpp"

namespace object_analytics_node
{
namespace splitter
{
using object_analytics_node::model::PointT;

void Splitter::split(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& points,
                     sensor_msgs::msg::Image::SharedPtr& image)
{
  std_msgs::msg::Header header = points->header;
  pcl::toROSMsg(*points, *image);
  image->header = header;
}
}  // namespace splitter
}  // namespace object_analytics_node
