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
#include <pcl/common/io.h>
#include <vector>
#include "object_analytics_node/model/object_utils.hpp"

namespace object_analytics_node
{
namespace model
{
void ObjectUtils::fill2DObjects(
  const ObjectsInBoxes::ConstSharedPtr & objects_in_boxes2d, Object2DVector & objects2d)
{
  for (auto item : objects_in_boxes2d->objects_vector) {
    Object2D object2d(item);
    objects2d.push_back(object2d);
  }
}

void ObjectUtils::fill3DObjects(
  const ObjectsInBoxes3D::ConstSharedPtr & objects_in_boxes3d, Object3DVector & objects3d)
{
  for (auto item : objects_in_boxes3d->objects_in_boxes) {
    Object3D object3d(item);
    objects3d.push_back(object3d);
  }
}

void ObjectUtils::findMaxIntersectionRelationships(
  const Object2DVector & objects2d, Object3DVector & objects3d, RelationVector & relations)
{
  if (objects2d.size() != objects3d.size()) {
    return;
  } else {
    for (size_t i = 0; i < objects2d.size(); i++) {
      relations.push_back(Relation(objects2d[i], objects3d[i]));
    }
  }
}

void ObjectUtils::getMinMaxPointsInX(
  const pcl::PointCloud<PointXYZPixel>::ConstPtr & point_cloud,
  PointXYZPixel & x_min, PointXYZPixel & x_max)
{
  auto cmp_x = [](PointXYZPixel const & l, PointXYZPixel const & r) {return l.x < r.x;};
  auto minmax_x = std::minmax_element(point_cloud->begin(), point_cloud->end(), cmp_x);
  x_min = *(minmax_x.first);
  x_max = *(minmax_x.second);
}

void ObjectUtils::getMinMaxPointsInY(
  const pcl::PointCloud<PointXYZPixel>::ConstPtr & point_cloud,
  PointXYZPixel & y_min, PointXYZPixel & y_max)
{
  auto cmp_y = [](PointXYZPixel const & l, PointXYZPixel const & r) {return l.y < r.y;};
  auto minmax_y = std::minmax_element(point_cloud->begin(), point_cloud->end(), cmp_y);
  y_min = *(minmax_y.first);
  y_max = *(minmax_y.second);
}

void ObjectUtils::getMinMaxPointsInZ(
  const pcl::PointCloud<PointXYZPixel>::ConstPtr & point_cloud,
  PointXYZPixel & z_min, PointXYZPixel & z_max)
{
  auto cmp_z = [](PointXYZPixel const & l, PointXYZPixel const & r) {return l.z < r.z;};
  auto minmax_z = std::minmax_element(point_cloud->begin(), point_cloud->end(), cmp_z);
  z_min = *(minmax_z.first);
  z_max = *(minmax_z.second);
}

void ObjectUtils::getProjectedROI(
  const pcl::PointCloud<PointXYZPixel>::ConstPtr & point_cloud,
  sensor_msgs::msg::RegionOfInterest & roi)
{
  auto cmp_x = [](PointXYZPixel const & l, PointXYZPixel const & r) {return l.pixel_x < r.pixel_x;};
  auto minmax_x = std::minmax_element(point_cloud->begin(), point_cloud->end(), cmp_x);
  roi.x_offset = minmax_x.first->pixel_x;
  auto max_x = minmax_x.second->pixel_x;
  roi.width = max_x - roi.x_offset;

  auto cmp_y = [](PointXYZPixel const & l, PointXYZPixel const & r) {return l.pixel_y < r.pixel_y;};
  auto minmax_y = std::minmax_element(point_cloud->begin(), point_cloud->end(), cmp_y);
  roi.y_offset = minmax_y.first->pixel_y;
  auto max_y = minmax_y.second->pixel_y;
  roi.height = max_y - roi.y_offset;
}

double ObjectUtils::getMatch(const cv::Rect2d & r1, const cv::Rect2d & r2)
{
  cv::Rect2i ir1(r1), ir2(r2);
  /* calculate center of rectangle #1*/
  cv::Point2i c1(ir1.x + (ir1.width >> 1), ir1.y + (ir1.height >> 1));
  /* calculate center of rectangle #2*/
  cv::Point2i c2(ir2.x + (ir2.width >> 1), ir2.y + (ir2.height >> 1));

  double a1 = ir1.area(), a2 = ir2.area(), a0 = (ir1 & ir2).area();
  /* calculate the overlap rate*/
  double overlap = a0 / (a1 + a2 - a0);
  /* calculate the deviation between centers #1 and #2*/
  double deviate = sqrt(powf((c1.x - c2.x), 2) + powf((c1.y - c2.y), 2));

  /* calculate the match rate. The more overlap, the more matching */
  return overlap * 100 / deviate;
}

void ObjectUtils::copyPointCloud(
  const PointCloudT::ConstPtr & original, const std::vector<int> & indices,
  pcl::PointCloud<PointXYZPixel>::Ptr & dest)
{
  pcl::copyPointCloud(*original, indices, *dest);
  uint32_t width = original->width;
  for (uint32_t i = 0; i < indices.size(); i++) {
    dest->points[i].pixel_x = indices[i] % width;
    dest->points[i].pixel_y = indices[i] / width;
  }
}

}  // namespace model
}  // namespace object_analytics_node
