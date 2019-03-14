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
#include <object_analytics_msgs/msg/objects_in_boxes3_d.hpp>
#include <object_analytics_msgs/msg/object_in_box3_d.hpp>
#include <object_msgs/msg/object_in_box.hpp>
#include <memory>
#include <vector>
#include <utility>
#include "object_analytics_node/segmenter/organized_multi_plane_segmenter.hpp"
#include "object_analytics_node/segmenter/segmenter.hpp"
#include "object_analytics_node/model/object_utils.hpp"
namespace object_analytics_node
{
namespace segmenter
{
using pcl::fromROSMsg;
using pcl::Label;
using pcl::Normal;
using pcl::PointIndices;
using pcl::IndicesPtr;
using object_analytics_node::model::Object3D;
using object_analytics_node::model::ObjectUtils;
using object_msgs::msg::ObjectsInBoxes;

Segmenter::Segmenter(std::unique_ptr<AlgorithmProvider> provider)
: provider_(std::move(provider))
{
}

void Segmenter::segment(
  const ObjectsInBoxes::ConstSharedPtr objs_2d,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points,
  ObjectsInBoxes3D::SharedPtr & msg)
{
  msg->header = objs_2d->header;
  PointCloudT::Ptr pointcloud(new PointCloudT);
  getPclPointCloud(points, *pointcloud);
  RelationVector relations;
  doSegment(objs_2d, pointcloud, relations);
  composeResult(relations, msg);
}


void Segmenter::setSamplingStep(size_t step)
{
  sampling_step_ = step;
}

void Segmenter::getPclPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points, PointCloudT & pcl_cloud)
{
  fromROSMsg<PointT>(*points, pcl_cloud);
}

void Segmenter::doSegment(
  const ObjectsInBoxes::ConstSharedPtr objs_2d,
  const PointCloudT::ConstPtr & cloud, RelationVector & relations)
{
  std::vector<PointIndices> cluster_indices_roi;
  PointCloudT::Ptr roi_cloud(new PointCloudT);
  std::vector<int> obj_points_indices;
  std::shared_ptr<Algorithm> seg = provider_->get();
  Object2DVector objects2d_vec;
  ObjectUtils::fill2DObjects(objs_2d, objects2d_vec);

  try {
    for (auto obj2d : objects2d_vec) {
      roi_cloud->clear();
      cluster_indices_roi.clear();
      obj_points_indices.clear();
      getRoiPointCloud(cloud, roi_cloud, obj2d);
      seg->segment(roi_cloud, cluster_indices_roi);
      for (auto & indices : cluster_indices_roi) {
        if (indices.indices.size() > obj_points_indices.size()) {
          obj_points_indices = indices.indices;
        }
      }
      if (obj_points_indices.size() > 0) {
        Object3D object3d_seg(roi_cloud, obj_points_indices);
        object3d_seg.setRoi(obj2d.getRoi());
        relations.push_back(Relation(obj2d, object3d_seg));
      }
    }
  } catch (std::exception & e) {
    std::cout << "std::exception: " << e.what() << std::endl;
  }
}

void Segmenter::composeResult(
  const RelationVector & relations, ObjectsInBoxes3D::SharedPtr & msgs)
{
  for (auto item : relations) {
    object_analytics_msgs::msg::ObjectInBox3D obj3d;
    obj3d.object = item.first.getObject();
    obj3d.roi = item.first.getRoi();
    obj3d.min = item.second.getMin();
    obj3d.max = item.second.getMax();
    msgs->objects_in_boxes.push_back(obj3d);
  }
}

void Segmenter::getRoiPointCloud(
  const PointCloudT::ConstPtr & cloud, PointCloudT::Ptr & roi_cloud, const Object2D & obj2d)
{
  auto obj2d_roi = obj2d.getRoi();
  std::vector<int> roi_indices;

  size_t x = obj2d_roi.x_offset;
  size_t y = obj2d_roi.y_offset;
  size_t x_end = x + obj2d_roi.width;
  size_t y_end = y + obj2d_roi.height;

  for (size_t idx_x = x; idx_x < x_end; idx_x += sampling_step_) {
    for (size_t idx_y = y; idx_y < y_end; idx_y += sampling_step_) {
      size_t idx = idx_x + idx_y * cloud->width;
      if (std::isfinite(cloud->points[idx].x)) {
        roi_indices.push_back(idx);
      }
    }
  }

  pcl::copyPointCloud(*cloud, roi_indices, *roi_cloud);
  roi_cloud->width = roi_indices.size();
  roi_cloud->height = 1;
}

void Segmenter::getRoiPointCloud(
  const PointCloudT::ConstPtr & cloud, const pcl::PointCloud<PointXYZPixel>::Ptr & pixel_pcl,
  PointCloudT::Ptr & roi_cloud, const Object2D & obj2d)
{
  auto obj2d_roi = obj2d.getRoi();
  std::vector<int> roi_indices;
  size_t x = obj2d_roi.x_offset;
  size_t y = obj2d_roi.y_offset;
  size_t x_ed = x + obj2d_roi.width;
  size_t y_ed = y + obj2d_roi.height;
  for (size_t i = 0; i < pixel_pcl->points.size(); i++) {
    if ((pixel_pcl->points[i].pixel_x >= x) && (pixel_pcl->points[i].pixel_x < x_ed) &&
      (pixel_pcl->points[i].pixel_y >= y) && (pixel_pcl->points[i].pixel_y < y_ed))
    {
      roi_indices.push_back(i);
    }
  }
  pcl::copyPointCloud(*cloud, roi_indices, *roi_cloud);
  roi_cloud->width = obj2d_roi.width;
  roi_cloud->height = obj2d_roi.height;
}

void Segmenter::getPixelPointCloud(
  const PointCloudT::ConstPtr & cloud, pcl::PointCloud<PointXYZPixel>::Ptr & pixel_pcl)
{
  std::vector<int> indices;
  for (size_t i = 0; i < cloud->points.size(); i++) {
    indices.push_back(i);
  }
  ObjectUtils::copyPointCloud(cloud, indices, pixel_pcl);
  pixel_pcl->height = cloud->height;
  pixel_pcl->width = cloud->width;
}

}  // namespace segmenter
}  // namespace object_analytics_node
