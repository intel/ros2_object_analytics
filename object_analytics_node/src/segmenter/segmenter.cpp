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
#include <memory>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <object_analytics_msgs/msg/objects_in_boxes3_d.hpp>
#include <object_analytics_msgs/msg/object_in_box3_d.hpp>
#include "object_analytics_node/segmenter/organized_multi_plane_segmenter.hpp"
#include "object_analytics_node/segmenter/segmenter.hpp"
#include <object_msgs/msg/object_in_box.hpp>
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

Segmenter::Segmenter(std::unique_ptr<AlgorithmProvider> provider) : provider_(std::move(provider))
{
}

void Segmenter::segment(const ObjectsInBoxes::ConstSharedPtr objs_2d, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& points, ObjectsInBoxes3D::SharedPtr& msg)
{
  msg->header = objs_2d->header;
  PointCloudT::Ptr pointcloud(new PointCloudT);
  getPclPointCloud(points, *pointcloud);

  std::vector<Object3D> objects;
  doSegment(objs_2d, pointcloud, objects);

  composeResult(objects, msg);
}

void Segmenter::getPclPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& points, PointCloudT& pcl_cloud)
{
  fromROSMsg<PointT>(*points, pcl_cloud);
}

void Segmenter::doSegment(const ObjectsInBoxes::ConstSharedPtr objs_2d, const PointCloudT::ConstPtr& cloud, std::vector<Object3D>& objects)
{
  pcl::PointCloud<PointXYZPixel>::Ptr pixel_pcl(new pcl::PointCloud<PointXYZPixel>);
  PointCloudT::Ptr cloud_segment(new PointCloudT);
  std::vector<PointIndices> cluster_indices_roi;
  PointCloudT::Ptr roi_cloud(new PointCloudT);
  std::vector<int> obj_points_indices;
  Segmenter::getPixelPointCloud(cloud, pixel_pcl);
  std::shared_ptr<Algorithm> seg = provider_->get();
  Object2DVector objects2d_vec;
  ObjectUtils::fill2DObjects(objs_2d, objects2d_vec);
  try
  {
    for (auto obj2d : objects2d_vec)
    {

      roi_cloud->clear();
      cloud_segment->clear();
      cluster_indices_roi.clear();
      obj_points_indices.clear();
      
      getRoiPointCloud(cloud, pixel_pcl, roi_cloud, obj2d);
      seg->segment(roi_cloud, cloud_segment, cluster_indices_roi);
      for (auto& indices:cluster_indices_roi)
      {
        if (indices.indices.size()>obj_points_indices.size())
        {
          obj_points_indices = indices.indices;
        }
      } 
        if (obj_points_indices.size()>0)
        {
        Object3D object3d_seg(roi_cloud,obj_points_indices);  
        object3d_seg.setRoi(obj2d.getRoi()); 
        objects.push_back(object3d_seg);
        }
      }
    } 
      catch (std::exception& e)
    {
       //ROS_INFO(e.what());
    }

  // ROS_DEBUG_STREAM("get " << objects.size() << " objects from segmentation");
}

void Segmenter::composeResult(const std::vector<Object3D>& objects, ObjectsInBoxes3D::SharedPtr& msg)
{
  for (auto& obj : objects)
  {
    object_analytics_msgs::msg::ObjectInBox3D oib3;
    oib3.min = obj.getMin();
    oib3.max = obj.getMax();
    oib3.roi = obj.getRoi();
    msg->objects_in_boxes.push_back(oib3);
  }

  // ROS_DEBUG_STREAM("segmenter publish message with " << objects.size() << " objects");
}
void Segmenter::getRoiPointCloud(const PointCloudT::ConstPtr& cloud, const pcl::PointCloud<PointXYZPixel>::Ptr& pixel_pcl, PointCloudT::Ptr& roi_cloud, const Object2D& obj2d)
{
  auto obj2d_roi = obj2d.getRoi();
  std::vector<int> roi_indices;
  size_t x = obj2d_roi.x_offset;
  size_t y = obj2d_roi.y_offset;
  size_t x_ed = x + obj2d_roi.width;
  size_t y_ed = y + obj2d_roi.height;
  for (size_t i=0; i<pixel_pcl->points.size(); i++)
  {
    if ((pixel_pcl->points[i].pixel_x >= x) && (pixel_pcl->points[i].pixel_x < x_ed) &&(pixel_pcl->points[i].pixel_y >= y) && (pixel_pcl->points[i].pixel_y < y_ed))
    {
      roi_indices.push_back(i);
    }
  }
  pcl::copyPointCloud(*cloud, roi_indices, *roi_cloud);
  roi_cloud->width = obj2d_roi.width;
  roi_cloud->height = obj2d_roi.height;
}

void Segmenter::getPixelPointCloud(const PointCloudT::ConstPtr& cloud, pcl::PointCloud<PointXYZPixel>::Ptr& pixel_pcl)
{
  std::vector<int> indices;
  
  for (size_t i=0; i<cloud->points.size();i++)
  {
    indices.push_back(i);
  }
  ObjectUtils::copyPointCloud(cloud, indices, pixel_pcl);
  pixel_pcl->height = cloud->height;
  pixel_pcl->width = cloud->width;
}

}  // namespace segmenter
}  // namespace object_analytics_node
