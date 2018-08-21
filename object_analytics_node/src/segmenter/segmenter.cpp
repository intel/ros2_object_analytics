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
#include "object_analytics_node/model/object_utils.hpp"
// #include "object_analytics_node/model/object_utils.hpp"
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
  // doSegment1(objs_2d, pointcloud, objects);
  doSegment(objs_2d, pointcloud, objects);
  composeResult(objects, msg);
}

void Segmenter::getPclPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& points, PointCloudT& pcl_cloud)
{
  fromROSMsg<PointT>(*points, pcl_cloud);
}

void Segmenter::doSegment1(const ObjectsInBoxes::ConstSharedPtr objs_2d, const PointCloudT::ConstPtr& cloud, std::vector<Object3D>& objects)
{
  std::vector<PointIndices> cluster_indices;
  PointCloudT::Ptr cloud_segment(new PointCloudT);
  std::shared_ptr<Algorithm> seg = provider_->get();
  seg->segment(cloud, cloud_segment, cluster_indices);

  for (auto& indices : cluster_indices)
  {
    try
    {
      std::cout << "obj_size:" << indices.indices.size() << std::endl;
      Object3D object3d(cloud_segment, indices.indices);
      objects.push_back(object3d);
    }
    catch (std::exception& e)
    {
      // ROS_ERROR_STREAM(e.what());
    }
  }

  // ROS_DEBUG_STREAM("get " << objects.size() << " objects from segmentation");
}
void Segmenter::doSegment(const ObjectsInBoxes::ConstSharedPtr objs_2d, const PointCloudT::ConstPtr& cloud, std::vector<Object3D>& objects)
{


  PointCloudT::Ptr cloud_segment(new PointCloudT);
  std::vector<PointIndices> cluster_indices_roi;
  std::shared_ptr<Algorithm> seg = provider_->get();
  PointCloudT::Ptr tmp_cloud(new PointCloudT);
  std::vector<int> true_obj_idx;
  std::vector<int> obj_points_indices;




  pcl::PointCloud<PointXYZPixel>::Ptr pixel_pcl(new pcl::PointCloud<PointXYZPixel>);

  std::vector<int> ind_all;
  for (int i=0;i<cloud->points.size()-1;i++)
  {
    ind_all.push_back(i);
  }
  ObjectUtils::copyPointCloud(cloud, ind_all, pixel_pcl);
  pixel_pcl->height = 480;
  pixel_pcl->width = 640;
  Object2DVector objects2d_vec;
  ObjectUtils::fill2DObjects(objs_2d, objects2d_vec);
  std::vector<int> tmp_point;
  std::vector<std::vector<int>> cluster_indices;
  int x, y, bg, ed, x_ed, y_ed;
  int width = 480;
  for (auto obj2d : objects2d_vec)
  {
    
    tmp_cloud->clear();
    cloud_segment->clear();
    cluster_indices_roi.clear();
    tmp_point.clear();
    true_obj_idx.clear(); 
    obj_points_indices.clear();

    auto obj2d_roi = obj2d.getRoi();
    x = obj2d_roi.x_offset;
    y = obj2d_roi.y_offset;
    
    x_ed = x + obj2d_roi.width;
    y_ed = y + obj2d_roi.height;


    // bg = y*640  + x;
    // ed = (y+obj2d_roi.height)*640+x+obj2d_roi.width;

    // std::cout << x << " " << y << " " << bg << " " << ed << std::endl;



    for (int i=0; i<pixel_pcl->points.size()-1; i++)
    {
      if ((pixel_pcl->points[i].pixel_x >= x) && (pixel_pcl->points[i].pixel_x < x_ed) &&(pixel_pcl->points[i].pixel_y >= y) && (pixel_pcl->points[i].pixel_y < y_ed))
      {
        tmp_point.push_back(i);
      }
    }


    // for (int i=0;i<640*480;i++)
    // {
    //   tmp_point.push_back(i);
    // }
    //getRoiPointCloud(cloud, tmp_cloud, tmp_point, 640, 480); 
    pcl::copyPointCloud(*cloud, tmp_point, *tmp_cloud);
    tmp_cloud->width = obj2d_roi.width;
    tmp_cloud->height = obj2d_roi.height;
    std::cout << "size: " << tmp_cloud->points.size() << std::endl; 
    std::cout << "cpd f: " <<tmp_cloud->points[0] << std::endl;
    std::cout << "cpd l: " <<tmp_cloud->points[obj2d_roi.width*obj2d_roi.height-1] << std::endl;


    seg->segment(tmp_cloud, cloud_segment, cluster_indices_roi);
    std::cout << "cluster size: " << cluster_indices_roi.size() << std::endl;
    for (auto& indices:cluster_indices_roi)
    {
      std::cout << indices.indices.size()<< std::endl;
      if (indices.indices.size()>obj_points_indices.size())
      {
        obj_points_indices = indices.indices;
      }
    }

    std::cout << "roi_points: " << tmp_point.size() <<std::endl;  
    std::cout << "obj_points:  " << obj_points_indices.size() << std::endl;
    // cluster_indices.push_back(true_obj_idx);
    Object3D object3d_seg(tmp_cloud,obj_points_indices);    
    object3d_seg.roi_ = obj2d_roi; 
    // Object3D object3d(cloud, tmp_point);
    // object3d.min_.x = object3d_seg.min_.x;
    // object3d.min_.y = object3d_seg.min_.y;
    // object3d.min_.z = object3d_seg.min_.z;
    // object3d.max_.x = object3d_seg.max_.x;
    // object3d.max_.y = object3d_seg.max_.y;
    // object3d.max_.z = object3d_seg.max_.z;
    objects.push_back(object3d_seg);
  } 
  
}

void Segmenter::composeResult(const std::vector<Object3D>& objects, ObjectsInBoxes3D::SharedPtr& msg)
{
  for (auto& obj : objects)
  {
    object_analytics_msgs::msg::ObjectInBox3D oib3;
    oib3.min = obj.getMin();
    // std::cout << "min_z"<<oib3.min.x << std::endl;
    oib3.max = obj.getMax();
    // std::cout << "max_z"<<oib3.max.z << std::endl;
    oib3.roi = obj.getRoi();
    msg->objects_in_boxes.push_back(oib3);
  }

  // ROS_DEBUG_STREAM("segmenter publish message with " << objects.size() << " objects");
}
void Segmenter::getRoiPointCloud(const PointCloudT::ConstPtr& cloud, PointCloudT::Ptr& tmp_cloud, const std::vector<int> tmp_point, int width, int height)
{
  for (int i= 0;i < tmp_point.size();i++)
  {
    tmp_cloud->push_back(cloud->points[tmp_point[i]]);
  }
  tmp_cloud->height = height;
  tmp_cloud->width = width;
  std::cout << "get roi area Done!" << std::endl;
}

void Segmenter::recoverPointCloud(std::vector<int> obj_points_indices, std::vector<int> true_obj_idx, int bg)
{

  for (int i=0;i<obj_points_indices.size();i++)
  {
    std::cout<<i<<std::endl;
    true_obj_idx.push_back(bg+ obj_points_indices[i]);
  }
}
}  // namespace segmenter
}  // namespace object_analytics_node
