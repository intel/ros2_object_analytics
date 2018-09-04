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

#include <vector>
#include <pcl/common/time.h>
#include <pcl/filters/impl/conditional_removal.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/search/impl/organized.hpp>
//#include <pcl/segmentation/impl/organized_connected_component_segmentation.hpp>
//#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <rcutils/logging_macros.h>
#include "object_analytics_node/const.hpp"
#include "object_analytics_node/segmenter/organized_multi_plane_segmenter.hpp"

namespace object_analytics_node
{
namespace segmenter
{
using pcl::Label;
using pcl::Normal;
using pcl::PointCloud;
using pcl::PointIndices;
using pcl::PlanarRegion;

OrganizedMultiPlaneSegmenter::OrganizedMultiPlaneSegmenter()
  : conf_(AlgorithmConfig())
  , plane_comparator_(new pcl::PlaneCoefficientComparator<PointT, Normal>)
  , euclidean_comparator_(new pcl::EuclideanPlaneCoefficientComparator<PointT, Normal>)
  , edge_aware_comparator_(new pcl::EdgeAwarePlaneComparator<PointT, Normal>)
  , euclidean_cluster_comparator_(new pcl::EuclideanClusterComparator<PointT, Normal, Label>)
{
  applyConfig();
}

void OrganizedMultiPlaneSegmenter::segment(const PointCloudT::ConstPtr& cloud, PointCloudT::Ptr& cloud_segment,
                                           std::vector<PointIndices>& cluster_indices)
{
  double start = pcl::getTime();
  RCUTILS_LOG_DEBUG("Total original point size = %d", cloud->size());

  pcl::copyPointCloud(*cloud, *cloud_segment);  // cloud_segment is same as cloud for this algorithm

  PointCloud<Normal>::Ptr normal_cloud(new PointCloud<Normal>);
  estimateNormal(cloud, normal_cloud);

  PointCloud<Label>::Ptr labels(new PointCloud<Label>);
  std::vector<PointIndices> label_indices;
  segmentPlanes(cloud, normal_cloud, labels, label_indices);
  segmentObjects(cloud, labels, label_indices, cluster_indices);

  double end = pcl::getTime();
  RCUTILS_LOG_DEBUG("Segmentation : %f", double(end - start));
}

void OrganizedMultiPlaneSegmenter::estimateNormal(const PointCloudT::ConstPtr& cloud,
                                                  PointCloud<Normal>::Ptr& normal_cloud)
{
  double start = pcl::getTime();

  pcl::copyPointCloud(*cloud, *normal_cloud);

  double end = pcl::getTime();
  RCUTILS_LOG_DEBUG("Calc normal : %f", double(end - start));
}

void OrganizedMultiPlaneSegmenter::segmentPlanes(const PointCloudT::ConstPtr& cloud,
                                                 const pcl::PointCloud<Normal>::Ptr& normal_cloud,
                                                 pcl::PointCloud<Label>::Ptr labels,
                                                 std::vector<PointIndices>& label_indices)
{
  double start = pcl::getTime();

  std::vector<PlanarRegion<PointT>, Eigen::aligned_allocator<PlanarRegion<PointT>>> regions;
  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<PointIndices> inlier_indices;
  std::vector<PointIndices> boundary_indices;
  plane_segmentation_.setInputNormals(normal_cloud);
  plane_segmentation_.setInputCloud(cloud);
  plane_segmentation_.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices,
                                       boundary_indices);

  double end = pcl::getTime();
  RCUTILS_LOG_DEBUG("Plane detection : %f", double(end - start));
}

void OrganizedMultiPlaneSegmenter::segmentObjects(const PointCloudT::ConstPtr& cloud, PointCloud<Label>::Ptr labels,
                                                  std::vector<PointIndices>& label_indices,
                                                  std::vector<PointIndices>& cluster_indices)
{
  double start = pcl::getTime();

  std::vector<bool> plane_labels;
  plane_labels.resize(label_indices.size(), false);
  for (size_t i = 0; i < label_indices.size(); i++)
  {
    if (label_indices[i].indices.size() > plane_minimum_points_)
    {
      plane_labels[i] = true;
    }
  }

  euclidean_cluster_comparator_->setInputCloud(cloud);
  euclidean_cluster_comparator_->setLabels(labels);
  euclidean_cluster_comparator_->setExcludeLabels(plane_labels);

  PointCloud<Label> euclidean_labels;
  pcl::OrganizedConnectedComponentSegmentation<PointT, Label> euclidean_segmentation(euclidean_cluster_comparator_);
  euclidean_segmentation.setInputCloud(cloud);
  euclidean_segmentation.segment(euclidean_labels, cluster_indices);

  auto func = [this](PointIndices indices) { return indices.indices.size() < this->object_minimum_points_; };
  cluster_indices.erase(std::remove_if(cluster_indices.begin(), cluster_indices.end(), func), cluster_indices.end());

  double end = pcl::getTime();
  RCUTILS_LOG_DEBUG("Cluster : %f", double(end - start));
}

void OrganizedMultiPlaneSegmenter::applyConfig()
{
  plane_minimum_points_ = conf_.get<size_t>("PLANE_MINIMUM_POINTS", 2000);
  object_minimum_points_ = conf_.get<size_t>("OBJECT_MINIMUM_POINTS", 200);

  normal_estimation_.setNormalEstimationMethod(normal_estimation_.SIMPLE_3D_GRADIENT);
  normal_estimation_.setNormalEstimationMethod(normal_estimation_.COVARIANCE_MATRIX);
  normal_estimation_.setMaxDepthChangeFactor(conf_.get<float>("NORMAL_MAX_DEPTH_CHANGE", 0.02f));
  normal_estimation_.setNormalSmoothingSize(conf_.get<float>("NORMAL_SMOOTH_SIZE", 30.0f));

  euclidean_cluster_comparator_->setDistanceThreshold(conf_.get<float>("EUCLIDEAN_DISTANCE_THRESHOLD", 0.02f), false);

  plane_segmentation_.setMinInliers(conf_.get<size_t>("MIN_PLANE_INLIERS", 1000));
  plane_segmentation_.setAngularThreshold(pcl::deg2rad(conf_.get<float>("NORMAL_ANGLE_THRESHOLD", 2.0f)));
  plane_segmentation_.setDistanceThreshold(conf_.get<float>("NORMAL_DISTANCE_THRESHOLD", 0.02f));

  const std::string comparator = conf_.get<std::string>("COMPARATOR", "PlaneCoefficientComparator");
  if (comparator == "PlaneCoefficientComparator")
  {
    plane_segmentation_.setComparator(plane_comparator_);
  }
  else if (comparator == "EuclideanPlaneCoefficientComparator")
  {
    plane_segmentation_.setComparator(euclidean_comparator_);
  }
  else if (comparator == "EdgeAwarePlaneComaprator")
  {
    plane_segmentation_.setComparator(edge_aware_comparator_);
  }
}
}  // namespace segmenter
}  // namespace object_analytics_node
