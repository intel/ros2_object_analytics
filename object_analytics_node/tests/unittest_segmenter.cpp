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
#include <cassert>
#include <vector>
#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>
#include "object_analytics_node/segmenter/segmenter.hpp"
#include "object_analytics_node/segmenter/algorithm.hpp"
#include "object_analytics_node/segmenter/algorithm_provider.hpp"
#include <object_msgs/msg/object_in_box.hpp>
#include "unittest_util.hpp"
#include "object_analytics_node/model/object2d.hpp"

using object_analytics_node::segmenter::Algorithm;
using object_analytics_node::segmenter::AlgorithmProvider;
using object_analytics_node::segmenter::Segmenter;
using object_msgs::msg::ObjectsInBoxes;
using object_analytics_msgs::msg::TrackedObject;
using object_analytics_msgs::msg::TrackedObjects;
class Algo : public Algorithm
{
public:
  Algo() = default;
  ~Algo() = default;

  void segment(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
               pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_segment, std::vector<pcl::PointIndices>& cluster_indices)
  {
    pcl::PointIndices indices;
    for (int i = 0; i < 15; i++)
    {
      indices.indices.push_back(i);
    }
    cluster_indices.push_back(indices);
    pcl::copyPointCloud(*cloud, *cloud_segment);
  }
};

class AlgoProvider : public AlgorithmProvider
{
public:
  virtual std::shared_ptr<Algorithm> get()
  {
    return algo_;
  }

  AlgoProvider() : algo_(std::make_shared<Algo>())
  {
  }

  ~AlgoProvider() = default;

private:
  std::shared_ptr<Algorithm> algo_;
};

TEST(UnitTestSegmenter, segmenter)
{
  PointCloudT::Ptr cloud(new PointCloudT);
  TrackedObject oib = getObjectInBox(0, 0, 100, 100, "table", 0.99);
  TrackedObjects::SharedPtr objects_in_boxes2d = std::make_shared<TrackedObjects>();
  std_msgs::msg::Header header2D = createHeader(builtin_interfaces::msg::Time(), "camera_rgb_optical_frame");
  objects_in_boxes2d->header = header2D;
  objects_in_boxes2d->tracked_objects.push_back(getObjectInBox(0, 0, 5, 5, "person", 0.99));
  objects_in_boxes2d->tracked_objects.push_back(getObjectInBox(6, 6, 5, 5, "person", 0.99));
  readPointCloudFromPCD(std::string(RESOURCE_DIR) + "/segment.pcd", cloud);

  sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*cloud, *cloudMsg);

  std::unique_ptr<Segmenter> impl;
  impl.reset(new Segmenter(std::unique_ptr<AlgoProvider>(new AlgoProvider())));
  std::shared_ptr<ObjectsInBoxes3D> obj3ds = std::make_shared<ObjectsInBoxes3D>();

  impl->segment(objects_in_boxes2d, cloudMsg, obj3ds);

  EXPECT_EQ(static_cast<size_t>(2), obj3ds->objects_in_boxes.size());
  ObjectInBox3D obj3d = obj3ds->objects_in_boxes[0];

  EXPECT_TRUE(obj3d.min == getPoint32(0.1, 0.2, 0.3));
  EXPECT_TRUE(obj3d.max == getPoint32(24.1, 24.2, 24.3));
  EXPECT_TRUE(obj3d.roi == getRoi(0, 0, 5, 5));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
