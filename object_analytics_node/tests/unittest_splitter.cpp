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
#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <cassert>
#include <memory>

#include "object_analytics_node/splitter/splitter.hpp"
#include "unittest_util.hpp"

using object_analytics_node::splitter::Splitter;

TEST(UnitTestSplitter, split_Normal)
{
  PointCloudT::Ptr pclCloudOriginal(new PointCloudT);
  readPointCloudFromPCD(std::string(RESOURCE_DIR) + "/split.pcd", pclCloudOriginal);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  copyPointCloud(*pclCloudOriginal, *pclCloud);

  sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg =
    std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*pclCloud, *cloudMsg);

  sensor_msgs::msg::Image::SharedPtr imageMsg = std::make_shared<sensor_msgs::msg::Image>();
  Splitter::split(cloudMsg, imageMsg);

  EXPECT_EQ(cloudMsg->header.stamp, imageMsg->header.stamp);
  EXPECT_EQ(cloudMsg->header.frame_id, imageMsg->header.frame_id);

  EXPECT_EQ(cloudMsg->height, imageMsg->height);
  EXPECT_EQ(cloudMsg->width, imageMsg->width);
  EXPECT_EQ(cloudMsg->is_bigendian, imageMsg->is_bigendian);

  // TBD(Peter Han): Verify each data equals the orginal's
  /*
  for (uint32_t i = 0; i < cloudMsg->data.size(); ++i)
  {
    EXPECT_EQ(pclCloud[i].rgb, imageMsg->data[i]);
  }
  */
}

TEST(UNITTESTSplitter, split_EmptyInput)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg =
    std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*pclCloud, *cloudMsg);

  sensor_msgs::msg::Image::SharedPtr imageMsg = std::make_shared<sensor_msgs::msg::Image>();
  Splitter::split(cloudMsg, imageMsg);

  EXPECT_EQ(cloudMsg->header.stamp, imageMsg->header.stamp);
  EXPECT_EQ(cloudMsg->header.frame_id, imageMsg->header.frame_id);

  EXPECT_EQ(cloudMsg->height, imageMsg->height);
  EXPECT_EQ(cloudMsg->width, imageMsg->width);
  EXPECT_EQ(cloudMsg->is_bigendian, imageMsg->is_bigendian);

  EXPECT_EQ(imageMsg->data.size(), static_cast<size_t>(0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
