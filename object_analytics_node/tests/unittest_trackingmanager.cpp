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
#include <omp.h>
#include <gtest/gtest.h>
#include <cv_bridge/cv_bridge.h>

#include <cfloat>
#include <string>
#include <vector>
#include <cassert>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "object_analytics_node/tracker/tracking_manager.hpp"
#include "object_analytics_node/tracker/tracking.hpp"
#include "unittest_util.hpp"

TEST(UnitTestTracking_Manager, getTrackedObjs_FirstWithin)
{
  object_msgs::msg::ObjectsInBoxes::SharedPtr objs = std::make_shared<ObjectsInBoxes>();
  objs->objects_vector.clear();
  ObjectInBox first = getObjectInBox(50, 50, 100, 100, "person", 0.8f);
  objs->objects_vector.push_back(first);
  ObjectInBox second = getObjectInBox(100, 100, 50, 50, "chair", 0.9f);
  objs->objects_vector.push_back(second);
  ObjectInBox third = getObjectInBox(320, 480, 1, 1, "key", 0.1f);
  objs->objects_vector.push_back(third);
  EXPECT_EQ(objs->objects_vector.size(), static_cast<size_t>(3));

  cv::Mat mat(320, 480, CV_8UC3);
  EXPECT_FALSE(mat.empty());
  EXPECT_EQ(mat.rows, 320);
  EXPECT_EQ(mat.cols, 480);
  rclcpp::Node node("test1");
  object_analytics_node::tracker::TrackingManager tr(&node);

  tr.detect(mat, objs);
  EXPECT_EQ(objs->objects_vector.size(), static_cast<size_t>(3));
  object_analytics_msgs::msg::TrackedObjects::SharedPtr msg =
    std::make_shared<object_analytics_msgs::msg::TrackedObjects>();
  EXPECT_EQ(tr.getTrackedObjs(msg), 2);
  EXPECT_EQ(msg->tracked_objects[0].id, 0);
  EXPECT_EQ(msg->tracked_objects[1].id, 1);
  if (msg->tracked_objects[0].roi.x_offset == 50) {
    EXPECT_EQ(msg->tracked_objects[0].roi.x_offset, static_cast<size_t>(50));
    EXPECT_EQ(msg->tracked_objects[0].roi.y_offset, static_cast<size_t>(50));
    EXPECT_EQ(msg->tracked_objects[0].roi.width, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[0].roi.height, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[1].roi.x_offset, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[1].roi.y_offset, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[1].roi.width, static_cast<size_t>(50));
    EXPECT_EQ(msg->tracked_objects[1].roi.height, static_cast<size_t>(50));
  } else if (msg->tracked_objects[0].roi.x_offset == 100) {
    EXPECT_EQ(msg->tracked_objects[0].roi.x_offset, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[0].roi.y_offset, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[0].roi.width, static_cast<size_t>(50));
    EXPECT_EQ(msg->tracked_objects[0].roi.height, static_cast<size_t>(50));
    EXPECT_EQ(msg->tracked_objects[1].roi.x_offset, static_cast<size_t>(50));
    EXPECT_EQ(msg->tracked_objects[1].roi.y_offset, static_cast<size_t>(50));
    EXPECT_EQ(msg->tracked_objects[1].roi.width, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[1].roi.height, static_cast<size_t>(100));
  }
}

TEST(UnitTestTracking_Manager, getTrackedObjs_FirstWithout)
{
  object_msgs::msg::ObjectsInBoxes::SharedPtr objs = std::make_shared<ObjectsInBoxes>();
  objs->objects_vector.clear();
  ObjectInBox first = getObjectInBox(400, 50, 600, 200, "person", 0.8f);
  objs->objects_vector.push_back(first);
  ObjectInBox second = getObjectInBox(100, 100, 50, 50, "chair", 0.9f);
  objs->objects_vector.push_back(second);
  ObjectInBox third = getObjectInBox(320, 480, 1, 1, "key", 0.1f);
  objs->objects_vector.push_back(third);
  EXPECT_EQ(objs->objects_vector.size(), static_cast<size_t>(3));

  cv::Mat mat(320, 480, CV_8UC3);
  EXPECT_FALSE(mat.empty());
  EXPECT_EQ(mat.rows, 320);
  EXPECT_EQ(mat.cols, 480);
  rclcpp::Node node("test2");
  object_analytics_node::tracker::TrackingManager tr(&node);

  tr.detect(mat, objs);
  EXPECT_EQ(objs->objects_vector.size(), static_cast<size_t>(3));
  object_analytics_msgs::msg::TrackedObjects::SharedPtr msg =
    std::make_shared<object_analytics_msgs::msg::TrackedObjects>();
  EXPECT_EQ(tr.getTrackedObjs(msg), 2);
  EXPECT_EQ(msg->tracked_objects[0].id, 2);
  EXPECT_EQ(msg->tracked_objects[1].id, 3);
  if (msg->tracked_objects[0].roi.x_offset == 100) {
    EXPECT_EQ(msg->tracked_objects[0].roi.x_offset, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[0].roi.y_offset, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[0].roi.width, static_cast<size_t>(50));
    EXPECT_EQ(msg->tracked_objects[0].roi.height, static_cast<size_t>(50));
    EXPECT_EQ(msg->tracked_objects[1].roi.x_offset, static_cast<size_t>(400));
    EXPECT_EQ(msg->tracked_objects[1].roi.y_offset, static_cast<size_t>(50));
    EXPECT_EQ(msg->tracked_objects[1].roi.width, static_cast<size_t>(80));
    EXPECT_EQ(msg->tracked_objects[1].roi.height, static_cast<size_t>(200));
  } else if (msg->tracked_objects[0].roi.x_offset == 400) {
    EXPECT_EQ(msg->tracked_objects[0].roi.x_offset, static_cast<size_t>(400));
    EXPECT_EQ(msg->tracked_objects[0].roi.y_offset, static_cast<size_t>(50));
    EXPECT_EQ(msg->tracked_objects[0].roi.width, static_cast<size_t>(80));
    EXPECT_EQ(msg->tracked_objects[0].roi.height, static_cast<size_t>(200));
    EXPECT_EQ(msg->tracked_objects[1].roi.x_offset, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[1].roi.y_offset, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[1].roi.width, static_cast<size_t>(50));
    EXPECT_EQ(msg->tracked_objects[1].roi.height, static_cast<size_t>(50));
  }
}

TEST(UnitTestTracking_Manager, getTrackedObjs_FirstPartialWithin_OtherWithin)
{
  object_msgs::msg::ObjectsInBoxes::SharedPtr objs = std::make_shared<ObjectsInBoxes>();
  objs->objects_vector.clear();
  ObjectInBox first = getObjectInBox(200, 100, 300, 100, "person", 0.8f);
  objs->objects_vector.push_back(first);
  ObjectInBox second = getObjectInBox(50, 100, 240, 100, "chair", 0.9f);
  objs->objects_vector.push_back(second);
  ObjectInBox third = getObjectInBox(320, 480, 1, 1, "key", 0.1f);
  objs->objects_vector.push_back(third);
  EXPECT_EQ(objs->objects_vector.size(), static_cast<size_t>(3));

  cv::Mat mat(320, 480, CV_8UC3);
  EXPECT_FALSE(mat.empty());
  EXPECT_EQ(mat.rows, 320);
  EXPECT_EQ(mat.cols, 480);
  rclcpp::Node node("test3");
  object_analytics_node::tracker::TrackingManager tr(&node);

  tr.detect(mat, objs);
  EXPECT_EQ(objs->objects_vector.size(), static_cast<size_t>(3));
  object_analytics_msgs::msg::TrackedObjects::SharedPtr msg =
    std::make_shared<object_analytics_msgs::msg::TrackedObjects>();
  EXPECT_EQ(tr.getTrackedObjs(msg), 2);
  EXPECT_EQ(msg->tracked_objects[0].id, 4);
  EXPECT_EQ(msg->tracked_objects[1].id, 5);
  if (msg->tracked_objects[0].roi.x_offset == 50) {
    EXPECT_EQ(msg->tracked_objects[0].roi.x_offset, static_cast<size_t>(50));
    EXPECT_EQ(msg->tracked_objects[0].roi.y_offset, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[0].roi.width, static_cast<size_t>(240));
    EXPECT_EQ(msg->tracked_objects[0].roi.height, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[1].roi.x_offset, static_cast<size_t>(200));
    EXPECT_EQ(msg->tracked_objects[1].roi.y_offset, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[1].roi.width, static_cast<size_t>(280));
    EXPECT_EQ(msg->tracked_objects[1].roi.height, static_cast<size_t>(100));
  } else if (msg->tracked_objects[0].roi.x_offset == 200) {
    EXPECT_EQ(msg->tracked_objects[0].roi.x_offset, static_cast<size_t>(200));
    EXPECT_EQ(msg->tracked_objects[0].roi.y_offset, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[0].roi.width, static_cast<size_t>(280));
    EXPECT_EQ(msg->tracked_objects[0].roi.height, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[1].roi.x_offset, static_cast<size_t>(50));
    EXPECT_EQ(msg->tracked_objects[1].roi.y_offset, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[1].roi.width, static_cast<size_t>(240));
    EXPECT_EQ(msg->tracked_objects[1].roi.height, static_cast<size_t>(100));
  }
}

TEST(UnitTestTracking_Manager, getTrackedObjs_FirstPartialWithin_OtherPartialWithin)
{
  object_msgs::msg::ObjectsInBoxes::SharedPtr objs = std::make_shared<ObjectsInBoxes>();
  objs->objects_vector.clear();
  ObjectInBox first = getObjectInBox(200, 100, 300, 100, "person", 0.8f);
  objs->objects_vector.push_back(first);
  ObjectInBox second = getObjectInBox(50, 100, 540, 100, "chair", 0.9f);
  objs->objects_vector.push_back(second);
  ObjectInBox third = getObjectInBox(320, 480, 1, 1, "key", 0.1f);
  objs->objects_vector.push_back(third);
  EXPECT_EQ(objs->objects_vector.size(), static_cast<size_t>(3));

  cv::Mat mat(320, 480, CV_8UC3);
  EXPECT_FALSE(mat.empty());
  EXPECT_EQ(mat.rows, 320);
  EXPECT_EQ(mat.cols, 480);
  rclcpp::Node node("test4");
  object_analytics_node::tracker::TrackingManager tr(&node);

  tr.detect(mat, objs);
  EXPECT_EQ(objs->objects_vector.size(), static_cast<size_t>(3));
  object_analytics_msgs::msg::TrackedObjects::SharedPtr msg =
    std::make_shared<object_analytics_msgs::msg::TrackedObjects>();
  EXPECT_EQ(tr.getTrackedObjs(msg), 2);
  EXPECT_EQ(msg->tracked_objects[0].id, 6);
  EXPECT_EQ(msg->tracked_objects[1].id, 7);
  if (msg->tracked_objects[0].roi.x_offset == 50) {
    EXPECT_EQ(msg->tracked_objects[0].roi.x_offset, static_cast<size_t>(50));
    EXPECT_EQ(msg->tracked_objects[0].roi.y_offset, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[0].roi.width, static_cast<size_t>(430));
    EXPECT_EQ(msg->tracked_objects[0].roi.height, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[1].roi.x_offset, static_cast<size_t>(200));
    EXPECT_EQ(msg->tracked_objects[1].roi.y_offset, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[1].roi.width, static_cast<size_t>(280));
    EXPECT_EQ(msg->tracked_objects[1].roi.height, static_cast<size_t>(100));
  } else if (msg->tracked_objects[0].roi.x_offset == 200) {
    EXPECT_EQ(msg->tracked_objects[0].roi.x_offset, static_cast<size_t>(200));
    EXPECT_EQ(msg->tracked_objects[0].roi.y_offset, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[0].roi.width, static_cast<size_t>(280));
    EXPECT_EQ(msg->tracked_objects[0].roi.height, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[1].roi.x_offset, static_cast<size_t>(50));
    EXPECT_EQ(msg->tracked_objects[1].roi.y_offset, static_cast<size_t>(100));
    EXPECT_EQ(msg->tracked_objects[1].roi.width, static_cast<size_t>(430));
    EXPECT_EQ(msg->tracked_objects[1].roi.height, static_cast<size_t>(100));
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
