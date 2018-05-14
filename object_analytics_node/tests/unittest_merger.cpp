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
#include <gtest/gtest.h>
#include <cassert>
#include <std_msgs/msg/header.hpp>
#include "object_analytics_node/merger/merger.hpp"
#include "unittest_util.hpp"

using object_analytics_node::merger::Merger;

TEST(UnitTestMerger, merge_BothEmpty)
{
  ObjectsInBoxes::SharedPtr objects_in_boxes2d = std::make_shared<ObjectsInBoxes>();
  std_msgs::msg::Header header2D = createHeader(builtin_interfaces::msg::Time(), "camera_rgb_optical_frame");
  objects_in_boxes2d->header = header2D;

  ObjectsInBoxes3D::SharedPtr objects_in_boxes3d = std::make_shared<ObjectsInBoxes3D>();
  std_msgs::msg::Header header3D = createHeader(builtin_interfaces::msg::Time(), "camera_rgb_optical_frame");
  objects_in_boxes3d->header = header3D;

  ObjectsInBoxes3D::SharedPtr merged = Merger::merge(objects_in_boxes2d, objects_in_boxes3d);
  EXPECT_TRUE(merged->header == header2D);
  EXPECT_EQ(merged->objects_in_boxes.size(), static_cast<size_t>(0));
}

TEST(UnitTestMerger, merge_Empty3D)
{
  ObjectsInBoxes::SharedPtr objects_in_boxes2d = std::make_shared<ObjectsInBoxes>();
  std_msgs::msg::Header header2D = createHeader(builtin_interfaces::msg::Time(), "camera_rgb_optical_frame");
  objects_in_boxes2d->header = header2D;
  objects_in_boxes2d->objects_vector.push_back(getObjectInBox(0, 0, 5, 5, "person", 0.99));
  objects_in_boxes2d->objects_vector.push_back(getObjectInBox(6, 6, 5, 5, "person", 0.99));

  ObjectsInBoxes3D::SharedPtr objects_in_boxes3d = std::make_shared<ObjectsInBoxes3D>();
  std_msgs::msg::Header header3D = createHeader(builtin_interfaces::msg::Time(), "camera_rgb_optical_frame");
  objects_in_boxes3d->header = header3D;

  ObjectsInBoxes3D::SharedPtr merged = Merger::merge(objects_in_boxes2d, objects_in_boxes3d);
  EXPECT_TRUE(merged->header == header2D);
  EXPECT_EQ(merged->objects_in_boxes.size(), static_cast<size_t>(0));
}

TEST(UnitTestMerger, merge_Empty2D)
{
  ObjectsInBoxes::SharedPtr objects_in_boxes2d = std::make_shared<ObjectsInBoxes>();
  std_msgs::msg::Header header2D = createHeader(builtin_interfaces::msg::Time(), "camera_rgb_optical_frame");
  objects_in_boxes2d->header = header2D;

  ObjectsInBoxes3D::SharedPtr objects_in_boxes3d = std::make_shared<ObjectsInBoxes3D>();
  std_msgs::msg::Header header3D = createHeader(builtin_interfaces::msg::Time(), "camera_rgb_optical_frame");
  objects_in_boxes3d->header = header3D;
  objects_in_boxes3d->objects_in_boxes.push_back(getObjectInBox3D(0, 0, 100, 100, 1, 1, 1, 100, 100, 100));

  ObjectsInBoxes3D::SharedPtr merged = Merger::merge(objects_in_boxes2d, objects_in_boxes3d);
  EXPECT_TRUE(merged->header == header2D);
  EXPECT_EQ(merged->objects_in_boxes.size(), static_cast<size_t>(0));
}

TEST(UnitTestMerger, merge_Normal)
{
  ObjectsInBoxes::SharedPtr objects_in_boxes2d = std::make_shared<ObjectsInBoxes>();
  std_msgs::msg::Header header2D = createHeader(builtin_interfaces::msg::Time(), "camera_rgb_optical_frame");
  objects_in_boxes2d->header = header2D;
  objects_in_boxes2d->objects_vector.push_back(getObjectInBox(0, 0, 100, 100, "person", 0.99));
  objects_in_boxes2d->objects_vector.push_back(getObjectInBox(101, 101, 100, 100, "person", 0.99));

  ObjectsInBoxes3D::SharedPtr objects_in_boxes3d(new ObjectsInBoxes3D);
  std_msgs::msg::Header header3D = createHeader(builtin_interfaces::msg::Time(), "camera_rgb_optical_frame");
  objects_in_boxes3d->header = header3D;
  ObjectInBox3D person = getObjectInBox3D(0, 0, 100, 100, 1, 1, 1, 100, 100, 100);
  objects_in_boxes3d->objects_in_boxes.push_back(person);

  ObjectsInBoxes3D::SharedPtr merged = Merger::merge(objects_in_boxes2d, objects_in_boxes3d);
  EXPECT_TRUE(merged->header == header2D);
  EXPECT_EQ(merged->objects_in_boxes.size(), static_cast<size_t>(1));
  EXPECT_TRUE(merged->objects_in_boxes[0] == person);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
