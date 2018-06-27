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
#include <gtest/gtest.h>
#include "object_analytics_node/tracker/tracking.hpp"

TEST(UnitTestTracking, TrackingNomal)
{
  object_analytics_node::tracker::Tracking t(2, "cat", cv::Rect2d(100, 100, 200, 300));
  int32_t t_id = t.getTrackingId();
  std::string name = t.getObjName();
  cv::Rect2d rect2 = t.getRect();
  EXPECT_EQ(name, std::string("cat"));
  EXPECT_EQ(t_id, int32_t(2));
  EXPECT_EQ(rect2.x, 100);
  EXPECT_EQ(rect2.y, 100);
  EXPECT_EQ(rect2.height, 300);
  EXPECT_EQ(rect2.width, 200);
  bool t_d = true;
  t_d = t.isDetected();
  EXPECT_EQ(t_d, false);
  t.setDetected();
  t_d = t.isDetected();
  EXPECT_EQ(t_d, true);
  t.clearDetected();
  t_d = t.isDetected();
  EXPECT_EQ(t_d, false);
}
TEST(UnitTestTracking, TrackingNullName)
{
  object_analytics_node::tracker::Tracking t(2, "", cv::Rect2d(100, 100, 200, 300));
  int32_t t_id = t.getTrackingId();
  std::string name = t.getObjName();
  cv::Rect2d rect2 = t.getRect();
  EXPECT_EQ(name, std::string(""));
  EXPECT_EQ(t_id, int32_t(2));
  EXPECT_EQ(rect2.x, 100);
  EXPECT_EQ(rect2.y, 100);
  EXPECT_EQ(rect2.height, 300);
  EXPECT_EQ(rect2.width, 200);
  bool t_d = true;
  t_d = t.isDetected();
  EXPECT_EQ(t_d, false);
  t.setDetected();
  t_d = t.isDetected();
  EXPECT_EQ(t_d, true);
  t.clearDetected();
  t_d = t.isDetected();
  EXPECT_EQ(t_d, false);
}



int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


