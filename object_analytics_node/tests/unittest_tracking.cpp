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
#include <cassert>
#include <string>
#include "tracker/tracking.hpp"

TEST(UnitTestTracking, TrackingNomal) {
  cv::Mat image = cv::Mat::zeros(640, 480, CV_32F);
  timespec stamp;
  stamp.tv_sec = 1;
  stamp.tv_nsec = 1;
  std::shared_ptr<sFrame> frame = std::make_shared<sFrame>(image, stamp);
  tracker::Tracking t(2, "cat");
  t.rectifyTracker(frame, cv::Rect2d(100,100,200,300));

  EXPECT_EQ(t.getObjName(), std::string("cat"));
  EXPECT_EQ(t.getTrackingId(), int32_t(2));
  EXPECT_EQ(t.getTrackedRect().x, 100);
  EXPECT_EQ(t.getTrackedRect().y, 100);
  EXPECT_EQ(t.getTrackedRect().height, 300);
  EXPECT_EQ(t.getTrackedRect().width, 200);
}
TEST(UnitTestTracking, TrackingNullName) {
  cv::Mat image = cv::Mat::zeros(640, 480, CV_32F);
  timespec stamp;
  stamp.tv_sec = 1;
  stamp.tv_nsec = 1;
  std::shared_ptr<sFrame> frame = std::make_shared<sFrame>(image, stamp);
  tracker::Tracking t(2, "");
  t.rectifyTracker(frame, cv::Rect2d(100,100,200,300));

  EXPECT_EQ(t.getObjName(), std::string(""));
  EXPECT_EQ(t.getTrackingId(), int32_t(2));
  EXPECT_EQ(t.getTrackedRect().x, 100);
  EXPECT_EQ(t.getTrackedRect().y, 100);
  EXPECT_EQ(t.getTrackedRect().height, 300);
  EXPECT_EQ(t.getTrackedRect().width, 200);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
