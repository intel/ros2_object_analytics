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

#include "frame.hpp"

sFrame::sFrame(cv::Mat& cv_frame)
{
  frame = cv_frame;
  stamp = getTimeStamp();
}

std::time_t sFrame::getTimeStamp()
{
  std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp =
      std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());

  return tp.time_since_epoch().count();
}

void sFrame::genFrame(cv::Mat& cv_frame)
{
  frame = cv_frame;
  stamp = getTimeStamp();
}
