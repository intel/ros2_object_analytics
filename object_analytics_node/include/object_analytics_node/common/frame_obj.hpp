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

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "utility.hpp"
#include "frame.hpp"

class FrameObjs : public sFrame{
 public:
  FrameObjs(){};

  virtual ~FrameObjs() = default;

  /**
   * @brief Generate sframe from cv::Mat and frame index
   */
  virtual void genFrame(cv::Mat &cv_frame, int idx);

  /**
   * @brief Add detection for the frame
   */
  void AddDetection(Object& detect);

  /**
   * @brief Add track for the frame
   */
  void AddTrack(Object& track);

 public:
  int frame_idx = -1;

  std::vector<Object> dets;
  std::vector<Object> tracks;
};
