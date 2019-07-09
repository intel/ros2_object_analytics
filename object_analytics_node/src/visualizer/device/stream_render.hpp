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

#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

class stream_render {
 public:
  stream_render();
  ~stream_render();

  /**
   * @brief A smart pointer containing stream device object
   */
  using Ptr = std::shared_ptr<stream_render>;

  /**
   * @brief A smart pointer to the const stream device object
   */
  using CPtr = std::shared_ptr<const stream_render>;

  /**
   * @brief init models which used by data proc
   */
  bool init_models();

 protected:
  std::vector<cv::Mat> face;
  std::vector<cv::Mat> person;
  std::vector<cv::Mat> reID;

 private:
};
