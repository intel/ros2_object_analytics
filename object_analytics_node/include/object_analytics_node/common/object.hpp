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

#ifndef OBJECT_ANALYTICS_NODE__COMMON__OBJECT_HPP_
#define OBJECT_ANALYTICS_NODE__COMMON__OBJECT_HPP_

#include <opencv2/opencv.hpp>

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

class Object
{
public:
  Object() {}
  ~Object() {}

public:
  int ObjectIdx_;
  std::string Category_;
  cv::Rect2d BoundBox_;
  float Confidence_;

  struct timespec Stamp_;

  cv::Mat Mean_;
  cv::Mat Covariance_;
};

#endif  // OBJECT_ANALYTICS_NODE__COMMON__OBJECT_HPP_
