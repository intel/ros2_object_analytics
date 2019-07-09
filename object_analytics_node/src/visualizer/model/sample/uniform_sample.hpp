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

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <memory>
#include <string>

#include "sample_model.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "utility.hpp"

class UniformSample : public SampleModel {
 public:
  UniformSample();

  ~UniformSample();

  /**
   * @brief A smart pointer containing stream device object
   */
  using Ptr = std::shared_ptr<UniformSample>;

  /**
   * @brief A smart pointer to the const stream device object
   */
  using CPtr = std::shared_ptr<const UniformSample>;

 public:
  /**
   * @brief Generate samples 
   */
	virtual bool GenSamples();

  /**
   * @brief Generate samples and restore in Samples_
   */
  void RecursiveSampling(cv::Mat& start, cv::Mat& interval, cv::Mat& counts, int l_offset);

  /**
   * @brief Fetch samples 
   */
	virtual bool FetchSamples(cv::Mat& samples);

};
