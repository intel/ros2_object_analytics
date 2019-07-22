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
#include <cstring>
#include <memory>
#include <string>
#include <functional>
#include <iostream>

#include <opencv2/core/core.hpp>
#include "utility.hpp"

class SampleModel {
 public:
  SampleModel();

  ~SampleModel();

  /**
   * @brief A smart pointer containing stream device object
   */
  using Ptr = std::shared_ptr<SampleModel>;

  /**
   * @brief A smart pointer to the const stream device object
   */
  using CPtr = std::shared_ptr<const SampleModel>;

  /**
   * @brief Callback function type
   */
	using CBPtr = std::function<double(cv::Mat&)>;

  /**
   * @brief Set Range for each dimension
   */
  virtual bool SetRanges(cv::Mat ranges, cv::Mat intervals);

  /**
   * @brief Register evaluator callback 
   */
	virtual bool RegisterEvaluator(CBPtr evaluator);

  /**
   * @brief Generate samples 
   */
	virtual bool GenSamples() = 0;

  /**
   * @brief Fetch samples 
   */
	virtual bool FetchSamples(cv::Mat& samples) = 0;


 public:
  /**
   * @brief Max and Min pdf value recorded during sampling
   */
  double max_pdf = .0f;
  double min_pdf = .0f; 

  /**
   * @brief Range for each dimension, format as Dimension X 2(low, high)
   */
  cv::Mat Samples_;

  /**
   * @brief Range for each dimension, format as Dimension X 2(low, high)
   */
  cv::Mat Ranges_;

  /**
   * @brief Interval for each dimension, format as Dimension x 1(interval)
   */
  cv::Mat Intervals_;

  /**
   * @brief Sample count for each dimension 
   */
  cv::Mat Counts_;

  /**
   * @brief Callback of evaluator
   */
  CBPtr Evaluator_Proc_ = nullptr;

};
