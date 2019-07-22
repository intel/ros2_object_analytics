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

#include "math_model.hpp"
#include "utility.hpp"

class StatModel : public MathModel {
 public:
  StatModel();

  ~StatModel();

  /**
   * @brief A smart pointer containing stream device object
   */
  using Ptr = std::shared_ptr<StatModel>;

  /**
   * @brief A smart pointer to the const stream device object
   */
  using CPtr = std::shared_ptr<const StatModel>;

  /**
   * @brief Set stat model params
   */
  virtual void SetMeanAndCovariance(cv::Mat& mean, cv::Mat& covariance); 

  /**
   * @brief Get stat model params
   */
  virtual void GetMeanAndCovariance(cv::Mat& mean, cv::Mat& covariance); 

  /**
   * @brief Settle down the prob model before take effect
   */
  virtual bool Settle() = 0;

  /**
   * @brief Evaluate value for specific coordinate
   */
  virtual double Evaluate(cv::Mat& coordinate) = 0;

  /**
   * @brief Get ellipse bounding rect 
   */
  virtual cv::RotatedRect GetCovEllipse();

 public:
  cv::Mat Mean_;
  cv::Mat Covariance_;
  cv::Mat InvCovariance_;
  int Dimension_ = 2;

};
