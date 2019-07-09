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

#include "stat_model.hpp"

StatModel::StatModel()
{
  TRACE_INFO();
}

StatModel::~StatModel()
{
  TRACE_INFO();
}

void StatModel::SetMeanAndCovariance(cv::Mat& mean, cv::Mat& covariance)
{
  TRACE_INFO();

  Mean_ = mean.clone();
  Covariance_ = covariance.clone();
  InvCovariance_ = covariance.inv();
}

void StatModel::GetMeanAndCovariance(cv::Mat& mean, cv::Mat& covariance)
{
  TRACE_INFO();

  mean = Mean_.clone();
  covariance = Covariance_.clone();
}

cv::RotatedRect StatModel::GetCovEllipse()
{
  TRACE_INFO();
  cv::RotatedRect box;

  cv::Mat U, S, Vt;
  cv::SVD::compute(Covariance_, S, U, Vt);
  box.size.width = sqrt(S.at<double>(0)) * 2;
  box.size.height = sqrt(S.at<double>(1)) * 2;
  box.angle = std::atan2(U.at<double>(1, 0), U.at<double>(0, 0)) * 180 / CV_PI;
  /*
   * Consider to use eigen functions for simplify
  cv::Mat eigen_vals, eigen_vecs;
  cv::eigen(Covariance_, eigen_vals, eigen_vecs);
  */
  if (box.size.width < box.size.height)
  {
    float tmp;
    CV_SWAP(box.size.width, box.size.height, tmp);
    box.angle = std::atan2(U.at<double>(1, 1), U.at<double>(0, 1)) * 180 / CV_PI;
  }

  if (box.angle < -180)
    box.angle += 360;
  if (box.angle > 360)
    box.angle -= 360;

  box.center.x = Mean_.at<double>(0);
  box.center.y = Mean_.at<double>(1);

  return box;
}
