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

#include "gaussian_model.hpp"

GaussianModel::GaussianModel()
{
  TRACE_INFO();
}

GaussianModel::~GaussianModel()
{
  TRACE_INFO();
}

bool GaussianModel::Settle()
{
  TRACE_INFO();
}

double GaussianModel::Evaluate(cv::Mat& coordinate)
{
  // TRACE_INFO();
  double ret = .0f;

  if (coordinate.cols != 1)
  {
    TRACE_ERR("coordinate cols not 1 !!!");
    return ret;
  }

  if (coordinate.cols != Mean_.cols || coordinate.rows != Mean_.rows)
  {
    TRACE_ERR("coordinate dimension not equal to Mean_ !!!");
    return ret;
  }

  double m_dist = cv::Mahalanobis(coordinate, Mean_, InvCovariance_);
  ret = 1.0f / sqrt(cv::determinant(2 * CV_PI * Covariance_));
  ret *= exp(-std::pow(m_dist, 2) / 2.0f);

  return ret;
}
