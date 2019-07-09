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

#include "sample_model.hpp"

SampleModel::SampleModel()
{
  TRACE_INFO();
}

SampleModel::~SampleModel()
{
  TRACE_INFO();
}

bool SampleModel::RegisterEvaluator(CBPtr evaluator)
{
  TRACE_INFO();
  bool ret = false;

  if (Evaluator_Proc_ == nullptr)
  {
    Evaluator_Proc_ = evaluator;
    ret = true;

    TRACE_INFO("Successfull Registered Evaluator!");
  }
  else
  {
    ret = false;
    TRACE_ERR("Failed, Evaluator already been registerred!");
  }

  return ret;
}

bool SampleModel::SetRanges(cv::Mat ranges, cv::Mat intervals)
{
  TRACE_INFO();
  bool ret = false;

  if ((ranges.type() != CV_64FC1) || (intervals.type() != CV_64FC1))
  {
    TRACE_ERR("Range or Interval format error!!!");
    return ret;
  }

  Ranges_ = ranges.clone();
  Intervals_ = intervals.clone();

  Counts_ = cv::Mat::zeros(Ranges_.rows, 1, CV_16UC1);
  for (int i = 0; i < Ranges_.rows; i++)
  {
    double high = Ranges_.at<double>(i, 1);
    double low = Ranges_.at<double>(i, 0);
    double interval = Intervals_.at<double>(i);

    if (high <= low || interval <= 0)
    {
      TRACE_ERR("Range or Interval value error!!!");
      return ret;
    }

    Counts_.at<uint16_t>(i) = std::floor((high - low) / interval) + 1;
  }

  Samples_ = cv::Mat(0, Ranges_.rows + 1, CV_64FC1);

  return true;
}
