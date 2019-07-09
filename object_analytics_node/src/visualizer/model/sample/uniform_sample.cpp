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

#include "uniform_sample.hpp"

UniformSample::UniformSample()
{
  TRACE_INFO();
}

UniformSample::~UniformSample()
{
  TRACE_INFO();
}

void UniformSample::RecursiveSampling(cv::Mat& start, cv::Mat& interval, cv::Mat& counts, int l_offset)
{
  //  TRACE_INFO();
  uint16_t count = counts.at<uint16_t>(l_offset);
  uint16_t idx = 0;
  static uint16_t sample_idx = 0;

  do
  {
    cv::Mat sample_rec = start.clone();
    sample_rec.at<double>(l_offset) += idx * interval.at<double>(l_offset);
    if (l_offset == (interval.rows - 1))
    {
      //  sample_rec.resize(sample_rec.rows+1, 3.14159f);
      double res = Evaluator_Proc_(sample_rec);

      if (max_pdf < res)
        max_pdf = res;
      if (min_pdf > res)
        min_pdf = res;

      sample_rec.resize(sample_rec.rows + 1, res);
      Samples_.push_back(sample_rec.t());
    }
    else
    {
      RecursiveSampling(sample_rec, interval, counts, l_offset + 1);
    }
  } while (++idx < count);
}

bool UniformSample::GenSamples()
{
  TRACE_INFO();
  bool ret = false;

  if (Evaluator_Proc_ == nullptr || Ranges_.empty() || Intervals_.empty())
  //  if (Ranges_.empty() || Intervals_.empty())
  {
    TRACE_ERR("Sampler not initialized correctly!!!");
    return ret;
  }

  if (Ranges_.rows != Intervals_.rows)
  {
    TRACE_ERR("Sampler ranges and intervals not match!!!");
    TRACE_ERR("Ranges rows(%d), Intervals rows(%d)", Ranges_.rows, Intervals_.rows);
    return ret;
  }

  cv::Mat low = Ranges_.col(0).clone();

  RecursiveSampling(low, Intervals_, Counts_, 0);

  return true;
}

bool UniformSample::FetchSamples(cv::Mat& samples)
{
  TRACE_INFO();
  samples = Samples_;
  return true;
}
