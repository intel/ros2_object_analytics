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

#include "rw_sample.hpp"

RWSample::RWSample()
{
  TRACE_INFO();
}

RWSample::~RWSample()
{
  TRACE_INFO();
}

bool RWSample::GenSamples()
{
  TRACE_INFO();
  bool ret = false;

  if (Evaluator_Proc_ == nullptr || Ranges_.empty() || Intervals_.empty())
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
  cv::Mat high = Ranges_.col(1).clone();

  cv::RNG rng;
  cv::Mat seed = (low + high) / 2;
  double res = Evaluator_Proc_(seed);
  uint64_t count = 1;
  for (int i = 0; i < Counts_.rows; i++)
  {
    count *= Counts_.at<uint16_t>(i);
  }

  while (count > 1)
  {
    for (int i = 0; i < seed.rows; i++)
    {
      double res_inc = .0f;
      double res_dec = .0f;
      cv::Mat seed_inc = seed.clone();
      cv::Mat seed_dec = seed.clone();

      double dim_val = seed.at<double>(i);
      if (dim_val < high.at<double>(i))
      {
        seed_inc.at<double>(i) += Intervals_.at<double>(i);
        res_inc = Evaluator_Proc_(seed_inc);
      }

      if (dim_val > low.at<double>(i))
      {
        seed_dec.at<double>(i) -= Intervals_.at<double>(i);
        res_dec = Evaluator_Proc_(seed_dec);
      }

      double uni_rand = rng.uniform((double)0, res_inc + res_dec);
      if (uni_rand <= (res_inc))
      {
        seed = seed_inc.clone();
        res = res_inc;
        seed_inc.resize(seed_inc.rows + 1, res_inc);
        Samples_.push_back(seed_inc.t());
        count--;
      }
      else
      {
        seed = seed_dec.clone();
        res = res_dec;
        seed_dec.resize(seed_dec.rows + 1, res_dec);
        Samples_.push_back(seed_dec.t());
        count--;
      }
    }
  }

  return true;
}

bool RWSample::FetchSamples(cv::Mat& samples)
{
  TRACE_INFO();
  samples = Samples_;
  return true;
}
