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

#include "math_sample.hpp"

MathSample::MathSample()
{
  TRACE_INFO();
}

MathSample::~MathSample()
{
  TRACE_INFO();
}

bool MathSample::Initial(std::string model, std::string sampler)
{
  TRACE_INFO();

  if (model == "Gaussian")
  {
    MathModel_ = std::make_shared<GaussianModel>();
  }
  else
  {
    TRACE_ERR("model(%s) is not exist!!!", model.c_str());
    return false;
  }

  if (sampler == "Uniform")
  {
    SampleModel_ = std::make_shared<UniformSample>();
  }
  else if (sampler == "RandomWalker")
  {
    SampleModel_ = std::make_shared<RWSample>();
  }
  else
  {
    TRACE_ERR("sampler(%s) is not exist!!!", sampler.c_str());
    return false;
  }

  return true;
}

bool MathSample::SetMeanAndCovariance(cv::Mat& mean, cv::Mat& covariance, uint32_t counts)
{
  TRACE_INFO();
  bool ret = false;

  MathModel_->SetMeanAndCovariance(mean, covariance);

  // Caculate ranges and intervals according to counts
  cv::Mat M_Range = cv::Mat(mean.rows, 2, CV_64FC1);
  cv::Mat M_Interval = cv::Mat(mean.rows, 1, CV_64FC1);
  for (int i = 0; i < mean.rows; i++)
  {
    double mu = mean.at<double>(i);
    double stddev = covariance.at<double>(i, i);
    M_Range.at<double>(i, 0) = mu - 3.0f * stddev;
    M_Range.at<double>(i, 1) = mu + 3.0f * stddev;
    M_Interval.at<double>(i) = 6.0f * stddev / (counts - 1);
  }

  Counts_ = counts;

  // Setup sampler
  SampleModel_->RegisterEvaluator(std::bind(&StatModel::Evaluate, MathModel_.get(), std::placeholders::_1));
  ret = SampleModel_->SetRanges(M_Range, M_Interval);

  return ret;
}

bool MathSample::GetMeanAndCovariance(cv::Mat& mean, cv::Mat& covariance)
{
  TRACE_INFO();

  MathModel_->GetMeanAndCovariance(mean, covariance);

  return true;
}

bool MathSample::SetSampleCounts(uint32_t counts)
{
  TRACE_INFO();

  Counts_ = counts;

  return false;
}

bool MathSample::GenSamples()
{
  TRACE_INFO();

  return SampleModel_->GenSamples();
}

bool MathSample::FetchSamples(cv::Mat& samples)
{
  TRACE_INFO();

  SampleModel_->FetchSamples(samples);

  return true;
}

cv::RotatedRect MathSample::GetCovEllipse()
{
  TRACE_INFO();

  return MathModel_->GetCovEllipse();
}
