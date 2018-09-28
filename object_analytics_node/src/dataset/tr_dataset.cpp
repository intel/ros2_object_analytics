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

#include <string>
#include <vector>

#include "object_analytics_node/dataset/track_dataset.hpp"

namespace datasets
{

cv::Ptr<trDataset> trDataset::create(dsType type)
{
  cv::Ptr<trDataset> nullp;
  switch (type) {
    case dsVideo:
      return cv::Ptr<vidDataset>(new vidDataset);
    case dsImage:
      return cv::Ptr<imgDataset>(new imgDataset);
    default:
      return nullp;
  }
}

int trDataset::getFrameIdx() {return frameIdx;}

}  // namespace datasets
