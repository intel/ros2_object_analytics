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
#include <memory>
#include "object_analytics_node/segmenter/algorithm_provider_impl.hpp"
#include "object_analytics_node/segmenter/organized_multi_plane_segmenter.hpp"

using object_analytics_node::segmenter::OrganizedMultiPlaneSegmenter;

namespace object_analytics_node
{
namespace segmenter
{
AlgorithmProviderImpl::AlgorithmProviderImpl()
{
  algorithms_["OrganizedMultiPlaneSegmentation"] =
    std::static_pointer_cast<Algorithm>(std::make_shared<OrganizedMultiPlaneSegmenter>());
}

std::shared_ptr<Algorithm> AlgorithmProviderImpl::get()
{
  std::shared_ptr<Algorithm> algo = algorithms_.at("OrganizedMultiPlaneSegmentation");
  return algo;
}

}  // namespace segmenter
}  // namespace object_analytics_node
