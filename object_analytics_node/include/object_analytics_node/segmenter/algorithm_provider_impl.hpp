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

#ifndef OBJECT_ANALYTICS_NODE__SEGMENTER__ALGORITHM_PROVIDER_IMPL_HPP_
#define OBJECT_ANALYTICS_NODE__SEGMENTER__ALGORITHM_PROVIDER_IMPL_HPP_

#define PCL_NO_PRECOMPILE
#include <map>
#include <string>
#include <memory>

#include "object_analytics_node/segmenter/algorithm_provider.hpp"

namespace object_analytics_node
{
namespace segmenter
{
using object_analytics_node::segmenter::Algorithm;

/** @class AlorithmProviderImpl
 *  Implementation of Segmentation algorithm factory class
 */
class AlgorithmProviderImpl : public AlgorithmProvider
{
public:
  /**
   * Constructor. Initialize algorithm map.
   */
  AlgorithmProviderImpl();

  /**
   * Default destructor
   */
  virtual ~AlgorithmProviderImpl() = default;

  /**
   * Get current selected algorithm instance
   *
   * @return Pointer to current slected algorithm instance
   */
  std::shared_ptr<Algorithm> get();

private:
  std::map<std::string, std::shared_ptr<Algorithm>> algorithms_;
};
}  // namespace segmenter
}  // namespace object_analytics_node
#endif  // OBJECT_ANALYTICS_NODE__SEGMENTER__ALGORITHM_PROVIDER_IMPL_HPP_
