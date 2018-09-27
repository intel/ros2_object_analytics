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

#ifndef OBJECT_ANALYTICS_NODE__SEGMENTER__ALGORITHM_CONFIG_HPP_
#define OBJECT_ANALYTICS_NODE__SEGMENTER__ALGORITHM_CONFIG_HPP_

#include <map>
#include <string>
#include "object_analytics_node/util/file_parser.hpp"

namespace object_analytics_node
{
namespace segmenter
{
/** @class AlorithmConfig
 *
 * Encapsulate config related operations.
 */
class AlgorithmConfig
{
public:
  /**
   * Constructor of AlgorithmConfig
   *
   * @param[in] name Configuration file name
   */
  AlgorithmConfig()
  {
    // TODO(Peter Han): Don't want to recreate the wheel.
    // To leverage 3rd part library, should get approved.
    // Not implement this is totally okay, becasue default value is always provided.
  }

  /**
   * Get value of given key
   *
   * @param[in] key         name of query item
   * @param[in] default_val default value used when fail to query
   *
   * @return value of given key, def_val if failed to query the key
   */
  template<typename T>
  inline T get(const std::string & key, const T default_val)
  {
    assert(false);
    return default_val;
  }

private:
  std::map<std::string, std::string> map_;
};

template<>
inline std::string AlgorithmConfig::get<std::string>(
  const std::string & key, const std::string default_val)
{
  try {
    return map_[key];
  } catch (...) {
  }
  return default_val;
}

template<>
inline size_t AlgorithmConfig::get<size_t>(const std::string & key, const size_t default_val)
{
  try {
    return static_cast<size_t>(std::stoi(map_[key]));
  } catch (...) {
  }
  return default_val;
}

template<>
inline float AlgorithmConfig::get<float>(const std::string & key, const float default_val)
{
  try {
    return static_cast<float>(std::stof(map_[key]));
  } catch (...) {
  }
  return default_val;
}

}  // namespace segmenter
}  // namespace object_analytics_node
#endif  // OBJECT_ANALYTICS_NODE__SEGMENTER__ALGORITHM_CONFIG_HPP_
