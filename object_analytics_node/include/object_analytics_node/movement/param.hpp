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

#ifndef OBJECT_ANALYTICS_NODE__MOVEMENT__PARAM_HPP_
#define OBJECT_ANALYTICS_NODE__MOVEMENT__PARAM_HPP_

#include <string>
#include <memory>

#include "object_analytics_node/movement/remap.hpp"

namespace object_analytics_node
{
namespace movement
{
class Param
{
public:
  using Ptr = std::shared_ptr<Param>;
  using ConstPtr = std::shared_ptr<Param const>;
  friend class MovingObjectRos;
  friend class MovingObjectFrame;
  friend class MovingObjects;

  Param();

  bool validateParam();

private:
  void init();

  bool social_filtering_enabled_;
  bool moving_object_msg_enabled_;
  double posibility_threshold_;
  int max_frames_; /**< The number of frames to be archived in memory. */
  bool velocity_enabled_;
};
}  // namespace movement
}  // namespace object_analytics_node

#endif  // OBJECT_ANALYTICS_NODE__MOVEMENT__PARAM_HPP_
