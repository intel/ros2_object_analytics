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

#include "object_analytics_node/const.hpp"

namespace object_analytics_node
{
const char Const::kTopicRegisteredPC2[] = "/object_analytics/registered_points";
const char Const::kTopicPC2[] = "/object_analytics/pointcloud";
const char Const::kTopicSegmentation[] = "/object_analytics/segmentation";
const char Const::kTopicRgb[] = "/object_analytics/rgb";
const char Const::kTopicDetection[] = "/object_analytics/detected_objects";
const char Const::kTopicLocalization[] = "/object_analytics/localization";
const char Const::kTopicTracking[] = "/object_analytics/tracking";
}  // namespace object_analytics_node
