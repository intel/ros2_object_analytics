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

#pragma once

#include <stdio.h>
#include <unistd.h>
#include <cstring>
#include <memory>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "utility.hpp"

class MathModel {
 public:
  MathModel();

  ~MathModel();

  /**
   * @brief A smart pointer containing stream device object
   */
  using Ptr = std::shared_ptr<MathModel>;

  /**
   * @brief A smart pointer to the const stream device object
   */
  using CPtr = std::shared_ptr<const MathModel>;

 public:
  std::string InstName_ = "";
};
