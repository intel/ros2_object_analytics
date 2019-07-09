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

#include "math_sample.hpp"

#include "utility.hpp"

#include "control.hpp"
#include "frame_obj.hpp"

class ControlDS : public Control{
 public:
  ControlDS();
  ~ControlDS();

  /**
   * @brief A smart pointer containing stream device object
   */
  using Ptr = std::shared_ptr<ControlDS>;

  /**
   * @brief A smart pointer to the const stream device object
   */
  using CPtr = std::shared_ptr<const ControlDS>;

  /**
   * @brief Initial internal structures
   */
  virtual bool CreateContext();

  /**
   * @brief Run the pipeline as data->process->display
   */
  virtual void Run();

  /**
   * @brief Step into next new frame 
   */
  void StepIn();

  /**
   * @brief Change to Pause mode or free run mode
   */
  void PlayMode();

 public:
  bool StepMode_ = false;
  bool PauseMode_ = true;
  bool InitialScreen_ = true;

};
