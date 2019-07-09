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

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

#include "render_object.hpp"
#include "utility.hpp"

class View {
 public:
  View();

  ~View();

  using Ptr = std::shared_ptr<View>;
  using CPtr = std::shared_ptr<const View>;

  // init OpenGL window and display
  bool InitDisplay(float width = 640.0f, float height = 480.0f,
                   float border_ratio = 0.2f);

  // reset OpenGL to initial state
  void Reset();

  // add object to view
  bool Add_Obj(RenderObject::Ptr& obj);

  // remove object from view
  bool Remove_Obj(RenderObject::Ptr& obj);

  // render view with all child objects
  void Render();

  enum Layout { LayoutHorizontal = 0, LayoutVertical, LayoutEqual, LayoutMax };

  // set Layout options 
  void SetLayout(Layout layout);

  // Perform layout according to objects change 
  void PerformLayout(Layout layout);

  // Change to other layout
  void ChangeLayout();

  // Draw grid on XZ plane 
  void DrawXZGrid(float x_size, float z_size, float step);

 public:
  // render state management
  pangolin::OpenGlRenderState S_Render_;

  // view handler
  pangolin::View* Disp_;

  // default resolution
  float Width_ = 640.0f;
  float Height_ = 480.0f;

  // screen projection border
  float Border_Ratio_ = 0.2f;

  // object list in view
  std::vector<RenderObject::Ptr> Obj_Vec_;
  std::uint32_t Obj_Max_Size_;

  Layout Layout_;
};
