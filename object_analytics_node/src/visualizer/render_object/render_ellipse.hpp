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

#include "render_object.hpp"

class RenderEllipse : public RenderObject {
 public:
  RenderEllipse(float width, float height);
  RenderEllipse(cv::RotatedRect rect);
  RenderEllipse(){};
  ~RenderEllipse();

  using Ptr = std::shared_ptr<RenderEllipse>;
  using CPtr = std::shared_ptr<const RenderEllipse>;

  virtual void SetTexture(cv::Mat &tex){};
  virtual void SetTexture(cv::Mat &tex, std::string id){};

  virtual void SetVertices(cv::Mat &vertices);
  virtual void SetEllipse(cv::RotatedRect rect);

  virtual bool Load();
  virtual void DrawObject();
  virtual bool Validate();

  virtual void DrawID(float size);

  //confident_scale associate to chi-squal iso-probability ellipse
  //99%-iso-probability ==> 9.21
  //95%-iso-probability ==> 5.99
  //70%-iso-probability ==> 2.41
  void DrawEllipse(cv::RotatedRect& rect, double confident_scale = 9.21);

 public:
  cv::RotatedRect Rect_;

 private:
};
