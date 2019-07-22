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

class RenderRect : public RenderObject {
 public:
  RenderRect(float width, float height);
  RenderRect(cv::Rect rect);
  RenderRect(){};
  ~RenderRect();

  using Ptr = std::shared_ptr<RenderRect>;
  using CPtr = std::shared_ptr<const RenderRect>;

  virtual void SetTexture(cv::Mat &tex){};
  virtual void SetTexture(cv::Mat &tex, std::string id){};

  virtual void SetVertices(cv::Mat &vertices);
  virtual void SetRect(cv::Rect rect);

  virtual bool Load();
  virtual void DrawObject();
  virtual bool Validate();

  virtual void DrawID(float size);

 public:
  cv::Rect Rect_;

 private:
};
