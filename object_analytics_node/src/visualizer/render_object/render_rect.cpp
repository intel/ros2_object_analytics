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

#include "render_rect.hpp"

RenderRect::RenderRect(float width, float height) : RenderObject(width, height)
{
  TRACE_INFO();
}

RenderRect::RenderRect(cv::Rect rect)
{
  TRACE_INFO();
  Rect_ = rect;
}

RenderRect::~RenderRect()
{
  TRACE_INFO();
}

bool RenderRect::Load()
{
  TRACE_INFO();

  bool ret = false;

  ret = Validate();

  if (ret)
  {
    ret = RenderObject::Load();
  }

  return ret;
}

void RenderRect::SetVertices(cv::Mat& vertices)
{
  TRACE_INFO();
}

void RenderRect::SetRect(cv::Rect rect)
{
  TRACE_INFO();
  Rect_ = rect;
}

bool RenderRect::Validate()
{
  TRACE_INFO();
  bool ret = true;

  if (Rect_.area() <= 0)
    return false;

  ret = Validate2DDim();

  return ret;
}

void RenderRect::DrawObject()
{
  TRACE_INFO();

  glPointSize(5);
  glLineWidth(3);

  // glColor4f(1.0f, 0.0f, 0.0f, 0.6f);
  glColor4f(ObjColor_[0], ObjColor_[1], ObjColor_[2], 0.6f);

  glDisable(GL_DEPTH_TEST);
  if (Stipple_)
  {
    glBegin(GL_LINE_LOOP);
  }
  else
  {
    glBegin(GL_QUADS);
  }

  glVertex2i(Rect_.tl().x, Rect_.tl().y);
  glVertex2i(Rect_.br().x, Rect_.tl().y);
  glVertex2i(Rect_.br().x, Rect_.br().y);
  glVertex2i(Rect_.tl().x, Rect_.br().y);
  glEnd();

  glEnable(GL_DEPTH_TEST);
}

void RenderRect::DrawID(float size)
{
  glColor4f(0.0f, 0.0f, 1.0f, 1.0f);

  pangolin::GlText txt = pangolin::GlFont::I().Text(Id_.c_str());
  txt.Draw(Rect_.tl().x + 10, Rect_.tl().y + 10, 0);
}
