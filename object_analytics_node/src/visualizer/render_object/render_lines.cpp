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

#include "render_lines.hpp"

RenderLines::RenderLines(float width, float height) : RenderObject(width, height)
{
  TRACE_INFO();
}

RenderLines::~RenderLines()
{
  TRACE_INFO();
}

bool RenderLines::Load()
{
  TRACE_INFO();

  VisibleID_ = false;
  bool ret = false;

  ret = Validate();

  if (ret)
  {
    ret = RenderObject::Load();
  }

  return ret;
}

void RenderLines::SetVertices(cv::Mat& vertices)
{
  TRACE_INFO();

  Vertices_ = vertices;
}

bool RenderLines::Validate()
{
  TRACE_INFO();
  bool ret = true;

  ret = Validate2DDim();
  if (!ret)
    return ret;

  if (Vertices_.empty())
    return false;

  ret = Vertices_.isContinuous();

  return ret;
}

void RenderLines::DrawObject()
{
  TRACE_INFO();

  glEnable(GL_LINE_SMOOTH);
  // Turn Blending On
  glEnable(GL_BLEND);
  // Turn Depth Testing Off
  //			glDisable(GL_DEPTH_TEST);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Turn Depth Testing Off
  glDisable(GL_DEPTH_TEST);

  glPointSize(5);
  glLineWidth(3);

  // glColor4f(0.5f, 0.0f, 0.0f, 0.1f);
  glColor4f(ObjColor_[0], ObjColor_[1], ObjColor_[2], 0.6f);

  glBegin(GL_LINE_STRIP);
  for (int i = 0; i < Vertices_.rows; i++)
  {
    glVertex3d(Vertices_.at<double>(i, 0), Vertices_.at<double>(i, 1), Vertices_.at<double>(i, 2) * (Height_ * 10));
  }
  glEnd();

  glEnable(GL_DEPTH_TEST);
}
