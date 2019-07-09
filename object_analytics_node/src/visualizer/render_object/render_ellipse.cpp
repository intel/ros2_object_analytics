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

#include "render_ellipse.hpp"

RenderEllipse::RenderEllipse(float width, float height) : RenderObject(width, height)
{
  TRACE_INFO();
}

RenderEllipse::RenderEllipse(cv::RotatedRect rect)
{
  TRACE_INFO();

  Rect_ = rect;
}

RenderEllipse::~RenderEllipse()
{
  TRACE_INFO();
}

bool RenderEllipse::Load()
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

void RenderEllipse::SetVertices(cv::Mat& vertices)
{
  TRACE_INFO();
}

void RenderEllipse::SetEllipse(cv::RotatedRect rect)
{
  TRACE_INFO();
  Rect_ = rect;
}

bool RenderEllipse::Validate()
{
  TRACE_INFO();
  bool ret = true;

  if (Rect_.boundingRect().area() <= 0)
    return false;

  ret = Validate2DDim();

  return ret;
}

void RenderEllipse::DrawObject()
{
  TRACE_INFO();

  glPointSize(5);
  glLineWidth(3);

  // glColor4f(1.0f, 0.0f, 0.0f, 0.6f);
  glColor4f(ObjColor_[0], ObjColor_[1], ObjColor_[2], 0.6f);

  glDisable(GL_DEPTH_TEST);
#if 0
  // Draw bounding rotatedbox
  // TBD: need scale with chi-square confidentce.
  cv::Point2f vertices[4];
  Rect_.points(vertices);

  glBegin(GL_LINE_LOOP);
  for (int i = 0; i < 4; i++)
  {
    glVertex2f(vertices[i].x, vertices[i].y);
  }
  glEnd();
#endif
  // 99% confidence
  DrawEllipse(Rect_, 9.21f);
  // 95% confidence
  DrawEllipse(Rect_, 5.99f);
  // 70% confidence
  DrawEllipse(Rect_, 2.41f);

  glEnable(GL_DEPTH_TEST);
}

void RenderEllipse::DrawID(float size)
{
  glColor4f(0.0f, 0.0f, 1.0f, 1.0f);

  pangolin::GlText txt = pangolin::GlFont::I().Text(Id_.c_str());
  txt.Draw(Rect_.boundingRect().tl().x + 10, Rect_.boundingRect().tl().y + 10, 0);
}

void RenderEllipse::DrawEllipse(cv::RotatedRect& rect, double confident_scale)
{
  const int SAMPLE_POINTS = 40;
  double xc = rect.center.x;
  double yc = rect.center.y;
  double a = sqrt(confident_scale) * rect.size.width / 2.0f;
  double b = sqrt(confident_scale) * rect.size.height / 2.0f;
  double angle = M_PI * ((double)(rect.angle)) / 180.0f;
  double t = 0, cr, sr, xi, yi;
  double delta = 2 * M_PI / SAMPLE_POINTS;
  double ca = cos(angle);
  double sa = sin(angle);

  glBegin(GL_LINE_LOOP);
  while (t <= 2 * M_PI)
  {
    cr = cos(t);
    sr = sin(t);
    xi = a * cr * ca - b * sr * sa;
    yi = a * cr * sa + b * sr * ca;
    t += delta;
    glVertex2d(xc + xi, yc + yi);
  }
  glEnd();

  glBegin(GL_LINES);
  // long direction
  glVertex2d(xc, yc);
  xi = a * ca;
  yi = a * sa;
  glVertex2d(xc + xi, yc + yi);
  // short direction
  glVertex2d(xc, yc);
  xi = -b * sa;
  yi = b * ca;
  glVertex2d(xc + xi, yc + yi);
  glEnd();
}
