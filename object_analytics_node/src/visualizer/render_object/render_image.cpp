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

#include "render_image.hpp"

RenderImage::RenderImage(float width, float height) : RenderObject(width, height)
{
  TRACE_INFO();
  VisibleGrid_ = true;
  CvCoordTransform_ = true;
}

RenderImage::~RenderImage()
{
  TRACE_INFO();
}

void RenderImage::SetTexture(cv::Mat& tex)
{
  TRACE_INFO();

  Tex_ = tex;

  RenderImageTex_ = pangolin::GlTexture(Tex_.cols, Tex_.rows, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
}

void RenderImage::SetTexture(cv::Mat& tex, std::string id)
{
  TRACE_INFO();

  Id_ = id;

  SetTexture(tex);

  cv::putText(Tex_, Id_, cv::Point(5, Tex_.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
}

bool RenderImage::Load()
{
  TRACE_INFO();

  bool ret = false;

  ret = Validate();

  if (ret)
  {
    RenderImageTex_.Upload(Tex_.data, GL_RGB, GL_UNSIGNED_BYTE);
    ret = RenderObject::Load();
  }

  return ret;
}

bool RenderImage::Validate()
{
  TRACE_INFO();
  bool ret = true;

  ret = Validate2DDim();
  if (!ret)
    return ret;

  if (Tex_.empty())
    return false;

  return ret;
}

void RenderImage::SetVertices(cv::Mat& vertices)
{
}

void RenderImage::DrawObject()
{
  TRACE_INFO();

  //  GLfloat sq_vert[] = {
  //      -Width_ / 2, Height_ / 2,  0, -Width_ / 2,  -Height_ / 2,  0,
  //      Width_ / 2, Height_ / 2, 0, Width_ / 2,  -Height_ / 2, 0};

  GLfloat sq_vert[] = { 0, 0, 0, 0, Height_, 0, Width_, 0, 0, Width_, Height_, 0 };

  glVertexPointer(3, GL_FLOAT, 0, sq_vert);
  glEnableClientState(GL_VERTEX_ARRAY);

  //  GLfloat sq_tex[] = {0, 0, 0, 1, 1, 0, 1, 1};
  GLfloat sq_tex[] = { 0, 0, 0, 1, 1, 0, 1, 1 };
  // Flip Y
  // GLfloat sq_tex[] = {1,0,  0,0,  0,1,  1,1};
  glTexCoordPointer(2, GL_FLOAT, 0, sq_tex);
  glEnableClientState(GL_TEXTURE_COORD_ARRAY);

  glEnable(GL_TEXTURE_2D);
  glDrawArrays(GL_QUAD_STRIP, 0, 4);
  //  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_TEXTURE_COORD_ARRAY);
  glDisable(GL_TEXTURE_2D);
}
