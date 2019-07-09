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

#include "render_object.hpp"

RenderObject::RenderObject()
{
  TRACE_INFO();

  Tcw_.SetIdentity();
  CvTransform_.SetIdentity();
}

RenderObject::RenderObject(float width, float height)
{
  TRACE_INFO();

  Width_ = width;
  Height_ = height;

  Tcw_.SetIdentity();
  CvTransform_.SetIdentity();
}

RenderObject::~RenderObject()
{
  TRACE_INFO();
  if (SubObjs_.size() > 0)
    SubObjs_.clear();
}

void RenderObject::AddSubObj(Ptr obj)
{
  TRACE_INFO();

  SubObjs_.push_back(obj);
}

void RenderObject::SetID(std::string id)
{
  Id_ = id;
  std::string::size_type sz;
  int i_dec = std::stoi(Id_, &sz);

  bool inv = (i_dec % 2);

  ObjColor_[0] = float(i_dec % 9) / (9.0f);

  ObjColor_[1] = float(i_dec % 5) / (5.0f);
  ObjColor_[2] = float(i_dec % 7) / (7.0f);
}

std::string RenderObject::GetID()
{
  return Id_;
}

void RenderObject::Set2DDim(int width, int height)
{
  TRACE_INFO();

  Width_ = width;
  Height_ = height;
}

bool RenderObject::Validate2DDim()
{
  TRACE_INFO();

  if (Width_ == 0 || Height_ == 0)
    return false;

  return true;
}

void RenderObject::SetPose(pangolin::OpenGlMatrix Tcw)
{
  TRACE_INFO();

  Tcw_ = Tcw.Transpose();

  if (CvCoordTransform_)
  {
    pangolin::OpenGlMatrix M_Tran = pangolin::OpenGlMatrix::Translate(-Width_ / 2, -Height_ / 2, 0);
    pangolin::OpenGlMatrix M_Rot = pangolin::OpenGlMatrix::RotateX(M_PI);
    Tcw_ = Tcw_ * M_Rot * M_Tran;
  }

  Tcw_ = Tcw_.Transpose();
}

void RenderObject::GetPose(pangolin::OpenGlMatrix& Tcw)
{
  TRACE_INFO();

  Tcw = Tcw_;
}

void RenderObject::DrawAxis(float size)
{
  glDepthFunc(GL_ALWAYS);  // to avoid visual artifacts with grid lines
                           //  glDisable(GL_LIGHTING);

  // draw axis
  glLineWidth(3);
  glBegin(GL_LINES);
  glColor3f(1, 0, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(Width_ / 2.0f, 0, 0);
  glColor3f(0, 1, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(0, Height_ / 2.0f, 0);
  glColor3f(0, 0, 1);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, -Width_ / 2.0f);
  glEnd();
  glLineWidth(3);

  // draw arrows(actually big square dots)
  glPointSize(5);
  glBegin(GL_POINTS);
  glColor3f(1, 0, 0);
  glVertex3f(size, 0, 0);
  glColor3f(0, 1, 0);
  glVertex3f(0, size, 0);
  glColor3f(0, 0, 1);
  glVertex3f(0, 0, size);
  glEnd();
  glPointSize(1);

  //  glEnable(GL_LIGHTING);
  glDepthFunc(GL_LEQUAL);
}

void RenderObject::DrawID(float size)
{
  glLineWidth(5);
  glColor4f(1.0f, 0.0f, 0.0f, 1.0f);

  pangolin::GlText txt = pangolin::GlFont::I().Text(Id_.c_str());
  txt.Draw(10, 10, 0);
}

void RenderObject::DrawGrid(float step)
{
  TRACE_INFO();

  glLineWidth(2);

  glEnable(GL_LINE_STIPPLE);
  glLineStipple(2, 0x4444);
  glBegin(GL_LINES);

  glColor3f(1.0f, 1.0f, 0.1f);
  for (float i = 0; i <= Height_; i += step)
  {
    glVertex3f(0, i, 0);  // lines parallel to X-axis
    glVertex3f(Width_, i, 0);
  }

  for (float i = 0; i <= Width_; i += step)
  {
    glVertex3f(i, 0, 0);  // lines parallel to Z-axis
    glVertex3f(i, Height_, 0);
  }

  glEnd();
  glDisable(GL_LINE_STIPPLE);

  for (float i = 0; i <= Height_; i += step)
  {
    std::string str = std::to_string((int)i);
    pangolin::GlText txt = pangolin::GlFont::I().Text(str.c_str());
    txt.Draw(0, i, 0);
    txt.Draw(Width_, i, 0);
  }

  for (float i = 0; i <= Width_; i += step)
  {
    std::string str = std::to_string((int)i);
    pangolin::GlText txt = pangolin::GlFont::I().Text(str.c_str());
    txt.Draw(i, 0, 0);  // lines parallel to Z-axis
    txt.Draw(i, Height_, 0);
  }
}

bool RenderObject::Load()
{
  TRACE_INFO();

  // Alpha:80%
  glColor4f(1.0f, 1.0f, 1.0f, 0.95f);
  // Turn Blending On
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  if (Stipple_)
  {
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(2, 0x7777);
  }

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();

  Tcw_.Transpose().Multiply();

  return true;
}

void RenderObject::Finish()
{
  TRACE_INFO();

  glPopMatrix();

  if (Stipple_)
  {
    glDisable(GL_LINE_STIPPLE);
  }
}

void RenderObject::Render()
{
  TRACE_INFO();

  if (Load())
  {
    DrawObject();

    if (VisibleAxis_)
      DrawAxis(100);

    if (VisibleID_)
      DrawID(10);

    if (VisibleGrid_)
      DrawGrid(50);

    std::vector<Ptr>::iterator it;
    for (it = SubObjs_.begin(); it != SubObjs_.end(); it++)
    {
      (*it)->Render();
    }

    Finish();
  }
}

void RenderObject::SetStipple(bool stipple)
{
  TRACE_INFO();
  Stipple_ = stipple;
}
