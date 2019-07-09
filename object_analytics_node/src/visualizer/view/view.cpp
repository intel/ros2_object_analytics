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

#include "view.hpp"

View::View()
{
  TRACE_INFO();

  // default layout
  Layout_ = LayoutHorizontal;
  // default objects count
  Obj_Max_Size_ = 3;
}

View::~View()
{
  TRACE_INFO();

  Obj_Vec_.clear();
}

bool View::Add_Obj(RenderObject::Ptr& obj)
{
  TRACE_INFO();

  if (Obj_Vec_.size() == Obj_Max_Size_)
    Obj_Vec_.erase(Obj_Vec_.begin());

  Obj_Vec_.push_back(obj);

  PerformLayout(Layout_);

  return true;
}

bool View::Remove_Obj(RenderObject::Ptr& obj)
{
  TRACE_INFO();

  int i;

  for (i = 0; i < Obj_Vec_.size(); i++)
  {
    if (Obj_Vec_[i] == obj)
    {
      Obj_Vec_.erase(Obj_Vec_.begin() + i);
      return true;
    }
  }

  PerformLayout(Layout_);

  return false;
}

bool View::InitDisplay(float width, float height, float border_ratio)
{
  TRACE_INFO();

  // 1. Validate initialize params
  if (width <= .0f || height <= .0f || border_ratio <= .0f)
  {
    TRACE_ERR("View initialize params got issue!!!");
    return false;
  }
  else
  {
    Width_ = width;
    Height_ = height;
    Border_Ratio_ = border_ratio;
  }

  // 2. Init OpenGL window
  pangolin::CreateWindowAndBind("RenderObject Analytics Visuializer", Width_, Height_);
  glEnable(GL_DEPTH_TEST);

  // 3. Init render engine (with MVP models)
  S_Render_ =
      pangolin::OpenGlRenderState(pangolin::ProjectionMatrix(Width_ * 2, Height_ * 2, Width_ * 2 / Border_Ratio_,
                                                             Width_ * 2 / Border_Ratio_, Width_, Height_, 0.1, 100000),
                                  pangolin::ModelViewLookAt(1.0f, Width_, 1.0f,        /*pos*/
                                                            0.0f, 0.0f, -(5 * Width_), /*lookat*/
                                                            0.0f, 1.0f, 0.0f /*up*/));

  // 4. Init display and viewport
  Disp_ = &(pangolin::CreateDisplay());
  Disp_->SetBounds(0.0f, 1.0f, 0.0f, 1.0f, -Width_ / Height_);
  Disp_->SetHandler(new pangolin::Handler3D(S_Render_));

  return true;
}

void View::Reset()
{
  TRACE_INFO();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  // Activate camera view
  Disp_->Activate(S_Render_);
  // Full Brightness, 100% Alpha
  glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
}

void View::SetLayout(Layout layout)
{
  TRACE_INFO();

  Layout_ = layout;

  PerformLayout(Layout_);
}

void View::ChangeLayout()
{
  TRACE_INFO();

  if (Layout_ == LayoutHorizontal)
    Layout_ = LayoutVertical;
  else
    Layout_ = LayoutHorizontal;

  PerformLayout(Layout_);
}

void View::PerformLayout(Layout layout)
{
  TRACE_INFO();

  int idx = 0;
  int vec_size = Obj_Vec_.size();
  pangolin::OpenGlMatrix M_Rot;
  pangolin::OpenGlMatrix M_Tran;

  if (layout == LayoutHorizontal)
  {
    pangolin::GLprecision theta_rad = (30.0f * M_PI) / 180.0f;
    M_Rot = pangolin::OpenGlMatrix::RotateY(theta_rad);
  }
  else if (layout == LayoutVertical)
  {
    pangolin::GLprecision theta_rad = (-60.0f * M_PI) / 180.0f;
    M_Rot = pangolin::OpenGlMatrix::RotateX(theta_rad);
  }

  std::vector<RenderObject::Ptr>::iterator it;

  for (it = Obj_Vec_.begin(); it != Obj_Vec_.end(); it++)
  {
    if (layout == LayoutHorizontal)
    {
      int delta_w = idx - vec_size + 1;
      M_Tran = pangolin::OpenGlMatrix::Translate(delta_w * Width_, 0, -(5 * Width_));
    }
    else if (layout == LayoutVertical)
    {
      int delta_h = idx - vec_size + 1;
      M_Tran = pangolin::OpenGlMatrix::Translate(0, delta_h * Height_, -(5 * Width_));
    }

    pangolin::OpenGlMatrix pose = M_Tran * M_Rot;

    (*it)->SetPose(pose.Transpose());

    idx++;
  }
}

void View::DrawXZGrid(float x_size, float z_size, float step)
{
  TRACE_INFO();

  glLineWidth(1);
  glEnable(GL_LINE_STIPPLE);
  glLineStipple(2, 0x4444);

  glBegin(GL_LINES);

  glColor3f(0.3f, 0.3f, 0.3f);
  for (float i = step; i <= z_size; i += step)
  {
    glVertex3f(-x_size, 0, i);  // lines parallel to X-axis
    glVertex3f(x_size, 0, i);
    glVertex3f(-x_size, 0, -i);  // lines parallel to X-axis
    glVertex3f(x_size, 0, -i);
  }

  for (float i = step; i <= x_size; i += step)
  {
    glVertex3f(i, 0, -z_size);  // lines parallel to Z-axis
    glVertex3f(i, 0, z_size);
    glVertex3f(-i, 0, -z_size);  // lines parallel to Z-axis
    glVertex3f(-i, 0, z_size);
  }

  // x-axis
  glColor3f(0.5f, 0, 0);
  glVertex3f(-x_size, 0, 0);
  glVertex3f(x_size, 0, 0);

  // z-axis
  glColor3f(0, 0, 0.5f);
  glVertex3f(0, 0, -z_size);
  glVertex3f(0, 0, z_size);

  glEnd();
  glDisable(GL_LINE_STIPPLE);
}

void View::Render()
{
  Disp_->Activate(S_Render_);

  //  DrawXZGrid(2*Width_, 2*Width_, 50);

  std::vector<RenderObject::Ptr>::iterator it;

  for (it = Obj_Vec_.begin(); it != Obj_Vec_.end(); it++)
  {
    (*it)->Render();
  }
}
