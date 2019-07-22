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
#include <string.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>
#include <pangolin/gl/gltext.h>

#include "utility.hpp"

class RenderObject {
 public:
  RenderObject();
  RenderObject(float width, float height);
  ~RenderObject();

  using Ptr = std::shared_ptr<RenderObject>;
  using CPtr = std::shared_ptr<const RenderObject>;

  virtual void Set2DDim(int width, int height);
  virtual bool Validate2DDim();
  virtual bool Validate() = 0;

  virtual void SetPose(pangolin::OpenGlMatrix Tcw);
  virtual void GetPose(pangolin::OpenGlMatrix &Tcw);

  virtual bool Load();
  virtual void Render();
  virtual void Finish();

  //To be implement!!!!
  //SetColorForeground and SetColorBackground 


  virtual void DrawAxis(float size);
  virtual void DrawID(float size);
  virtual void DrawObject() = 0;
  virtual void DrawGrid(float step);

  virtual void SetTexture(cv::Mat &tex) = 0;
  virtual void SetTexture(cv::Mat &tex, std::string id) = 0;

  virtual void SetVertices(cv::Mat &vertices) = 0;

  virtual void AddSubObj(Ptr obj);

  virtual void SetID(std::string id);
  virtual std::string GetID();
  virtual void SetStipple(bool stipple);

 public:
  pangolin::OpenGlMatrix Tcw_;
  pangolin::OpenGlMatrix CvTransform_;
  float Width_ = 0;
  float Height_ = 0;

  std::string Id_;
  pangolin::GlSlProgram prog_text;

  bool VisibleAxis_ = true;
  bool VisibleID_ = true;
  bool VisibleGrid_ = false;
  bool CvCoordTransform_ = false;

  bool Stipple_ = false;
  /*R, G, B color caculate from ID_*/
  float ObjColor_[3]  = {0.3f, 0.3f, 0.3f};

  std::vector<Ptr> SubObjs_;

 private:
};
