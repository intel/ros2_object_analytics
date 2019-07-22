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

#include "control_ds.hpp"
#include "render_lines.hpp"
#include "render_rect.hpp"
#include "render_ellipse.hpp"
#include "render_image.hpp"

ControlDS::ControlDS()
{
  TRACE_INFO();
}

ControlDS::~ControlDS()
{
  TRACE_INFO();
}

bool ControlDS::CreateContext()
{
  TRACE_INFO();
  bool ret = false;

  std::shared_ptr<sFrame> im;
  ret = StreamDev_->process();
  if (!ret)
  {
    TRACE_ERR("Capture Device error, start process failed!!!");
    return ret;
  }

  ret = StreamDev_->query(im);
  if (!ret || im->frame.empty())
  {
    TRACE_ERR("Capture Device error, read frame failed!!!");
    return ret;
  }

  ret = DataView_->InitDisplay(im->frame.cols, im->frame.rows);
  if (!ret)
  {
    TRACE_ERR("Data View initialize failed!!!");
    return ret;
  }

  pangolin::RegisterKeyPressCallback(' ', [this]() { PlayMode(); });
  pangolin::RegisterKeyPressCallback('s', [this]() { StepIn(); });
  pangolin::RegisterKeyPressCallback('l', [this]() { DataView_->ChangeLayout(); });

  return ret;
}

void ControlDS::StepIn()
{
  TRACE_INFO();
  StepMode_ = true;
  PauseMode_ = false;
}

void ControlDS::PlayMode()
{
  TRACE_INFO();
  PauseMode_ = !PauseMode_;
}

void ControlDS::Run()
{
  TRACE_INFO();

  while (!pangolin::ShouldQuit())
  {
    DataView_->Reset();

    if (!PauseMode_ || InitialScreen_)
    {
      InitialScreen_ = false;

      if (StepMode_)
      {
        StepMode_ = false;
        PauseMode_ = true;
      }

      std::shared_ptr<sFrame> im;
      bool ret = StreamDev_->read(im);
      if (!ret || im->frame.empty())
      {
        TRACE_ERR("Can not get frame!!!");
        break;
      }

      std::shared_ptr<FrameObjs> objs = std::dynamic_pointer_cast<FrameObjs>(im);

      cv::cvtColor(im->frame, im->frame, CV_RGB2BGR);

      std::shared_ptr<RenderObject> img = std::make_shared<RenderImage>(im->frame.cols, im->frame.rows);
      img->SetTexture(im->frame, std::to_string(objs->frame_idx));

      for (auto t : objs->dets)
      {
        std::shared_ptr<RenderRect> rect = std::make_shared<RenderRect>(im->frame.cols, im->frame.rows);
        rect->SetRect(t.BoundBox_);
        rect->SetID(std::to_string(t.ObjectIdx_));
        img->AddSubObj(rect);

        double mean[] = { t.BoundBox_.tl().x, t.BoundBox_.tl().y };
        double covariance[] = { t.BoundBox_.width / 2, 10, 10, t.BoundBox_.height / 2 };
        cv::Mat Mean(2, 1, CV_64FC1, &mean);
        cv::Mat Covariance(2, 2, CV_64FC1, &covariance);

        MathSample sample_math;
        cv::Mat Sample;
        if (sample_math.Initial(std::string("Gaussian"), std::string("RandomWalker")))
        {
          sample_math.SetMeanAndCovariance(Mean, Covariance, 50);
          sample_math.GenSamples();
          sample_math.FetchSamples(Sample);
        }

        std::shared_ptr<RenderObject> lines = std::make_shared<RenderLines>(im->frame.cols, im->frame.rows);
        lines->SetVertices(Sample);
        lines->SetID(std::to_string(t.ObjectIdx_));
        img->AddSubObj(lines);

        std::shared_ptr<RenderEllipse> ellipse = std::make_shared<RenderEllipse>(im->frame.cols, im->frame.rows);
        ellipse->SetID(std::to_string(t.ObjectIdx_));
        ellipse->SetEllipse(sample_math.GetCovEllipse());
        ellipse->SetStipple(true);
        img->AddSubObj(ellipse);
      }

      DataView_->Add_Obj(img);
    }

    DataView_->Render();

    pangolin::FinishFrame();
  }
}
