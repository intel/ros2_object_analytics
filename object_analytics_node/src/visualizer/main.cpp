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

#include <stdio.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <string>
#include <memory>

#include "control_ds.hpp"
#include "stream_device.hpp"
#include "view.hpp"

static const char * keys = {
  "{@device_type | | camera or image files}"
  "{@device_path | | camera index or image path}"};

int main(int argc, char ** argv)
{
  /* stream device support camera/video and some image dataset format
  ** give some example of input to initialize input camera as below:
    1. Default camera
       int CAM = 0;
    2. Video files
      std::string vid_file = "/data/dataset/crossroad/TownCentreXVID.avi";
    3. Multi-tracking dataset
      std::string ds_file =
  "ds:///data/dataset/PETS2009/Crowd_PETS09/S2/L1/Time_12-34/View_001";
  */

  cv::CommandLineParser parser(argc, argv, keys);
  std::string SrcType = parser.get<std::string>(0);
  std::string FilePath = parser.get<std::string>(1);

  /*Just hack for convinience*/
  std::string ds_file =
    "ds:///data/dataset/PETS2009/Crowd_PETS09/S2/L1/Time_12-34/View_001";
  stream_device::Ptr inputCapture;

  if (SrcType == "Cap") {
    int CAM = 0;
    inputCapture = inputCapture->create(CAM);
  } else if (SrcType == "Vid") {
    inputCapture = inputCapture->create(ds_file);
  }

  // inputCapture = inputCapture->create(CAM);
  if (inputCapture == nullptr) {
    TRACE_ERR("camera/Images can not initialize");
    return 1;
  }

  View::Ptr scene = std::make_shared<View>();

  Control::Ptr control = std::make_shared<ControlDS>();

  control->Initial(scene, inputCapture);
  control->CreateContext();
  control->Run();

  return 0;
}
