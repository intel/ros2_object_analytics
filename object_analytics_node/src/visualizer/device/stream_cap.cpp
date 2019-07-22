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

#include "stream_cap.hpp"

stream_cap::stream_cap()
{
  TRACE_INFO();
}

stream_cap::~stream_cap()
{
  TRACE_INFO();
}

bool stream_cap::init_stream(int stream_name)
{
  TRACE_INFO();

  //	stream_name_ =  stream_name;
  cap_ = std::make_shared<cv::VideoCapture>(stream_name);

  if (cap_->isOpened())
    return true;

  return false;
}

bool stream_cap::init_stream(std::string& stream_name)
{
  TRACE_INFO();

  //	stream_name_ =  stream_name;
  cap_ = std::make_shared<cv::VideoCapture>(stream_name, cv::CAP_FFMPEG);

  if (cap_->isOpened())
    return true;

  return false;
}

bool stream_cap::reset_stream()
{
  TRACE_INFO();
  return true;

  if (cap_ != nullptr)
  {
    if (cap_->isOpened())
    {
      cap_->release();
    }
    cap_.reset();
  }
  init_stream(stream_name_);

  return true;
}

bool stream_cap::fetch_frame(std::shared_ptr<sFrame>& frame)
{
  TRACE_INFO();
  bool ret = false;

  if (cap_ == nullptr || !cap_->isOpened())
  {
    return false;
  }

  cv::Mat frame_cap;
  ret = cap_->read(frame_cap);

  frame = std::make_shared<sFrame>();
  frame->genFrame(frame_cap);

  return ret;
}
