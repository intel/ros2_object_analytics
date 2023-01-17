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

#include "stream_vid.hpp"

#include <string>
#include <memory>

stream_vid::stream_vid() {TRACE_FUNC();}

stream_vid::~stream_vid() {TRACE_FUNC();}

bool stream_vid::init_stream(int stream_name)
{
  TRACE_FUNC();

  return false;  // for fake test
}

bool stream_vid::init_stream(std::string & stream_name)
{
  TRACE_FUNC();

  cap_ = std::make_shared<cv::VideoCapture>(stream_name);

  if (cap_->isOpened()) {
    return true;
  }

  return false;  // for fake test
}

void stream_vid::release_stream()
{
  TRACE_FUNC();

  stream_device::release_stream();

  if (cap_ != nullptr) {
    if (cap_->isOpened()) {
      cap_->release();
    }
    cap_.reset();
  }
}

bool stream_vid::reset_stream()
{
  TRACE_FUNC();

  if (cap_->isOpened()) {
    return cap_->set(cv::CAP_PROP_POS_FRAMES, 1);
  }

  return false;
}

bool stream_vid::fetch_frame(std::shared_ptr<sFrame> & frame)
{
  TRACE_FUNC();
  bool ret = false;

  if (cap_ == nullptr || !cap_->isOpened()) {
    return false;
  }

  cv::Mat frame_cap;
  ret = cap_->read(frame_cap);
  frame = std::make_shared<sFrame>();
  frame->genFrame(frame_cap);

  return ret;
}
