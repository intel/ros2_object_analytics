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

#include "control.hpp"

Control::Control()
{
  TRACE_INFO();
}

Control::~Control()
{
  TRACE_INFO();
}

bool Control::Initial(View::Ptr view, stream_device::Ptr stream_device)
{
  TRACE_INFO();

  DataView_ = view;
  StreamDev_ = stream_device;

  return true;
}
