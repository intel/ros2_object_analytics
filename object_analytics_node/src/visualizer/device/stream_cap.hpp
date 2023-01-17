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

#ifndef VISUALIZER__DEVICE__STREAM_CAP_HPP_
#define VISUALIZER__DEVICE__STREAM_CAP_HPP_

#include <opencv2/opencv.hpp>

#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <random>
#include <string>
#include <vector>
#include <memory>

#include "stream_device.hpp"

class stream_cap : public stream_device
{
public:
  stream_cap();
  ~stream_cap();

  /**
   * @brief Init stream from camera(USB)
   */
  virtual bool init_stream(int stream_name);
  /**
   * @brief Init stream from camera(RTSP)/or video file
   */
  virtual bool init_stream(std::string & stream_name);

  /**
   * @brief release stream device/file
   */
  virtual bool reset_stream();

  /**
   * @brief Fetech frame from iniitialized camera(RTSP) stream
   */
  virtual bool fetch_frame(std::shared_ptr<sFrame> & frame);

protected:
private:
};
#endif  // VISUALIZER__DEVICE__STREAM_CAP_HPP_
