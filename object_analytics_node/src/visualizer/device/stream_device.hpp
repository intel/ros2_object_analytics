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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "frame.hpp"
#include "utility.hpp"

class stream_device {
 public:
  stream_device();
  ~stream_device();

  /**
   * @brief A smart pointer containing stream device object
   */
  using Ptr = std::shared_ptr<stream_device>;

  /**
   * @brief A smart pointer to the const stream device object
   */
  using CPtr = std::shared_ptr<const stream_device>;

  /**
   * @brief Create stream device
   */
  stream_device::Ptr create(std::string &stream_name);

  /**
   * @brief Create stream device
   */
  stream_device::Ptr create(int stream_name);

 public:
  /**
   * @brief Init stream from camera(USB)
   */
  virtual bool init_stream(int stream_name) = 0;

  /**
   * @brief Init stream from camera(RTSP)/or video file
   */
  virtual bool init_stream(std::string &stream_name) = 0;

  /**
   * @brief release stream device/file
   */
  virtual void release_stream();

  /**
   * @brief release stream device/file
   */
  virtual bool reset_stream() = 0;

  /**
   * @brief Fetech frame from iniitialized device
   */
  virtual bool fetch_frame(std::shared_ptr<sFrame> &frame) = 0;

  /**
   * @brief Start fetch frames to queue in async mode
   */
  bool process();

  /**
   * @brief read frame from queue or device
   */
  bool read(std::shared_ptr<sFrame>& frame);

  /**
   * @brief get frame from queue or device but not pop
   */
  bool query(std::shared_ptr<sFrame>& frame);

 protected:
  /*TBD: consolidate to stream_params*/
  std::string stream_name_;
  const bool isAsync = true;

  std::shared_ptr<cv::VideoCapture> cap_;

  std::thread workThread;
  std::atomic_bool terminate = {false};
  std::string videoName;

  std::mutex mutex;
  std::condition_variable condVar;
  std::condition_variable hasFrame;
  std::queue<std::pair<bool, std::shared_ptr<sFrame>>> queue;

  bool realFps = false;

  const size_t queueSize = 1;
  const size_t pollingTimeMSec = 1000;

 private:
  bool initialized_ = false;
};
