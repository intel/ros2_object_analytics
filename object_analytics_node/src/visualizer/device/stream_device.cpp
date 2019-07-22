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
 
#include "stream_device.hpp"
#include "stream_cap.hpp"
#include "stream_ds.hpp"
#include "stream_vid.hpp"

stream_device::stream_device()
{
  TRACE_INFO();
}

stream_device::~stream_device()
{
  TRACE_INFO();

  release_stream();
}

stream_device::Ptr stream_device::create(int stream_name)
{
  TRACE_INFO();

  stream_device::Ptr stream_dev = nullptr;

  /*create stream device as camera(RTSP) device*/
  stream_dev = std::make_shared<stream_cap>();

  if (stream_dev != nullptr)
  {
    if (!stream_dev->init_stream(stream_name))
    {
      stream_dev.reset();
    }
    else
    {
      stream_dev->initialized_ = true;
    }
  }

  return stream_dev;
}

stream_device::Ptr stream_device::create(std::string& stream_name)
{
  TRACE_INFO();

  stream_device::Ptr stream_dev = nullptr;

  std::size_t found = stream_name.find("rtsp://");
  if (found != std::string::npos)
  {
    /*create stream device as camera(RTSP) device*/
    stream_dev = std::make_shared<stream_cap>();
  }
  else
  {
    found = stream_name.find("ds://");

    if (found != std::string::npos)
    {
      /*create stream device for dataset*/
      stream_dev = std::make_shared<stream_ds>();
    }
    else
    {
      /*create stream device for video file*/
      stream_dev = std::make_shared<stream_vid>();
    }
  }

  if (stream_dev != nullptr)
  {
    if (!stream_dev->init_stream(stream_name))
    {
      stream_dev.reset();
    }
    else
    {
      stream_dev->initialized_ = true;
    }
  }

  return stream_dev;
}

void stream_device::release_stream()
{
  TRACE_INFO();

  if (isAsync)
  {
    terminate = true;
    condVar.notify_one();
    if (workThread.joinable())
    {
      workThread.join();
    }
    else
    {
    }
  }
}

bool stream_device::process()
{
  TRACE_INFO();
  bool ret = false;

  if (initialized_ && isAsync)
  {
    terminate = false;
    workThread = std::thread([&]() {
      while (!terminate)
      {
        {
          std::shared_ptr<sFrame> frame;
          bool result = false;
          while (!((result = fetch_frame(frame)) || terminate))
          {
            TRACE_ERR("\t fetch frame failed!");

            std::unique_lock<std::mutex> lock(mutex);
            if (queue.empty() || queue.back().first)
            {
              queue.push({ false, frame });
              lock.unlock();
              hasFrame.notify_one();
              reset_stream();
              lock.lock();
            }

            std::chrono::milliseconds timeout(pollingTimeMSec);
            condVar.wait_for(lock, timeout, [&]() {
              terminate.store(true);
              return terminate.load();
            });
          }

          std::unique_lock<std::mutex> lock(mutex);
          condVar.wait(lock, [&]() { return queue.size() < queueSize || terminate; });

          if (queue.size() == queueSize)
            queue.pop();
          queue.push({ result, frame });
          TRACE_INFO("Stream PUSH, QUEUE SIZE(%ld)", queue.size());
        }
        hasFrame.notify_one();
      }
    });
    ret = true;
  }

  return ret;
}

bool stream_device::read(std::shared_ptr<sFrame>& frame)
{
  TRACE_INFO();
  if (isAsync)
  {
    size_t count = 0;
    bool res = false;
    {
      std::unique_lock<std::mutex> lock(mutex);
      hasFrame.wait(lock, [&]() { return !queue.empty() || terminate; });
      res = queue.front().first;
      frame = queue.front().second;
      if (queue.size() > 0)
      {
        queue.pop();
      }
      count = queue.size();
      TRACE_INFO("Stream POP, QUEUE SIZE(%ld)", count);
      (void)count;
    }
    condVar.notify_one();
    return res;
  }
  else
  {
    return fetch_frame(frame);
  }
}

bool stream_device::query(std::shared_ptr<sFrame>& frame)
{
  TRACE_INFO();
  if (isAsync)
  {
    size_t count = 0;
    bool res = false;
    {
      std::unique_lock<std::mutex> lock(mutex);
      hasFrame.wait(lock, [&]() { return !queue.empty() || terminate; });
      res = queue.front().first;
      frame = queue.front().second;
      count = queue.size();
      TRACE_INFO("Stream POP, QUEUE SIZE(%ld)", count);
      (void)count;
    }
    return res;
  }
  else
  {
    return fetch_frame(frame);
  }
}
