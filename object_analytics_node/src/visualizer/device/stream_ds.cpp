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

#include "stream_ds.hpp"
#include "object.hpp"
#include "frame_obj.hpp"

stream_ds::stream_ds()
{
  TRACE_INFO();
}

stream_ds::~stream_ds()
{
  TRACE_INFO();

  stream_device::release_stream();
}

bool stream_ds::init_stream(int stream_name)
{
  TRACE_INFO();

  return false;
}

bool stream_ds::init_stream(std::string& stream_name)
{
  TRACE_INFO();

  int slashIndex = stream_name.find_last_of('/');

  std::string path = stream_name.substr(strlen("ds://"), slashIndex - strlen("ds://"));
  std::string dsName = stream_name.substr(slashIndex + 1, stream_name.length());
  TRACE_INFO("dataset path(%s)", path.c_str());
  TRACE_INFO("dataset name(%s)", dsName.c_str());

  ds_ = ds_->create(datasets::dsMTImage);
  ds_->load(path);

  if (ds_->initDataset(dsName))
  {
    return true;
  }

  return false;
}

void stream_ds::release_stream()
{
  TRACE_INFO();
}

bool stream_ds::reset_stream()
{
  TRACE_INFO();

  return false;
}

bool stream_ds::fetch_frame(std::shared_ptr<sFrame>& frame)
{
  TRACE_INFO();

  bool ret = false;

  if (ds_ == nullptr)
  {
    return false;
  }

  cv::Mat frame_img;
  ret = ds_->getNextFrame(frame_img);
  if (ret)
  {
    std::shared_ptr<FrameObjs> frameobj = std::make_shared<FrameObjs>();
    int frameId = ds_->getFrameIdx();
    frameobj->genFrame(frame_img, frameId);

    for (auto t : ds_->getIdxGT(frameId))
    {
      Object obj;
      obj.ObjectIdx_ = t.objIdx;
      obj.Confidence = t.confidence;
      obj.BoundBox_ = t.bb;

      frameobj->AddDetection(obj);
    }

    frame = frameobj;
  }
  else
  {
    TRACE_ERR("stream_ds fetch frame failed!!!");
  }

  return ret;
}
