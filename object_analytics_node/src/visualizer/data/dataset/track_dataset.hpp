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

#ifndef OBJECT_ANALYTICS_NODE__DATASET__TRACK_DATASET_HPP_
#define OBJECT_ANALYTICS_NODE__DATASET__TRACK_DATASET_HPP_

#include <omp.h>
#include <sys/stat.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <vector>

#include "utility.hpp"

#define MAX_IMG_BYTES 4

namespace datasets {

typedef struct {
  int startFrame;
  int frameCount;
  int countBytes;
  std::string prefix;
  std::string suffix;
  std::vector<int> omitFrames;
} attr_;

typedef struct {
  int objIdx;
  cv::Rect2d bb;
  float confidence;
} Obj_;

struct trImgObj {
  std::string dsName;
  std::vector<std::string> imagePath;
  std::vector<std::vector<Obj_>> gtbb;
  attr_ attr;
};

struct trVidObj {
  std::string dsName;
  std::string vidPath;
  std::vector<std::vector<Obj_>> gtbb;
  attr_ attr;
};

struct trImgMTObj {
  std::string dsName;
  std::vector<std::string> imagePath;
  std::string gt_file;
  std::string det_file;
  std::vector<std::vector<Obj_>> gtbb;
  std::vector<std::vector<Obj_>> detbb;
  attr_ attr;
};

enum dsType { dsSTVideo = 0, dsSTImage, dsMTVideo, dsMTImage, dsInvalid };

class trDataset {
 public:
  cv::Ptr<trDataset> create(dsType type);

  virtual void load(const std::string& rootPath) = 0;

  virtual int getDatasetsNum() = 0;

  virtual int getDatasetLength(int id) = 0;

  virtual bool initDataset(std::string dsName) = 0;

  virtual bool getNextFrame(cv::Mat& frame) = 0;

  virtual bool getIdxFrame(cv::Mat& frame, int idx) = 0;

  virtual std::vector<std::vector<Obj_>> getGT() = 0;

  virtual std::vector<Obj_> getIdxGT(int idx) = 0;

  virtual int getFrameIdx();

  inline bool fileExists(const std::string& name) {
    struct stat buffer;
    return stat(name.c_str(), &buffer) == 0;
  }

 protected:
  std::vector<int> datasetLength;
  int activeDatasetID;
  int frameIdx;
};

class vidDataset : public trDataset {
 public:
  virtual void load(const std::string& rootPath);

  virtual int getDatasetsNum();

  virtual int getDatasetLength(int id);

  virtual bool initDataset(std::string dsName);

  virtual bool getNextFrame(cv::Mat& frame);

  virtual bool getIdxFrame(cv::Mat& frame, int idx);

  virtual std::vector<std::vector<Obj_>> getGT();

  virtual std::vector<Obj_> getIdxGT(int idx);

 protected:
  std::vector<cv::Ptr<trVidObj>> data;
  cv::VideoCapture cap;
};

class imgDataset : public trDataset {
 public:
  virtual void load(const std::string& rootPath);

  virtual int getDatasetsNum();

  virtual int getDatasetLength(int id);

  virtual bool initDataset(std::string dsName);

  virtual bool getNextFrame(cv::Mat& frame);

  virtual bool getIdxFrame(cv::Mat& frame, int idx);

  virtual std::vector<std::vector<Obj_>> getGT();

  virtual std::vector<Obj_> getIdxGT(int idx);

  std::string numberToString(int number) {
    std::string out;
    char numberStr[9];
    snprintf(numberStr, MAX_IMG_BYTES, "%u", number);
    for (unsigned int i = 0; i < MAX_IMG_BYTES - strlen(numberStr); ++i) {
      out += "0";
    }
    out += numberStr;
    return out;
  }

 protected:
  std::vector<cv::Ptr<trImgObj>> data;
};

class imgMTDataset : public trDataset {
 public:
  virtual void load(const std::string& rootPath);

  virtual int getDatasetsNum();

  virtual int getDatasetLength(int id);

  virtual bool initDataset(std::string dsName);

  virtual bool getNextFrame(cv::Mat& frame);

  virtual bool getIdxFrame(cv::Mat& frame, int idx);

  virtual std::vector<std::vector<Obj_>> getGT();

  virtual std::vector<Obj_> getIdxGT(int idx);

  std::string numberToString(int number, int count_bytes) {
    std::string out;
    char numberStr[9];
    snprintf(numberStr, count_bytes, "%u", number);
    for (unsigned int i = 0; i < count_bytes - strlen(numberStr); ++i) {
      out += "0";
    }
    out += numberStr;
    return out;
  }

 protected:
  std::vector<cv::Ptr<trImgMTObj>> data;
};

}  // namespace datasets

#endif  // OBJECT_ANALYTICS_NODE__DATASET__TRACK_DATASET_HPP_ _
