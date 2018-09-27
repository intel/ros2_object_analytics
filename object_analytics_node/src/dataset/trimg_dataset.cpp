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

#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <omp.h>
#include <fstream>
#include <string>
#include <vector>
#include "object_analytics_node/dataset/track_dataset.hpp"
#include "rcutils/logging_macros.h"

namespace datasets
{

void imgDataset::load(const std::string & rootPath)
{
  std::string nameListPath = rootPath + "/list.txt";
  std::ifstream namesList(nameListPath.c_str());
  std::string datasetName;

  if (namesList.is_open()) {
    int currDatasetID = 0;

    // All datasets/folders loop
    while (getline(namesList, datasetName)) {
      // Open dataset's ground truth file
      std::string gtListPath =
        rootPath + "/" + datasetName + "/groundtruth_rect.txt";
      std::ifstream gtList(gtListPath.c_str());
      if (!gtList.is_open()) {
        RCUTILS_LOG_DEBUG("Error to open (%s)!!!\n", gtListPath.c_str());
        continue;
      }

      cv::Ptr<trImgObj> currObj(new trImgObj);

      int currFrameID = 0;
      if (currDatasetID == 0) {RCUTILS_LOG_DEBUG("Dataset Initialization...\n");}
      bool trFLG = true;
      do {
        currFrameID++;
        std::string fullPath = rootPath + "/" + datasetName + "/img/" +
          numberToString(currFrameID) + ".jpg";
        if (!fileExists(fullPath)) {break;}

        // Make images Object
        currObj->imagePath.push_back(fullPath);

        // Get Ground Truth data
        cv::Rect2d gt(0, 0, 0, 0);
        std::string tmp;
        getline(gtList, tmp);
        int ret =
          sscanf(tmp.c_str(), "%lf%*[ \t,]%lf%*[ \t,]%lf%*[ \t,]%lf%*[ \t,]",
            &gt.x, &gt.y, &gt.width, &gt.height);
        if (ret > 0) {
          currObj->gtbb.push_back(gt);
        } else {
          break;
        }
      } while (trFLG);

      gtList.close();

      currObj->dsName = datasetName;
      // TBD: get attributions from YAML config file
      currObj->attr.frameCount = currFrameID;
      currObj->attr.startFrame = 1;

      data.push_back(currObj);
      currDatasetID++;
    }
  } else {
    RCUTILS_LOG_DEBUG("Couldn't find a *list.txt* in folder!!!");
  }

  namesList.close();
}

int imgDataset::getDatasetsNum() {return static_cast<int>(data.size());}

int imgDataset::getDatasetLength(int id)
{
  if (id > 0 && id <= static_cast<int>(data.size())) {
    return static_cast<int>(data[id - 1]->attr.frameCount);
  } else {
    RCUTILS_LOG_DEBUG("Dataset ID is out of range...\nAllowed IDs are: 1~%d\n",
      static_cast<int>(data.size()));
    return -1;
  }
}

bool imgDataset::initDataset(std::string dsName)
{
  int id = 0;
  frameIdx = 0;

  for (auto t : data) {
    id++;
    if (t->dsName == dsName) {break;}
  }

  if (id > 0 && id <= static_cast<int>(data.size())) {
    activeDatasetID = id;
    return true;
  } else {
    RCUTILS_LOG_DEBUG("Dataset ID is out of range...\nAllowed IDs are: 1~%d\n",
      static_cast<int>(data.size()));
    return false;
  }
}

bool imgDataset::getNextFrame(cv::Mat & frame)
{
  if (frameIdx >= static_cast<int>(data[activeDatasetID - 1]->attr.frameCount)) {
    return false;
  }
  std::string imgPath = data[activeDatasetID - 1]->imagePath[frameIdx];
  frame = cv::imread(imgPath);
  frameIdx++;
  return !frame.empty();
}

bool imgDataset::getIdxFrame(cv::Mat & frame, int idx)
{
  if (idx >= static_cast<int>(data[activeDatasetID - 1]->attr.frameCount)) {
    return false;
  }

  std::string imgPath = data[activeDatasetID - 1]->imagePath[idx - 1];
  frame = cv::imread(imgPath);
  return !frame.empty();
}

std::vector<cv::Rect2d> imgDataset::getGT()
{
  return data[activeDatasetID - 1]->gtbb;
}

cv::Rect2d imgDataset::getIdxGT(int idx)
{
  cv::Ptr<trImgObj> currObj = data[activeDatasetID - 1];
  return currObj->gtbb[idx - currObj->attr.startFrame];
}

}  // namespace datasets
