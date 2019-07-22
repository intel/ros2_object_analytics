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

#include <omp.h>
#include <fstream>
#include <string>
#include <vector>
#include "track_dataset.hpp"

namespace datasets
{
void vidDataset::load(const std::string& rootPath)
{
  std::string nameListPath = rootPath + "/list.txt";
  std::ifstream namesList(nameListPath.c_str());
  std::vector<int> datasetsLengths;
  std::string datasetName;

  if (namesList.is_open())
  {
    int currDatasetID = 0;

    // All datasets/folders loop
    while (getline(namesList, datasetName))
    {
      std::vector<cv::Ptr<trVidObj>> objects;

      // All frames/images loop
      cv::Ptr<trVidObj> currDataset(new trVidObj);

      // Open dataset's ground truth file
      std::string gtListPath = rootPath + "/" + datasetName + "/gt.txt";
      std::ifstream gtList(gtListPath.c_str());
      if (!gtList.is_open())
      {
        TRACE_ERR("Error to open (%s)!!!\n", gtListPath.c_str());
        continue;
      }

      if (currDatasetID == 0)
      {
        TRACE_ERR("Video Dataset Initialization...\n");
      }

      std::string fullPath = rootPath + "/" + datasetName + "/data/" + datasetName + ".webm";
      if (!fileExists(fullPath))
      {
        TRACE_ERR("vid(%s) file is not exist\n", fullPath.c_str());
        continue;
      }

      // Open video config file
      std::string cfgFilePath = rootPath + "/" + datasetName + "/" + datasetName + ".yml";
      cv::FileStorage cfgFile(cfgFilePath, cv::FileStorage::READ);
      if (!cfgFile.isOpened())
      {
        TRACE_ERR("Error to open (%s)!!!\n", cfgFilePath.c_str());
        continue;
      }

      // Make video Object
      cv::Ptr<trVidObj> currObj(new trVidObj);
      currObj->vidPath = fullPath;

      int currFrameID = 0;
      bool trFLG = true;
      do
      {
        // Get Ground Truth data
        std::vector<Obj_> obj_vec;
        Obj_ obj = { 0, cv::Rect2d(0, 0, 0, 0), 1.0f };
        std::string tmp;
        getline(gtList, tmp);
        int ret = sscanf(tmp.c_str(), "%lf,%lf,%lf,%lf", &obj.bb.x, &obj.bb.y, &obj.bb.width, &obj.bb.height);
        if (ret > 0)
        {
          obj_vec.push_back(obj);
          currObj->gtbb.push_back(obj_vec);
          currFrameID++;
        }
        else
        {
          break;
        }
      } while (trFLG);

      // Get configurations
      currObj->attr.startFrame = static_cast<int>(cfgFile["start"]);

      gtList.close();

      currObj->dsName = datasetName;
      // TBD: get attributions from YAML config file
      currObj->attr.frameCount = currFrameID;

      // Add object to storage
      data.push_back(currObj);
      currDatasetID++;
    }
  }
  else
  {
    TRACE_ERR("Couldn't find a *list.txt* in video dataset!!!");
  }

  activeDatasetID = 1;
  namesList.close();
}

int vidDataset::getDatasetsNum()
{
  return static_cast<int>(data.size());
}

int vidDataset::getDatasetLength(int id)
{
  if (id > 0 && id <= static_cast<int>(data.size()))
  {
    return static_cast<int>(data[id - 1]->attr.frameCount);
  }
  else
  {
    TRACE_ERR("Dataset ID is out of range...\nAllowed IDs are: 1~%d\n", static_cast<int>(data.size()));
    return -1;
  }
}

bool vidDataset::initDataset(std::string dsName)
{
  int id = 0;

  for (auto t : data)
  {
    id++;
    if (t->dsName == dsName)
    {
      break;
    }
  }

  if (id > 0 && id <= static_cast<int>(data.size()))
  {
    activeDatasetID = id;
    cv::Ptr<trVidObj> trObj = data[activeDatasetID - 1];

    cap.open(trObj->vidPath);
    if (!cap.isOpened())
    {
      TRACE_ERR("Failed to open video filei\n");
      return false;
    }
    cap.set(cv::CAP_PROP_POS_FRAMES, trObj->attr.startFrame);

    frameIdx = trObj->attr.startFrame;
    return true;
  }
  else
  {
    TRACE_ERR("Dataset ID is out of range...\nAllowed IDs are: 1~%d\n", static_cast<int>(data.size()));
    return false;
  }
}

bool vidDataset::getNextFrame(cv::Mat& frame)
{
  if (frameIdx >= static_cast<int>(data[activeDatasetID - 1]->attr.frameCount))
  {
    return false;
  }

  cap >> frame;
  frameIdx++;
  return !frame.empty();
}

bool vidDataset::getIdxFrame(cv::Mat& frame, int idx)
{
  if (idx >= static_cast<int>(data[activeDatasetID - 1]->attr.frameCount))
  {
    return false;
  }

  cap.set(cv::CAP_PROP_POS_FRAMES, idx);
  cap >> frame;
  cap.set(cv::CAP_PROP_POS_FRAMES, frameIdx);

  return !frame.empty();
}

std::vector<std::vector<Obj_>> vidDataset::getGT()
{
  return data[activeDatasetID - 1]->gtbb;
}

std::vector<Obj_> vidDataset::getIdxGT(int idx)
{
  cv::Ptr<trVidObj> currObj = data[activeDatasetID - 1];
  return currObj->gtbb[idx - currObj->attr.startFrame];
}

}  // namespace datasets
