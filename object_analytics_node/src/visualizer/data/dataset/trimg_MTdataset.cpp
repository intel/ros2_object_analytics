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
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>
#include "track_dataset.hpp"

namespace datasets
{
#define TRACE_INFO

void imgMTDataset::load(const std::string& rootPath)
{
  std::string nameListPath = rootPath + "/list.txt";
  std::ifstream namesList(nameListPath.c_str());
  std::string datasetName;

  if (namesList.is_open())
  {
    int currDatasetID = 0;

    // All datasets/folders loop
    while (getline(namesList, datasetName))
    {
      // Open dataset config file
      std::string cfgFilePath = rootPath + "/" + datasetName + "/" + datasetName + ".yml";
      cv::FileStorage cfgFile(cfgFilePath, cv::FileStorage::READ);
      if (!cfgFile.isOpened())
      {
        TRACE_INFO("Error to open (%s)!!!\n", cfgFilePath.c_str());
        continue;
      }

      cv::Ptr<trImgMTObj> currObj(new trImgMTObj);

      // Get configurations
      currObj->attr.startFrame = static_cast<int>(cfgFile["start"]);
      currObj->attr.countBytes = static_cast<int>(cfgFile["count_bytes"]);
      currObj->attr.prefix = static_cast<std::string>(cfgFile["prefix"]);
      currObj->attr.suffix = static_cast<std::string>(cfgFile["suffix"]);
      currObj->gt_file = static_cast<std::string>(cfgFile["gt_file"]);
      currObj->det_file = static_cast<std::string>(cfgFile["det_file"]);

      // Open dataset's ground truth file
      std::string gtListPath = rootPath + "/" + datasetName + "/" + currObj->gt_file;
      cv::FileStorage gtList(gtListPath, cv::FileStorage::READ);
      if (!gtList.isOpened())
      {
        TRACE_INFO("Error to open (%s)!!!\n", gtListPath.c_str());
        continue;
      }
      cv::FileNode gtRoot = gtList["dataset"]["frame"];
      cv::FileNodeIterator gt_it = gtRoot.begin();
      cv::FileNodeIterator gt_end = gtRoot.end();

      // Open dataset's ground truth file
      std::string detListPath = rootPath + "/" + datasetName + "/" + currObj->det_file;
      cv::FileStorage detList(detListPath, cv::FileStorage::READ);
      if (!detList.isOpened())
      {
        TRACE_INFO("Error to open (%s)!!!\n", detListPath.c_str());
        continue;
      }
      cv::FileNode detRoot = detList["dataset"]["frame"];
      cv::FileNodeIterator dt_it = detRoot.begin();
      cv::FileNodeIterator dt_end = detRoot.end();

      int currFrameID = currObj->attr.startFrame;
      if (currDatasetID == 0)
      {
        TRACE_INFO("Dataset Initialization...\n");
      }
      bool trFLG = true;
      do
      {
        std::string fullPath = rootPath + "/" + datasetName + "/img/" + currObj->attr.prefix +
                               numberToString(currFrameID, currObj->attr.countBytes) + currObj->attr.suffix;
        if (!fileExists(fullPath))
        {
          break;
        }

        // Make images Object
        currObj->imagePath.push_back(fullPath);

        for (; gt_it != gt_end; gt_it++)
        {
          TRACE_INFO("gt frame num(%d)\n", static_cast<int>((*gt_it)["number"]));
          cv::FileNode obj_node = (*gt_it)["objectlist"]["object"];
          cv::FileNodeIterator obj_it = obj_node.begin();
          cv::FileNodeIterator obj_end = obj_node.end();
          std::vector<Obj_> obj_vec;
          for (; obj_it != obj_end; obj_it++)
          {
            TRACE_INFO("obj box idx(%d)\t", static_cast<int>((*obj_it)["id"]));
            TRACE_INFO("h(%f)\t", static_cast<float>((*obj_it)["box"]["h"]));
            TRACE_INFO("w(%f)\t", static_cast<float>((*obj_it)["box"]["w"]));
            TRACE_INFO("xc(%f)\t", static_cast<float>((*obj_it)["box"]["xc"]));
            TRACE_INFO("yc(%f)\n", static_cast<float>((*obj_it)["box"]["yc"]));
            int idx = static_cast<int>((*obj_it)["id"]);
            float h = static_cast<float>((*obj_it)["box"]["h"]);
            float w = static_cast<float>((*obj_it)["box"]["w"]);
            float xc = static_cast<float>((*obj_it)["box"]["xc"]) - w / 2;
            float yc = static_cast<float>((*obj_it)["box"]["yc"]) - h / 2;
            Obj_ obj = { idx, cv::Rect2d(xc, yc, w, h), 1.0f };
            obj_vec.push_back(obj);
          }
          currObj->gtbb.push_back(obj_vec);
        }

        for (; dt_it != dt_end; dt_it++)
        {
          TRACE_INFO("det frame num(%d)\n", static_cast<int>((*dt_it)["number"]));
          cv::FileNode obj_node = (*dt_it)["objectlist"]["object"];
          cv::FileNodeIterator obj_it = obj_node.begin();
          cv::FileNodeIterator obj_end = obj_node.end();
          std::vector<Obj_> obj_vec;
          for (; obj_it != obj_end; obj_it++)
          {
            TRACE_INFO("obj box confidence(%f)\t", static_cast<float>((*obj_it)["confidence"]));
            TRACE_INFO("obj box h(%f)\t", static_cast<float>((*obj_it)["box"]["h"]));
            TRACE_INFO("obj box w(%f)\t", static_cast<float>((*obj_it)["box"]["w"]));
            TRACE_INFO("obj box xc(%f)\t", static_cast<float>((*obj_it)["box"]["xc"]));
            TRACE_INFO("obj box yc(%f)\n", static_cast<float>((*obj_it)["box"]["yc"]));
            float confidence = static_cast<float>((*obj_it)["confidence"]);
            float h = static_cast<float>((*obj_it)["box"]["h"]);
            float w = static_cast<float>((*obj_it)["box"]["w"]);
            float xc = static_cast<float>((*obj_it)["box"]["xc"]) - w / 2;
            float yc = static_cast<float>((*obj_it)["box"]["yc"]) - h / 2;
            Obj_ obj = { 0, cv::Rect2d(xc, yc, w, h), confidence };
            obj_vec.push_back(obj);
          }
          currObj->detbb.push_back(obj_vec);
        }

        currFrameID++;
      } while (trFLG);

      currObj->dsName = datasetName;
      // TBD: get attributions from YAML config file
      currObj->attr.frameCount = currFrameID - currObj->attr.startFrame;

      data.push_back(currObj);
      currDatasetID++;
    }
  }
  else
  {
    TRACE_INFO("Couldn't find a *list.txt* in folder!!!");
  }

  namesList.close();
}

int imgMTDataset::getDatasetsNum()
{
  return static_cast<int>(data.size());
}

int imgMTDataset::getDatasetLength(int id)
{
  if (id > 0 && id <= static_cast<int>(data.size()))
  {
    return static_cast<int>(data[id - 1]->attr.frameCount);
  }
  else
  {
    TRACE_INFO("Dataset ID is out of range...\nAllowed IDs are: 1~%d\n", static_cast<int>(data.size()));
    return -1;
  }
}

bool imgMTDataset::initDataset(std::string dsName)
{
  int id = 0;
  frameIdx = 0;

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
    return true;
  }
  else
  {
    TRACE_INFO("Dataset ID is out of range...\nAllowed IDs are: 1~%d\n", static_cast<int>(data.size()));
    return false;
  }
}

bool imgMTDataset::getNextFrame(cv::Mat& frame)
{
  if (frameIdx >= static_cast<int>(data[activeDatasetID - 1]->attr.frameCount))
  {
    return false;
  }
  std::string imgPath = data[activeDatasetID - 1]->imagePath[frameIdx];
  frame = cv::imread(imgPath);
  frameIdx++;
  return !frame.empty();
}

bool imgMTDataset::getIdxFrame(cv::Mat& frame, int idx)
{
  if (idx >= static_cast<int>(data[activeDatasetID - 1]->attr.frameCount))
  {
    return false;
  }

  std::string imgPath = data[activeDatasetID - 1]->imagePath[idx - 1];
  frame = cv::imread(imgPath);
  return !frame.empty();
}

std::vector<std::vector<Obj_>> imgMTDataset::getGT()
{
  return data[activeDatasetID - 1]->gtbb;
}

std::vector<Obj_> imgMTDataset::getIdxGT(int idx)
{
  cv::Ptr<trImgMTObj> currObj = data[activeDatasetID - 1];
  return currObj->gtbb[idx - 1];
}

}  // namespace datasets
