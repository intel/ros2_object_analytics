/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <string>
#include <vector>
#include <omp.h>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include "rcutils/logging_macros.h"
#include "object_analytics_node/dataset/track_dataset.hpp"

namespace datasets
{

void vidDataset::load(const std::string &rootPath)
{
	string nameListPath = rootPath + "/list.txt";
	ifstream namesList(nameListPath.c_str());
	vector <int> datasetsLengths;
	string datasetName;

	if (namesList.is_open())
	{
		int currDatasetID = 0;

		//All datasets/folders loop
		while (getline(namesList, datasetName))
		{
			vector <Ptr<trVidObj> > objects;

			//All frames/images loop
			Ptr<trVidObj> currDataset(new trVidObj);

			//Open dataset's ground truth file
			string gtListPath = rootPath + "/" + datasetName + "/gt.txt";
			ifstream gtList(gtListPath.c_str());
			if (!gtList.is_open())
			{
				RCUTILS_LOG_DEBUG("Error to open (%s)!!!\n", gtListPath.c_str());
				continue;
			}

			if (currDatasetID == 0)
				RCUTILS_LOG_DEBUG("Video Dataset Initialization...\n");

			string fullPath = rootPath + "/" + datasetName + "/data/" + datasetName +".webm";
			if (!fileExists(fullPath))
			{
				RCUTILS_LOG_DEBUG("vid(%s) file is not exist\n", fullPath.c_str());
				continue;
			}


			//Open video config file
			string cfgFilePath = rootPath + "/" + datasetName + "/"+ datasetName +".yml";
			cv::FileStorage cfgFile(cfgFilePath, cv::FileStorage::READ);
			if (!cfgFile.isOpened())
			{
				RCUTILS_LOG_DEBUG("Error to open (%s)!!!\n", cfgFilePath.c_str());
				continue;
			}

			//Make video Object
			Ptr<trVidObj> currObj(new trVidObj);
			currObj->vidPath = fullPath;

			int currFrameID = 0;
			bool trFLG = true;
			do
			{
				//Get Ground Truth data
				Rect2d gt(0, 0, 0, 0);
				string tmp;
				getline(gtList, tmp);
				int ret = sscanf(tmp.c_str(), "%lf,%lf,%lf,%lf", &gt.x, &gt.y, &gt.width, &gt.height);
				if(ret > 0)
				{
					currObj->gtbb.push_back(gt);
					currFrameID++;
				} else {
					break;
				}
			} while (trFLG);

			//Get configurations
			currObj->attr.startFrame = (int)cfgFile["start"];

			gtList.close();

			currObj->dsName =  datasetName;
			//TBD: get attributions from YAML config file
			currObj->attr.frameCount =  currFrameID;

			//Add object to storage
			data.push_back(currObj);
			currDatasetID++;

		}
	}
	else
	{
		RCUTILS_LOG_DEBUG("Couldn't find a *list.txt* in video dataset!!!");
	}

	activeDatasetID = 1;
	namesList.close();
	return;

}

int vidDataset::getDatasetsNum()
{
	return (int)(data.size());
}

int vidDataset::getDatasetLength(int id)
{
	if (id > 0 && id <= (int)data.size())
		return (int)(data[id - 1]->attr.frameCount);
	else
	{
		RCUTILS_LOG_DEBUG("Dataset ID is out of range...\nAllowed IDs are: 1~%d\n", (int)data.size());
		return -1;
	}
}

bool vidDataset::initDataset(string dsName)
{
	int id = 0;

	for(auto t : data)
	{
		id++;
		if(t->dsName == dsName)
			break;
	}	

	if (id > 0 && id <= (int)data.size())
	{
		activeDatasetID = id;
		Ptr<trVidObj> trObj = data[activeDatasetID - 1]; 

		cap.open(trObj->vidPath);
		if (!cap.isOpened())
		{
			RCUTILS_LOG_DEBUG("Failed to open video filei\n");
			return false;
		}
		cap.set(cv::CAP_PROP_POS_FRAMES, trObj->attr.startFrame);

		frameIdx = trObj->attr.startFrame;
		return true;
	}
	else
	{
		RCUTILS_LOG_DEBUG("Dataset ID is out of range...\nAllowed IDs are: 1~%d\n", (int)data.size());
		return false;
	}
}

bool vidDataset::getNextFrame(Mat &frame)
{
	if (frameIdx >= (int)data[activeDatasetID - 1]->attr.frameCount)
		return false;

	cap >> frame;
	frameIdx++;
	return !frame.empty();
}

bool vidDataset::getIdxFrame(Mat &frame, int idx)
{
	if (idx >= (int)data[activeDatasetID - 1]->attr.frameCount)
		return false;

	cap.set(cv::CAP_PROP_POS_FRAMES, idx);
	cap >> frame;
    cap.set(cv::CAP_PROP_POS_FRAMES, frameIdx);

	return !frame.empty();
}

vector <Rect2d> vidDataset::getGT()
{
	return data[activeDatasetID - 1]->gtbb;
}

Rect2d vidDataset::getIdxGT(int idx)
{
	Ptr<trVidObj> currObj = data[activeDatasetID - 1];
	return currObj->gtbb[idx - currObj->attr.startFrame];
}

}
