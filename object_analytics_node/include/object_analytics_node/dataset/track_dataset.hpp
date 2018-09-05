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

#ifndef OBJECT_ANALYTICS_NODE__DATASET_HPP_
#define OBJECT_ANALYTICS_NODE__DATASET_HPP_

#include <string>
#include <vector>
#include <omp.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <sys/stat.h>

using namespace std;
using namespace cv;

namespace datasets
{

typedef struct {
	int startFrame;
	int frameCount;
	vector <int> omitFrames;
}attr_;

struct trImgObj
{
	string dsName;
    vector <std::string> imagePath;
    vector <Rect2d> gtbb;
	attr_ attr;
};

struct trVidObj
{
	string dsName;
    std::string vidPath;
    vector <Rect2d> gtbb;
	attr_ attr;
};

enum dsType
{
	dsVideo = 0,
	dsImage,
	dsInvalid
};

class trDataset 
{
public:
	Ptr<trDataset> create(dsType type);

    virtual void load(const std::string &rootPath)= 0;

    virtual int getDatasetsNum() = 0;

    virtual int getDatasetLength(int id) = 0;

    virtual bool initDataset(string dsName) = 0;

    virtual bool getNextFrame(Mat &frame) = 0;

    virtual bool getIdxFrame(Mat &frame, int idx) = 0;

    virtual vector <Rect2d> getGT() = 0;

    virtual Rect2d getIdxGT(int idx) = 0;

	virtual int getFrameIdx();

	inline bool fileExists(const std::string& name)
	{
		struct stat buffer;
		return (stat(name.c_str(), &buffer) == 0);
	}

protected:
    vector <int> datasetLength;
    int activeDatasetID;
	int frameIdx;
};


class vidDataset : public trDataset
{
public:
    virtual void load(const std::string &rootPath);

    virtual int getDatasetsNum();

    virtual int getDatasetLength(int id);

    virtual bool initDataset(string dsName);

    virtual bool getNextFrame(Mat &frame);

    virtual bool getIdxFrame(Mat &frame, int idx);

    virtual vector <Rect2d> getGT();

    virtual Rect2d getIdxGT(int idx);
protected:
	vector <Ptr<trVidObj> >  data;
	cv::VideoCapture cap;
};

class imgDataset : public trDataset
{
public:
    virtual void load(const std::string &rootPath);

    virtual int getDatasetsNum();

    virtual int getDatasetLength(int id);

    virtual bool initDataset(string dsName);

    virtual bool getNextFrame(Mat &frame);

    virtual bool getIdxFrame(Mat &frame, int idx);

    virtual vector <Rect2d> getGT();

    virtual Rect2d getIdxGT(int idx);

	string numberToString(int number)
	{
		string out;
		char numberStr[9];
		sprintf(numberStr, "%u", number);
		for (unsigned int i = 0; i < 4 - strlen(numberStr); ++i)
		{
			out += "0";
		}
		out += numberStr;
		return out;
	};
protected:
    vector <Ptr<trImgObj> > data;
};


}

#endif  // OBJECT_ANALYTICS_NODE__DATASET_HPP_
