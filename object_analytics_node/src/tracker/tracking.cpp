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

#include <vector>
#include <utility>
#include <string>
#include "object_analytics_node/tracker/tracking.hpp"

namespace object_analytics_node
{
namespace tracker
{
const int32_t Tracking::kAgeingThreshold = 60;

Tracking::Tracking(
  int32_t tracking_id, const std::string & name,
  const float & probability, const cv::Rect2d & rect)
: tracker_(cv::Ptr<cv::Tracker>()),
  tracked_rect_(rect),
  obj_name_(name),
  probability_(probability),
  tracking_id_(tracking_id),
  detected_(false),
  detect_mis_(0),
  algo_("MEDIAN_FLOW") {}

Tracking::~Tracking()
{
  if (tracker_.get()) {
    tracker_.release();
  }

  clearHistory();
}

void Tracking::rectifyTracker(
  const cv::Mat & mat, const cv::Rect2d & t_rect,
  const cv::Rect2d & d_rect,
  builtin_interfaces::msg::Time stamp)
{
  if (tracker_.get()) {
    tracker_.release();
  }

  clearHistory();

  tracker_ = createTrackerByAlgo(algo_);
  tracker_->init(mat, t_rect);
  tracked_rect_ = t_rect;
  detected_rect_ = d_rect;

  collectHistory(stamp, t_rect);
}

bool Tracking::updateTracker(
  const cv::Mat & mat,
  builtin_interfaces::msg::Time stamp)
{
  bool ret = tracker_->update(mat, tracked_rect_);

  if (ret) {collectHistory(stamp, tracked_rect_);}

  ageing_++;
  return ret;
}

cv::Rect2d Tracking::getTrackedRect() {return tracked_rect_;}

bool Tracking::getHisTrackedRect(
  builtin_interfaces::msg::Time stamp,
  cv::Rect2d & t_rect)
{
  std::vector<std::pair<builtin_interfaces::msg::Time, cv::Rect2d>>::iterator
    iter = hisCor_.begin();
  while (iter != hisCor_.end()) {
    if (iter->first == stamp) {
      t_rect = iter->second;
      return true;
    }
    iter++;
  }

  RCUTILS_LOG_DEBUG("Fail to get trect(%ld)", hisCor_.size());
  return false;
}

std::string Tracking::getObjName() {return obj_name_;}

float Tracking::getObjProbability() {return probability_;}

cv::Rect2d Tracking::getDetectedRect() {return detected_rect_;}

int32_t Tracking::getTrackingId() {return tracking_id_;}

bool Tracking::isActive()
{
  return (ageing_ < kAgeingThreshold) || (detect_mis_ > -30);
}

bool Tracking::isDetected() {return detected_;}

void Tracking::clearDetected()
{
  detected_ = false;
  detect_mis_--;
}

void Tracking::setDetected()
{
  ageing_ = 0;
  detected_ = true;
  detect_mis_ = 0;
}

void Tracking::collectHistory(
  builtin_interfaces::msg::Time stamp,
  cv::Rect2d t_rect)
{
  hisCor_.push_back(std::make_pair(stamp, t_rect));
  if (hisCor_.size() > 30) {hisCor_.erase(hisCor_.begin());}
}

void Tracking::clearHistory()
{
  std::vector<std::pair<builtin_interfaces::msg::Time, cv::Rect2d>>::iterator
    iter = hisCor_.begin();

  while (hisCor_.size() > 0) {iter = hisCor_.erase(iter);}
}

std::string Tracking::getAlgo() {return algo_;}

bool Tracking::setAlgo(std::string algo)
{
  if (algo == "KCF" || algo == "TLD" || algo == "BOOSTING" ||
    algo == "MEDIAN_FLOW" || algo == "MIL" || algo == "GOTURN")
  {
    algo_ = algo;
    return true;
  }

  return false;
}

bool Tracking::checkTimeZone(builtin_interfaces::msg::Time stamp)
{
  std::vector<std::pair<builtin_interfaces::msg::Time, cv::Rect2d>>::iterator
    iter = hisCor_.begin();
  while (iter != hisCor_.end()) {
    if (iter->first == stamp) {return true;}

    if (iter->first.sec < stamp.sec) {
      return true;
    } else if (iter->first.sec == stamp.sec) {
      if (iter->first.nanosec < stamp.nanosec) {
        return true;
      }
    }
    iter++;
  }

  return false;
}

#if CV_VERSION_MINOR == 2
cv::Ptr<cv::Tracker> Tracking::createTrackerByAlgo(std::string name)
{
  return cv::Tracker::create(name);
}
#else
cv::Ptr<cv::Tracker> Tracking::createTrackerByAlgo(std::string name)
{
  cv::Ptr<cv::Tracker> tracker;

  if (name == "KCF") {
    tracker = cv::TrackerKCF::create();
  } else if (name == "TLD") {
    tracker = cv::TrackerTLD::create();
  } else if (name == "BOOSTING") {
    tracker = cv::TrackerBoosting::create();
  } else if (name == "MEDIAN_FLOW") {
    tracker = cv::TrackerMedianFlow::create();
  } else if (name == "MIL") {
    tracker = cv::TrackerMIL::create();
  } else if (name == "GOTURN") {
    tracker = cv::TrackerGOTURN::create();
  } else {
    CV_Error(cv::Error::StsBadArg, "Invalid tracking algorithm name\n");
  }

  return tracker;
}
#endif

}  // namespace tracker
}  // namespace object_analytics_node
