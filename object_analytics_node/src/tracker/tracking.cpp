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
#include "tracker/tracking.hpp"

namespace tracker
{
const int32_t Tracking::kAgeingThreshold = 60;
const int32_t Tracking::kDetLostThreshold = 3;
const int32_t Tracking::kTrackLostThreshold = 2;

#define DEBUG_ID 2

Tracking::Tracking(
  int32_t tracking_id, const std::string & name,
  const float & probability, const cv::Rect2d & rect)
: tracker_(cv::Ptr<TrackerKCFImpl>()),
  obj_name_(name),
  probability_(probability),
  tracking_id_(tracking_id),
  algo_("MEDIAN_FLOW")
{

}

Tracking::~Tracking()
{
  trajVec_.clear();

  if (tracker_.get()) {
    tracker_.release();
  }
}

std::vector<Traj> Tracking::getTrajs()
{
  return trajVec_;
}

void Tracking::rectifyTracker(
  const std::shared_ptr<sFrame> frame, const cv::Rect2d &d_rect)
{

  if (tracker_.get())
  {
    tracker_.release();
  }

  double lstamp = frame->stamp.tv_sec*1e3 + frame->stamp.tv_nsec*1e-6;

  TRACE_INFO("Tracker(%d) rectify stamp(%f)",tracking_id_, lstamp);

  tracked_rect_ = d_rect;

  tracker_ = createTrackerByAlgo(algo_);
  tracker_->initImpl(frame->frame, tracked_rect_);
  cv::Mat covar = tracker_->getCovar();


  cv::Mat initialCov  = cv::Mat::eye(4, 4, CV_32F);
  initialCov.at<float>(0, 0) = covar.at<float>(0, 0);
  initialCov.at<float>(1, 1) = covar.at<float>(1, 1);
  initialCov.at<float>(2, 2) = 2*covar.at<float>(0, 0);
  initialCov.at<float>(3, 3) = 2*covar.at<float>(1, 1);

  cv::Mat state = cv::Mat::zeros(4, 1, CV_32F);
  state.at<float>(0) = (float)d_rect.x + d_rect.width/2.0f - 1;
  state.at<float>(1) = (float)d_rect.y + d_rect.height/2.0f - 1;

 // initKalmanFilter(4, 2, 0, tracked_rect_, frame->stamp);
  kalman_.init(4, 2, 0, CV_32F);
  kalman_.initialParams(state, initialCov, frame->stamp);

  prediction_ = tracked_rect_;

  trajVec_.push_back(Traj(frame->stamp, prediction_, covar, frame->frame));

  ageing_ = 0;
  trackLost_ = 0;
  detLost_ = 0;
}

bool Tracking::detectTracker(const std::shared_ptr<sFrame> frame)
{
  double lstamp = frame->stamp.tv_sec*1e3 + frame->stamp.tv_nsec*1e-6;
  TRACE_INFO("\nTracker(%d) detect stamp(%f)",tracking_id_, lstamp);

  cv::Mat bcentra = kalman_.predict(frame->stamp);
  prediction_.x = bcentra.at<float>(0) - prediction_.width/2 + 1;
  prediction_.y = bcentra.at<float>(1) - prediction_.height/2 + 1;;
  TRACE_INFO("Tracker(%d), (%f, %f), predict centra", tracking_id_, bcentra.at<float>(0), bcentra.at<float>(1));

  bool debug = (tracking_id_ == DEBUG_ID);
  debug = true;
  bool ret = tracker_->detectImpl(frame->frame, prediction_, probability_, debug);

  if (ret)
  {
    cv::Mat bcentra = cv::Mat::zeros(2, 1, CV_32F);
    bcentra.at<float>(0) = prediction_.x + prediction_.width/2 - 1;
    bcentra.at<float>(1) = prediction_.y + prediction_.height/2 - 1;

    cv::Mat covar = tracker_->getCovar();
    kalman_.correct(bcentra, covar);
    TRACE_INFO("Tracker(%d), (%f, %f), correct centra", tracking_id_, bcentra.at<float>(0), bcentra.at<float>(1));

    Traj traj(frame->stamp, prediction_, covar, frame->frame);
    trajVec_.push_back(traj);
    if (trajVec_.size() > 5)
      trajVec_.erase(trajVec_.begin());

    tracked_rect_ = prediction_;

    trackLost_ = 0;
  } else {

    Traj traj(frame->stamp, prediction_, kalman_.measurementCovPre, frame->frame);
    trajVec_.push_back(traj);
    if (trajVec_.size() > 5)
      trajVec_.erase(trajVec_.begin());

    trackLost_++;

    TRACE_ERR("Tracker(%d) is missing!!!", tracking_id_);

  }

  ageing_++;

  return ret;
}

void Tracking::updateTracker(const std::shared_ptr<sFrame> frame, Rect2d& boundingBox,
                             Mat &covar, float confidence, bool det)
{
  double lstamp = frame->stamp.tv_sec*1e3 + frame->stamp.tv_nsec*1e-6;
  TRACE_INFO("Tracker(%d) update stamp(%f), det(%d)",tracking_id_, lstamp, det);

  if (det)
  {   
    bool debug = (tracking_id_ == DEBUG_ID);
    debug = false;

    tracker::Traj traj = trajVec_.back();

#if 1
	  bool ret = tracker_->updateWithDetectImpl(frame->frame, boundingBox, traj.frame_, tracked_rect_, covar, probability_, debug);
    if (ret)
      prediction_ = tracked_rect_;
    else
      tracked_rect_ = prediction_;
#endif
  }
  else
  {
#if 1
    bool debug = (tracking_id_ == DEBUG_ID);
    debug = false;
    tracker_->updateWithTrackImpl(frame->frame, boundingBox, covar, probability_, debug);
#endif
  }
}

cv::Rect2d Tracking::getTrackedRect()
{
  return tracked_rect_;
}

cv::Rect2d Tracking::getPredictedRect()
{
  return prediction_;
 // return estimation_;
}

bool Tracking::getTraj(timespec stamp, Traj& traj)
{
  bool ret = false;

  for(auto &rec : trajVec_)
  {
    if (rec.stamp_ == stamp)
    {
      traj = rec;
      return true;
    }
  }

  return ret;
}

std::string Tracking::getObjName()
{
  return obj_name_;
}

float Tracking::getObjProbability()
{
  return probability_;
}

int32_t Tracking::getTrackingId()
{
  return tracking_id_;
}

bool Tracking::isActive()
{
  return (detLost_ < kDetLostThreshold &&
          trackLost_ < kTrackLostThreshold);
}

std::string Tracking::getAlgo()
{
  return algo_;
}

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

void Tracking::incDetLost()
{
  detLost_++;
}

void Tracking::clearDetLost()
{
  detLost_ = 0;
}

cv::Ptr<TrackerKCFImpl> Tracking::createTrackerByAlgo(std::string name)
{
  cv::Ptr<TrackerKCFImpl> handler = new TrackerKCFImpl();
  return handler;
}

#if 0
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
#endif

}  // namespace tracker
