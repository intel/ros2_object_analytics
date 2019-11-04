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

#include "tracker/tracking.hpp"

#include <string>
#include <utility>
#include <vector>
#include <memory>

#include "util/logger.hpp"

namespace tracker
{
const int32_t Tracking::kAgeingThreshold = 60;
const int32_t Tracking::kDetCountThreshold = 8;
const int32_t Tracking::kTrackLostThreshold = 3;
const int32_t Tracking::kTrajLength = 30;

#define DEBUG_ID 2

Tracking::Tracking(
  int32_t tracking_id, const std::string & name,
  const float & probability, const cv::Rect2d & rect)
: tracker_(cv::Ptr<TrackerKCFImpl>()),
  obj_name_(name),
  probability_(probability),
  tracking_id_(tracking_id),
  algo_("KCF"),
  state_(INIT)
{
  UNUSED(rect);
  UNUSED(name);
}

Tracking::~Tracking()
{
  trajVec_.clear();

  if (tracker_.get()) {
    tracker_.release();
  }
}

std::vector<Traj> Tracking::getTrajs() {return trajVec_;}

void Tracking::rectifyTracker(
  const std::shared_ptr<sFrame> frame,
  const cv::Rect2d & d_rect)
{
  if (tracker_.get()) {
    tracker_.release();
  }

  double lstamp = frame->stamp.tv_sec * 1e3 + frame->stamp.tv_nsec * 1e-6;
  UNUSED(lstamp);
  TRACE_INFO("Tracker(%d) rectify stamp(%f)", tracking_id_, lstamp);

  tracked_rect_ = d_rect;

  tracker_ = createTrackerByAlgo(algo_);
  tracker_->initImpl(frame->frame, tracked_rect_);
  cv::Mat covar = tracker_->getCovar().clone();

  cv::Mat initialCov = cv::Mat::eye(4, 4, CV_32F);
  initialCov.at<float>(0, 0) = covar.at<float>(0, 0);
  initialCov.at<float>(1, 1) = covar.at<float>(1, 1);
  initialCov.at<float>(2, 2) = 0;
  initialCov.at<float>(3, 3) = 0;

  cv::Mat state = cv::Mat::zeros(4, 1, CV_32F);
  state.at<float>(0) = static_cast<float>(d_rect.x) + d_rect.width / 2.0f;
  state.at<float>(1) = static_cast<float>(d_rect.y) + d_rect.height / 2.0f;

  kalman_.init(4, 2, 0, CV_32F);
  kalman_.initialParams(state, initialCov, frame->stamp);

  prediction_ = tracked_rect_;

  storeTraj(frame->stamp, prediction_, covar, frame->frame);

  ageing_ = 0;
  trackLost_ = 0;
  detCount_ = 0;
}

bool Tracking::detectTracker(const std::shared_ptr<sFrame> frame)
{
  double lstamp = frame->stamp.tv_sec * 1e3 + frame->stamp.tv_nsec * 1e-6;
  UNUSED(lstamp);
  TRACE_INFO("\nTracker(%d) detect stamp(%f)", tracking_id_, lstamp);

  cv::Mat bcentra = kalman_.predict(frame->stamp);
  prediction_.x = bcentra.at<float>(0) - prediction_.width / 2;
  prediction_.y = bcentra.at<float>(1) - prediction_.height / 2;

  tracked_rect_.x = bcentra.at<float>(0) - tracked_rect_.width / 2;
  tracked_rect_.y = bcentra.at<float>(1) - tracked_rect_.height / 2;

  TRACE_INFO("Tracker(%d), predict centra(%f, %f)", tracking_id_,
    bcentra.at<float>(0), bcentra.at<float>(1));

  bool ret =
    tracker_->detectImpl(frame->frame, tracked_rect_, probability_, false);
  if (ret) {
    cv::Mat bcentra = cv::Mat::zeros(2, 1, CV_32F);
    bcentra.at<float>(0) = tracked_rect_.x + tracked_rect_.width / 2;
    bcentra.at<float>(1) = tracked_rect_.y + tracked_rect_.height / 2;

    cv::Mat covar = tracker_->getCovar().clone();
    kalman_.correct(bcentra, covar);
    TRACE_INFO("Tracker(%d), correct centra(%f, %f)", tracking_id_,
      bcentra.at<float>(0), bcentra.at<float>(1));

    storeTraj(frame->stamp, prediction_, covar, frame->frame);

    clearTrackLost();

  } else {
    storeTraj(frame->stamp, prediction_, kalman_.measurementCovPre,
      frame->frame);

    TRACE_ERR("Tracker(%d) is missing!!!", tracking_id_);
  }

  ageing_++;

  return ret;
}

void Tracking::updateTracker(
  const std::shared_ptr<sFrame> frame,
  cv::Rect2d & boundingBox, float confidence, bool det)
{
  UNUSED(confidence);

  double lstamp = frame->stamp.tv_sec * 1e3 + frame->stamp.tv_nsec * 1e-6;
  UNUSED(lstamp);
  TRACE_INFO("Tracker(%d) update stamp(%f), det(%d)", tracking_id_, lstamp,
    det);

  if (det) {
    bool debug = (tracking_id_ == DEBUG_ID);
    debug = false;

    Traj traj;
    bool ret = getTraj(traj);
    if (!ret) {
      TRACE_INFO("Tracker(%d) update stamp(%f), det(%d), failed since of no base frame!!!!",
        tracking_id_, lstamp, det);
      state_ = LOST;
      return;
    }

    cv::Mat frame_latest = traj.frame_;
    ret =
      tracker_->updateWithDetectImpl(frame->frame, boundingBox, frame_latest,
        tracked_rect_, probability_, debug);

    if (!ret) {
      TRACE_INFO( "Tracker(%d) update stamp(%f), det(%d), failed since of match fail!!!!",
        tracking_id_, lstamp, det);

      state_ = LOST;
      return;
    }

    cv::Mat covar = tracker_->getCovar().clone();

    prediction_ = tracked_rect_;

    if (state_ == INIT) {
      cv::Mat bcentra = kalman_.predict(frame->stamp);
      prediction_.x = bcentra.at<float>(0) - prediction_.width / 2;
      prediction_.y = bcentra.at<float>(1) - prediction_.height / 2;
      bcentra.at<float>(0) = tracked_rect_.x + tracked_rect_.width / 2;
      bcentra.at<float>(1) = tracked_rect_.y + tracked_rect_.height / 2;
      kalman_.correct(bcentra, covar);
    }

    if (state_ == INIT) {
      storeTraj(frame->stamp, prediction_, covar, frame->frame);
    }
  } else {
    tracker_->updateWithTrackImpl(frame->frame, boundingBox, probability_,
      false);
  }
}

cv::Rect2d Tracking::getTrackedRect() {return tracked_rect_;}

cv::Rect2d Tracking::getPredictedRect() {return prediction_;}

bool Tracking::getTraj(timespec stamp, Traj & traj)
{
  bool ret = false;

  for (auto & rec : trajVec_) {
    if (rec.stamp_ == stamp) {
      traj = rec;
      return true;
    }
  }

  return ret;
}

bool Tracking::getTraj(Traj & traj)
{
  if (trajVec_.size() > 0) {
    traj = trajVec_.back();
    return true;

  } else {
    return false;
  }
}

void Tracking::storeTraj(
  timespec stamp, cv::Rect rect, cv::Mat & cov,
  cv::Mat frame)
{
  Traj traj(stamp, rect, cov, frame);

  trajVec_.push_back(traj);
  if (trajVec_.size() > kTrajLength) {trajVec_.erase(trajVec_.begin());}
}

std::string Tracking::getObjName() {return obj_name_;}

float Tracking::getObjProbability() {return probability_;}

int32_t Tracking::getTrackingId() {return tracking_id_;}

bool Tracking::isActive() {return state_ == ACTIVE || state_ == INACTIVE;}

bool Tracking::isAvailable() {return state_ != LOST;}

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

void Tracking::incDetCount()
{
  detCount_++;

  if (detCount_ > kDetCountThreshold) {
    detCount_ = kDetCountThreshold;

    if (state_ == INIT) {state_ = INACTIVE;}

  } else if (detCount_ > 0) {
    if (state_ == INACTIVE) {state_ = ACTIVE;}
  }

  clearTrackLost();
}

void Tracking::decDetCount()
{
  detCount_--;
  if (detCount_ < 0) {
    /*TBD: Long time tracker may still functional
     * and reliable when detection lost
     * */
    // if (state_ == INIT)
    state_ = LOST;
  }
}

void Tracking::clearTrackLost()
{
  trackLost_ = 0;

  if (state_ == INACTIVE) {
    state_ = ACTIVE;
  }
}

void Tracking::incTrackLost()
{
  trackLost_++;

  if (trackLost_ > kTrackLostThreshold) {
    state_ = LOST;
  } else {
    state_ = INACTIVE;
  }
}

cv::Ptr<TrackerKCFImpl> Tracking::createTrackerByAlgo(std::string name)
{
  UNUSED(name);

  cv::Ptr<TrackerKCFImpl> handler = new TrackerKCFImpl();
  return handler;
}

}  // namespace tracker
