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

namespace tracker
{
const int32_t Tracking::kAgeingThreshold = 60;

Tracking::Tracking(
  int32_t tracking_id, const std::string & name,
  const float & probability, const cv::Rect2d & rect)
: tracker_(cv::Ptr<cv::Tracker>()),
  obj_name_(name),
  probability_(probability),
  tracking_id_(tracking_id),
  initial_kf_state_(false),
  algo_("MEDIAN_FLOW")
{

}

Tracking::~Tracking()
{
  kf_vec_.clear();

  if (tracker_.get()) {
    tracker_.release();
  }
}

void Tracking::rectifyTracker(
  const std::shared_ptr<sFrame> frame, const cv::Rect2d &d_rect)
{
  double lstamp = frame->stamp.tv_sec*1e3 + frame->stamp.tv_nsec*1e-6;

  if (tracker_.get())
  {
    tracker_.release();
  }

  TRACE_INFO("Tracker(%d) rectify stamp(%ld)",tracking_id_, lstamp);

  tracker_ = createTrackerByAlgo(algo_);
  tracker_->init(frame->frame, d_rect);
  tracked_rect_ = d_rect;

  initKalmanFilter(kf_, 4, 2, 0, d_rect, frame->stamp);
}

bool Tracking::updateTracker(const std::shared_ptr<sFrame> frame)
{
  bool ret = tracker_->update(frame->frame, tracked_rect_);

  if (!initial_kf_state_ && !ret)
  {
    return false;
  }

  kf_configInterval(kf_, frame->stamp);
  kf_predict(kf_, prediction_);

  if (ret)
  {
    kf_update(kf_, tracked_rect_, frame->stamp);

    if (!initial_kf_state_)
    {
      initial_kf_state_ = true;
    }

  } else {
    TRACE_ERR("Tracker(%d) is missing!!!", tracking_id_);
    tracked_rect_ = prediction_;
  }

  ageing_++;
  return ret;
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

bool Tracking::getPrediction(timespec stamp, cv::Mat &prediction, cv::Mat &innoCov)
{
  bool ret = false;

  for(auto &rec : kf_vec_)
  {
    if (rec.stamp_ == stamp)
    {
     // prediction = rec.statePre_.clone();
      prediction = rec.measurementPre_.clone();
      innoCov = rec.innoCov_.clone();
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
  return (ageing_ < kAgeingThreshold);
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

void Tracking::initKalmanFilter(filter::KalmanFilter &KF, int nStates, int nMeasurements,
                        int nInputs, cv::Rect2d rect, timespec stamp)
{
  KF.init(nStates, nMeasurements, nInputs, CV_64F);             // init Kalman Filter
  kf_.statePost.at<double>(0) = rect.x + rect.width/2;
  kf_.statePost.at<double>(1) = rect.y + rect.height/2;
  kf_.stamp = stamp;
}

void Tracking::kf_configInterval(filter::KalmanFilter &KF, timespec stamp, bool det)
{

  double dt = (stamp.tv_sec - KF.stamp.tv_sec)*1e3 + (stamp.tv_nsec - KF.stamp.tv_nsec)*1e-6;
  TRACE_INFO("kalman interval(%f)", dt);
  setIdentity(KF.processNoiseCov, cv::Scalar::all(1));// set process noise
  /** DYNAMIC MODEL **/
  //  [1 0 dt 0  ]
  //  [0 1 0  dt ]
  //  [0 0 1  0  ]
  //  [0 0 0  1  ]

  // speed
  KF.transitionMatrix.at<double>(0,2) = dt;
  KF.transitionMatrix.at<double>(1,3) = dt;

  /** MEASUREMENT MODEL **/
  //  [1 0 0 0]
  //  [0 1 0 0]
  
  KF.measurementMatrix.at<double>(0,0) = 1;  // x
  KF.measurementMatrix.at<double>(1,1) = 1;  // y
  /** Process Noise Covariance  **/
  //  [dt5/20 0      0       0       dt4/8  0       0      0     dt3/6    0     0      0    ]
  //  [0      dt5/20 0       0       0      dt4/8   0      0     0       dt3/6  0      0    ]
  //  [0      0      dt5/20  0       0      0      dt4/8   0     0        0     dt3/6  0    ]
  //  [0      0      0       dt5/20  0      0       0      dt4/8 0        0     0      dt3/6]
  //  [dt4/8  0      0       0       dt3/3  0       0      0     dt2/2    0     0      0    ]
  //  [0      dt4/8  0       0       0      dt3/3   0      0     0        dt2/2 0      0    ]
  //  [0      0      dt4/8   0       0      0       dt3/3  0     0        0     dt2/2  0    ]
  //  [0      0      0       dt4/8   0      0       0      dt3/3 0        0     0      dt2/2]
  //  [dt3/6  0      0       0       dt2/2  0       0      0     dt       0     0      0    ]
  //  [0      dt3/6  0       0       0      dt2/2   0      0     0        dt    0      0    ]
  //  [0      0      dt3/6   0       0      0       dt2/2  0     0        0     dt     0    ]
  //  [0      0      0       dt3/6   0      0       0      dt2/2 0        0     0      dt   ]

  double n1 = pow(dt, 4.) / 4.;
  double n2 = pow(dt, 3.) / 2.;
  double n3 = pow(dt, 2.);
  KF.processNoiseCov = (cv::Mat_<double>(4, 4) <<
                                       n1, 0,  n2, 0,
                                       0,  n1, 0,  n2,
                                       n2,  0,  n3, 0, 
                                       0,  n2,  0,  n3);

  KF.processNoiseCov *= 1;

  //setIdentity(KF.measurementNoiseCov, cv::Scalar::all(pow(dt,5)));// set measurement noise
  if (det)
    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(0.001f*n1));// set measurement noise from detect
  else
    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(0.1f*n1));// set measurement noise from tracker

}

#if 0
//Config for rect regression.
void Tracking::kf_configInterval(filter::KalmanFilter &KF, timespec stamp)
{

  double dt = (stamp.tv_sec - KF.stamp.tv_sec)*1e3 + (stamp.tv_nsec - KF.stamp.tv_nsec)*1e-6;
  setIdentity(KF.processNoiseCov, cv::Scalar::all(1));// set process noise
  //setIdentity(KF.measurementNoiseCov, cv::Scalar::all(pow(dt,5)));// set measurement noise
  setIdentity(KF.measurementNoiseCov, cv::Scalar::all(0.001f));// set measurement noise

  /** DYNAMIC MODEL **/
  //  [1 0 0  0  dt  0  0   0 ]
  //  [0 1 0  0  0  dt  0   0 ]
  //  [0 0 1  0  0  0  dt   0 ]
  //  [0 0 0  1  0  0   0  dt ]
  //  [0 0 0  0  1  0   0   0 ]
  //  [0 0 0  0  0  1   0   0 ]
  //  [0 0 0  0  0  0   1   0 ]
  //  [0 0 0  0  0  0   0   1 ]

  // speed
  KF.transitionMatrix.at<double>(0,4) = dt;
  KF.transitionMatrix.at<double>(1,5) = dt;
  KF.transitionMatrix.at<double>(2,6) = dt;
  KF.transitionMatrix.at<double>(3,7) = dt;

  /** MEASUREMENT MODEL **/
  //  [1 0 0 0 0 0 0 0]
  //  [0 1 0 0 0 0 0 0]
  //  [0 0 1 0 0 0 0 0]
  //  [0 0 0 1 0 0 0 0]
  
  KF.measurementMatrix.at<double>(0,0) = 1;  // x
  KF.measurementMatrix.at<double>(1,1) = 1;  // y
  KF.measurementMatrix.at<double>(2,2) = 1;  // w
  KF.measurementMatrix.at<double>(3,3) = 1;  // h
          /** Process Noise Covariance  **/
  //  [dt5/20 0      0       0       dt4/8  0       0      0     dt3/6    0     0      0    ]
  //  [0      dt5/20 0       0       0      dt4/8   0      0     0       dt3/6  0      0    ]
  //  [0      0      dt5/20  0       0      0      dt4/8   0     0        0     dt3/6  0    ]
  //  [0      0      0       dt5/20  0      0       0      dt4/8 0        0     0      dt3/6]
  //  [dt4/8  0      0       0       dt3/3  0       0      0     dt2/2    0     0      0    ]
  //  [0      dt4/8  0       0       0      dt3/3   0      0     0        dt2/2 0      0    ]
  //  [0      0      dt4/8   0       0      0       dt3/3  0     0        0     dt2/2  0    ]
  //  [0      0      0       dt4/8   0      0       0      dt3/3 0        0     0      dt2/2]
  //  [dt3/6  0      0       0       dt2/2  0       0      0     dt       0     0      0    ]
  //  [0      dt3/6  0       0       0      dt2/2   0      0     0        dt    0      0    ]
  //  [0      0      dt3/6   0       0      0       dt2/2  0     0        0     dt     0    ]
  //  [0      0      0       dt3/6   0      0       0      dt2/2 0        0     0      dt   ]
  //
  double n1 = pow(dt, 4.) / 4.;
  double n2 = pow(dt, 3.) / 2.;
  double n3 = pow(dt, 2.);
  KF.processNoiseCov = (cv::Mat_<double>(8, 8) <<
                                       n1, 0,  0,  0,  n2, 0,  0,  0,
                                       0,  n1, 0,  0,  0,  n2, 0,  0,
                                       0,  0,  n1, 0,  0,  0,  n2, 0,
                                       0,  0,  0,  n1, 0,  0,  0,  n2,
                                       n2, 0,  0,  0,  n3, 0,  0,  0,
                                       0,  n2, 0,  0,  0,  n3, 0,  0,
                                       0,  0,  n2, 0,  0,  0,  n3, 0,
                                       0,  0,  0,  n2, 0,  0,  0,  n3);

  KF.processNoiseCov *= 1;
}
#endif

void Tracking::kf_predict(filter::KalmanFilter &KF, cv::Rect2d &tr_predict)
{
  // First predict, to update the internal statePre variable
  cv::Mat prediction = KF.predict();
  tr_predict.x = prediction.at<double>(0);
  tr_predict.y = prediction.at<double>(1);

  //To be removed. Test purpose, emit dx, dy as width, height
  tr_predict.width = prediction.at<double>(2);
  tr_predict.height = prediction.at<double>(3);
}

void Tracking::kf_update(filter::KalmanFilter &KF, cv::Rect2d measurement, timespec &stamp)
{
  // The "correct" phase that is going to use the predicted value and our measurement
  cv::Mat measure(2, 1, CV_64F);
  measure.at<double>(0) = measurement.x + measurement.width/2;
  measure.at<double>(1) = measurement.y + measurement.height/2;

  cv::Mat estimated = KF.correct(measure, stamp);

  KF.stamp = stamp;
  StateRec rec(kf_);
  kf_vec_.push_back(rec);
  if (kf_vec_.size() > 10)
    kf_vec_.erase(kf_vec_.begin());
}

}  // namespace tracker
