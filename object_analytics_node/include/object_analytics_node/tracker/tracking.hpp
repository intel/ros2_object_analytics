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

#ifndef OBJECT_ANALYTICS_NODE__TRACKER__TRACKING_HPP_
#define OBJECT_ANALYTICS_NODE__TRACKER__TRACKING_HPP_

#include <opencv2/tracking.hpp>
#include <opencv2/video/tracking.hpp>
#include <string>
#include <utility>
#include <vector>
#include <memory>

#include "common/frame.hpp"
#include "common/utility.hpp"
#include "filter/kalman.hpp"
#include "tracker/trackerKCF.hpp"

namespace tracker
{

class Traj
{
public:
  Traj(timespec stamp, cv::Rect2d rect, cv::Mat covar, cv::Mat frame)
  {
    stamp_ = stamp;
    rect_ = rect;
    covar_ = covar.clone();
    frame_ = frame;
  }

  Traj() {}

public:
  timespec stamp_;
  cv::Rect2d rect_;
  cv::Mat covar_;
  cv::Mat frame_;
};


/**
 * @INIT: just retrieved external detections, need to be confirmed.
 * @ACTIVE: confirmed as target, continuous tracking.
 * @INACTIVE: lost tracking for short frames.
 * @LOST: lost tracking, need to remove from tracking list.
 */
enum STATE
{
  INIT   = (1 << 0),
  ACTIVE = (1 << 1),
  INACTIVE = (1 << 2),
  LOST = (1 << 3)
};

class Tracking
{
public:
  Tracking() {}

  /**
   * @brief Constructor of Tracking.
   *
   * @param[in] tracking_id ID of this tracking.
   * @param[in] name Name of the tracked object.
   * @param[in] probability of the tracked object.
   * @param[in] rect Roi of the tracked object.
   */
  Tracking(
    int32_t tracking_id, const std::string & name,
    const float & probability, const cv::Rect2d & rect);

  /**
   * @brief Default destructor.
   */
  ~Tracking();

  /**
   * @brief Rectify tracker with the roi of the detected object.
   *
   * @param[in] mat The detection frame.
   * @param[in] tracked_rect Roi of the tracked object.
   * @param[in] detected_rect Roi of the detected object.
   */
  void rectifyTracker(
    const std::shared_ptr<sFrame> frame, const cv::Rect2d & d_rect);

  /**
   * @brief Detect tracker with the tracking frame.
   *
   * @param[in] mat The tracking frame.
   * @param[in] stamp Time stamp of the tracking frame.
   * @return true if tracker was updated successfully, otherwise false.
   */
  bool detectTracker(const std::shared_ptr<sFrame> frame);

  /**
   * @brief Update tracker with the tracking frame.
   *
   * @param[in] mat The tracking frame.
   * @param[in] stamp Time stamp of the tracking frame.
   * @return true if tracker was updated successfully, otherwise false.
   */
  void updateTracker(
    const std::shared_ptr<sFrame> frame, Rect2d & boundingBox, float confidence,
    bool det);

  /**
   * @brief Get the roi of tracked object.
   *
   * @return Roi of the tracked object.
   */
  cv::Rect2d getTrackedRect();

  /**
   * @brief Get prediction of tracked object.
   *
   * @return Roi of prediction.
   */
  cv::Rect2d getPredictedRect();

  /**
   * @brief Get the name of the tracked object.
   *
   * @return Name of the tracked object.
   */
  std::string getObjName();

  /**
   * @brief Get the probability of the tracked object.
   *
   * @return probability of the tracked object.
   */
  float getObjProbability();

  /**
   * @brief Get the tracking id.
   *
   * @return ID of the tracking.
   */
  int32_t getTrackingId();

  /**
   * @brief Get the active status of a tracking
   *
   * @return true if tracking is active, otherwise false.
   */
  bool isActive();

  /**
   * @brief Get the active status of a tracking
   *
   * @return true if tracking is available, otherwise false.
   */
  bool isAvailable();

  /**
   * @brief Get algorithm used for tracking.
   * @return tracking algorithm name.
   */
  std::string getAlgo();

  /**
   * @brief Set algorithm used for tracking.
   * @return true if the algorithm is available.
   */
  bool setAlgo(std::string algo);

  /**
   * @brief create Tracker accoring to algorithm name.
   * @return the tracker created.
   */
  cv::Ptr<TrackerKCFImpl> createTrackerByAlgo(std::string name);

  /**
   * @brief find traj according to stamp.
   * @return the traj found.
   */
  bool getTraj(timespec stamp, Traj & traj);

  /**
   * @brief find latest traj.
   * @return the traj found.
   */
  bool getTraj(Traj & traj);

  /**
   * @brief push traj to pool.
   */
  void storeTraj(timespec stamp, cv::Rect rect, cv::Mat & cov, cv::Mat frame);

  /**
   * @brief get all the trajs.
   * @return the traj list.
   */
  std::vector<Traj> getTrajs();

  /**
   * @brief increase detection count, change state if need.
   */
  void incDetCount();
  /**
   * @brief decrease detection count, change state if need.
   */
  void decDetCount();

  /**
   * @brief clear TrackLost count, change state if need.
   */
  void clearTrackLost();
  /**
   * @brief increase TrackLost count, change state if need.
   */
  void incTrackLost();

  /**
   * @brief get current tracker state.
   * @return the current tracker state.
   */
  STATE getState() {return state_;}

public:
  /*Latest track covariance.*/
  cv::Mat covar_;

private:
  /*The maximum ageing of an active tracking.*/
  static const int32_t kAgeingThreshold;
  /*The maximum count of detection.*/
  static const int32_t kDetCountThreshold;
  /*The maximum count of track lost.*/
  static const int32_t kTrackLostThreshold;
  /*The maximum length of trajtories.*/
  static const int32_t kTrajLength;

  /*Tracker associated to this tracking.*/
  cv::Ptr<TrackerKCFImpl> tracker_;
  /*Roi of the tracked object.*/
  cv::Rect2d tracked_rect_;
  /*Prediction of the tracked object.*/
  cv::Rect2d prediction_;

  /*Name of the tracked object.*/
  std::string obj_name_;
  /*Probability of the tracked object.*/
  float probability_;
  /*ID of this tracking.*/
  int32_t tracking_id_;

  /*Algorithm name for the tracking.*/
  std::string algo_;

  /*Kalman filter for prediction*/
  filter::KalmanFilter kalman_;
  /*Vector of kalman status.*/
  std::vector<Traj> trajVec_;

  /*Detection count of this tracking.*/
  int32_t detCount_;
  /*Tracking lost count of this tracking.*/
  int32_t trackLost_;
  /*Age of this tracking.*/
  int32_t ageing_;

  /*State of this tracking.*/
  STATE state_;
};

}  // namespace tracker

#endif  // OBJECT_ANALYTICS_NODE__TRACKER__TRACKING_HPP_
