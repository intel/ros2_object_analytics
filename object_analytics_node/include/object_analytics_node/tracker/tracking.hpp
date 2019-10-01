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

#pragma once

#include <opencv2/tracking.hpp>
#include <opencv2/video/tracking.hpp>
#include <string>
#include <utility>
#include <vector>

#include "frame.hpp"
#include "object_analytics_node/filter/kalman.hpp"

namespace tracker
{
/** @class Tracking
 * One single tracking against an object detected across camera frames.
 *
 * A tracking has a few external attributes
 * - tracking id, a unique ID denoting a same object arcoss frames.
 * - object name, name of the tracked object.
 * - rect, roi of the tracked object.
 *
 * Also a tracking has some internal attributs
 * - ageing, will be increased by one upon a tracking frame arrives, and will be
 * reset to zero when a detection frame arrives. @ref kAgeingThreshold specify
 * the maximum age of an active tracking. Trackings below this age are actively
 * updated when tracking frame arrives. Trackings above this age are consider
 * inactive, and to be removed from the list.
 * - detected, will be set when a detection frame arrives. Tracking associated
 * to a detected object will have its detected flag set as true.
 *
 * When a tracking is created, it is assigned a tracking ID, and associated with
 * the name and roi of the detected object. When a detection frame arrives, a
 * tracking shall rectify its tracker, see @ref rectifyTracker, with the
 * detection roi. While when a tracking frame arrives, a tracking shall update
 * its tracker.
 */

class StateRec
{
public:
  explicit StateRec(filter::KalmanFilter &kf)
  {
    statePre_ = kf.statePre.clone();
    statePost_ = kf.statePost.clone();
    errorCovPre_ = kf.errorCovPre.clone();
    errorCovPost_ = kf.errorCovPost.clone();
    innoCov_ = kf.innoCov.clone();
    measurementPre_ = kf.measurementPre.clone();
    gain_= kf.gain;
    stamp_ = kf.stamp;
  };

public:
  timespec stamp_;
  cv::Mat statePre_;
  cv::Mat statePost_;
  cv::Mat errorCovPre_;
  cv::Mat errorCovPost_;
  cv::Mat innoCov_;
  cv::Mat gain_;
  cv::Mat measurementPre_;
};

class Tracking
{
public:
  /**
   * @brief Constructor of Tracking.
   *
   * @param[in] tracking_id ID of this tracking.
   * @param[in] name Name of the tracked object.
   * @param[in] probability of the tracked object.
   * @param[in] rect Roi of the tracked object.
   */
  explicit Tracking(
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
   * @brief Update tracker with the tracking frame.
   *
   * @param[in] mat The tracking frame.
   * @param[in] stamp Time stamp of the tracking frame.
   * @return true if tracker was updated successfully, otherwise false.
   */
  bool updateTracker(const std::shared_ptr<sFrame> frame);

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
   * @brief Get the active status of a tracking, see @ref kAgeingThreshold.
   *
   * @return true if tracking is active, otherwise false.
   */
  bool isActive();

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
  cv::Ptr<cv::Tracker> createTrackerByAlgo(std::string name);

  /**
   * @brief initialize data dimensions kalman filter.
   * @param[in] KF Kalman filter structure.
   * @param[in] nStates State vector dimension.
   * @param[in] nMeasurements Measurements vector dimension.
   * @param[in] nInputs control vector dimension.
   */
  void initKalmanFilter(filter::KalmanFilter &KF, int nStates,
                        int nMeasurements, int nInputs, cv::Rect2d rect, timespec stamp);

  /**
   * @brief prefict status based on previous status.
   * @param[in] KF Kalman filter structure.
   * @param[in] tr_predict state predicted from previous status.
   */
  void kf_predict(filter::KalmanFilter &KF, cv::Rect2d &tr_predict);

  /**
   * @brief configure the time interval of kalman filter.
   * @param[in] KF Kalman filter structure.
   * @param[in] destinate time to predict/update.
   */
  void kf_configInterval(filter::KalmanFilter &KF, timespec stamp, bool det=false);

  /**
   * @brief update status.
   * @param[in] KF Kalman filter structure.
   * @param[in] nMeasurements Measurements vector dimension.
   */
  void kf_update(filter::KalmanFilter &KF, cv::Rect2d measurement, timespec &stamp);

  bool getPrediction(timespec stamp, cv::Mat &prediction, cv::Mat &innoCov);

private:
  static const int32_t kAgeingThreshold;   /**< The maximum ageing of an active tracking.*/
  cv::Ptr<cv::Tracker> tracker_; /**< Tracker associated to this tracking.*/
  cv::Rect2d tracked_rect_;      /**< Roi of the tracked object.*/
  cv::Rect2d prediction_;        /**< Prediction of the tracked object.*/
  std::string obj_name_;         /**< Name of the tracked object.*/
  float probability_;            /**< Probability of the tracked object.*/
  int32_t tracking_id_;          /**< ID of this tracking.*/
  int32_t ageing_;               /**< Age of this tracking.*/
  std::string algo_;             /**< Algorithm name for the tracking.*/
  filter::KalmanFilter kf_;      /*Kalmanfilter to predict and estimate tracking*/
  std::vector<StateRec> kf_vec_; /**< Vector of kalman status.*/
  bool initial_kf_state_;

};

}  // namespace tracker
