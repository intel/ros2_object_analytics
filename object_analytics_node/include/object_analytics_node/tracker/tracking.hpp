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
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

namespace object_analytics_node
{
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
    const cv::Mat & mat, const cv::Rect2d & tracked_rect,
    const cv::Rect2d & detected_rect,
    builtin_interfaces::msg::Time stamp);

  /**
   * @brief Update tracker with the tracking frame.
   *
   * @param[in] mat The tracking frame.
   * @return true if tracker was updated successfully, otherwise false.
   */
  bool updateTracker(const cv::Mat & mat, builtin_interfaces::msg::Time stamp);

  /**
   * @brief Get the roi of tracked object.
   *
   * @return Roi of the tracked object.
   */
  cv::Rect2d getTrackedRect();

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
   * @brief Get the roi of the detected object.
   *
   * @return Roi of the detected object.
   */
  cv::Rect2d getDetectedRect();

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
   * @brief Get the detected status of a tracking.
   *
   * @return true if tracking is detected, otherwise false.
   */
  bool isDetected();

  /**
   * @brief Set the detected status of a tracking. Ageing is set to zero also.
   */
  void setDetected();

  /**
   * @brief Clear the detected status of a tracking.
   */
  void clearDetected();

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
   * @brief collect the history coordination with time stamps.
   * @param[in] stamp The tracking frame stamp.
   * @param[in] t_rect Roi of the tracked object.
   */
  void collectHistory(builtin_interfaces::msg::Time stamp, cv::Rect2d t_rect);

  /**
   * @brief find ROI in histories.
   * @param[in] mat The query frame.
   * @param[out] t_rect Roi of the tracked object.
   * @return if found corresponding image and rect.
   */
  bool getHisTrackedRect(
    builtin_interfaces::msg::Time stamp,
    cv::Rect2d & t_rect);

  /**
   * @brief Clear the history of tracking.
   */
  void clearHistory();
  /**
   * @brief check if timestamp in side history.
   * @param[in] stamp Time stamp to check.
   * @return if stamp inside history timezone.
   */
  bool checkTimeZone(builtin_interfaces::msg::Time stamp);

private:
  static const int32_t
    kAgeingThreshold;   /**< The maximum ageing of an active tracking.*/
  cv::Ptr<cv::Tracker> tracker_; /**< Tracker associated to this tracking.*/
  cv::Rect2d tracked_rect_;      /**< Roi of the tracked object.*/
  std::string obj_name_;         /**< Name of the tracked object.*/
  float probability_;            /**< Probability of the tracked object.*/
  cv::Rect2d detected_rect_;     /**< Roi of the detected object. */
  int32_t tracking_id_;          /**< ID of this tracking.*/
  int32_t ageing_;               /**< Age of this tracking.*/
  bool detected_;                /**< Detected status of this tracking.*/
  int32_t detect_mis_;           /**< Count of missed in detection.*/
  std::string algo_;             /**< Algorithm name for the tracking.*/
  std::vector<std::pair<builtin_interfaces::msg::Time, cv::Rect2d>>
  hisCor_;     /*tracked coordinates history.*/
};
}  // namespace tracker
}  // namespace object_analytics_node
#endif  // OBJECT_ANALYTICS_NODE__TRACKER__TRACKING_HPP_
