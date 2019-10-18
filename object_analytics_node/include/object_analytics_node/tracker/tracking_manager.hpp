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

#include <memory>
#include <string>
#include <vector>
#include "tracker/tracking.hpp"

namespace tracker
{

class TrackingManager
{
public:
  /**
   * @brief Constructor, a TrackingManager shall be created for one stream.
   */
  explicit TrackingManager();

  /**
   * @brief Manage trackings when objects detected from a new frame.
   * @param[in] mat A new frame with time stamp.
   * @param[in] objs Objects detected from this frame.
   */

  void detectRecvProcess(
    std::shared_ptr<sFrame> frame,
    std::vector<Object>& objs);

  /**
   * @brief Manage trackings when a new frame arrives.
   *
   * When a new frame arrives, for all existing trackings, TrackingManager will
   * update their trackers, each calculating a new roi.
   *
   * @param[in] mat A new frame.
   * @param[in] stamp Time stamp for this track.
   */
  void track(std::shared_ptr<sFrame> frame);

  /**
   * @brief Get Tracked objects list.
   *
   * @param[out] objs List of tracked objects.
   */
  std::vector<std::shared_ptr<Tracking>> getTrackedObjs();

  /**
   * @brief Get algorithm name used by trackers.
   */
  std::string getAlgo();

  /**
   * @brief Set algorithm name used to create trackers.
   */
  void setAlgo(std::string algo) {algo_ = algo;}

  /**
   * @brief Check if detection frame valid.
   */
  bool isDetFrameValid(timespec stamp);

  /**
   * @brief Check if track frame valid.
   */
  bool isTrackFrameValid(timespec stamp);

  /**
   * @brief Check if track frame valid.
   */
  void storeTrackFrameStamp(timespec stamp);

private:
  // The minimum matching level of roi
  static const float kMatchThreshold;
  // The minimum confidence level of detected object
  static const float kProbabilityThreshold;
  // Count of trackings, as a unique ID of a same object
  static int32_t tracking_cnt;
  // Number of threads used for paralleling computation
  static const int32_t kNumOfThread;
  // History timestamps count 
  static const int32_t qFrameNumLimit;
  // List of trackings, each for one detected object
  std::vector<std::shared_ptr<Tracking>> trackings_;
  // Algorithm name to create tracker
  std::string algo_;
  // Flag to record the initialize state
  bool initialized_; 
  // History timestamps in order
  std::deque<timespec> validFrames_;

  /**
   * @brief Add a new tracking to the list.
   *
   * The current value of @ref tracking_cnt will be assigned to this new
   * tracking, which denotes a unique ID of the same object across frames. Then
   * the tracking_cnt automatically increases by one.
   *
   * @param[in] obj_name Name of the object (e.g. people, dog, etc.).
   * @param[in] probability the object.
   * @param[in] rect Roi of the tracked object.
   * @return Pointer to the tracking added.
   */
  std::shared_ptr<Tracking> addTracking(
    const std::string & obj_name,
    const float & probability,
    const cv::Rect2d & rect);

  /**
   * @brief Clean up inactive tracking in the list.
   *
   * See @ref Tracking::isActive() for inactive trackings.
   */
  void cleanTrackings();

  /**
   * @brief Validate the ROI against the size of an image array.
   *
   * In Debug build, ROS_ASSERT failure will be raised in case unexpected ROI
   * detected. In Release build, false shall be returned in case unexpected ROI
   * detected.
   *
   * @param[in] mat Input image arrary
   * @param[in] droi ROI
   * @return true if ROI valid, otherwise false.
   */
  bool validateROI(
    const cv::Mat & mat,
    const cv::Rect2d& droi);


  cv::Mat calcTrackDetWeights(
    std::vector<Object>& dets,
    std::vector<std::shared_ptr<Tracking>>& tracks,
    struct timespec stamp);

  cv::Mat calcTrackDetMahaDistance(
    std::vector<Object>& dets,
    std::vector<std::shared_ptr<Tracking>>& tracks,
    struct timespec stamp);


  /*Kuhn-Munkres algorithm on weight(probabilities)*/
  void matchTrackDetWithProb(cv::Mat& weights, cv::Mat& matches);

  /*Kuhn-Munkres algorithm on cost(distance)*/
  void matchTrackDetWithDistance(cv::Mat& distance, cv::Mat& row_match, cv::Mat& col_match);

  /*recursive find path for each track on weight(probabilities)*/
  bool searchMatch(int srcId,
                   cv::Mat& srcVisit,
                   cv::Mat& srcCorr,
                   cv::Mat& tgtVisit,
                   cv::Mat& tgtCorr,
                   cv::Mat& tgtmatch,
                   cv::Mat& weightDelta,
                   cv::Mat& correlations);

};

}  // namespace tracker
