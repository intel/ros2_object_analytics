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
#include <cv_bridge/cv_bridge.h>
#include "object_analytics_node/model/object_utils.hpp"
#include "object_analytics_node/tracker/tracking_manager.hpp"

using object_analytics_node::model::ObjectUtils;

namespace object_analytics_node
{
namespace tracker
{
// TrackingManager class implementation
const float TrackingManager::kMatchThreshold = 0.2;
const float TrackingManager::kProbabilityThreshold = 0.5;
int32_t TrackingManager::tracking_cnt = 0;
const int32_t TrackingManager::kNumOfThread = 4;

TrackingManager::TrackingManager(const rclcpp::Node* node) : node_(node)
{
  algo_ = "MIL";
}

void TrackingManager::track(const cv::Mat& mat)
{
  std::vector<std::shared_ptr<Tracking>>::iterator t = trackings_.begin();

  while (t != trackings_.end())
  {
    if (!(*t)->updateTracker(mat))
    {
      RCLCPP_INFO(node_->get_logger(), "Tracking[%d] failed, remove ---", (*t)->getTrackingId());
      t = trackings_.erase(t);
    }
    else
    {
      ++t;
    }
  }

}

void TrackingManager::detect(const cv::Mat& mat, const object_msgs::msg::ObjectsInBoxes::ConstSharedPtr& objs)
{
  uint32_t i;

  for (auto t : trackings_)
  {
    t->clearDetected();
  }
  RCLCPP_DEBUG(node_->get_logger(), "****detected objects: %zu", objs->objects_vector.size());
/* rectify tracking ROIs with detected ROIs*/
#pragma omp parallel for num_threads(kNumOfThread)
  for (i = 0; i < objs->objects_vector.size(); i++)
  {
    object_msgs::msg::Object dobj = objs->objects_vector[i].object;
    if (dobj.probability < kProbabilityThreshold)
    {
      continue;
    }
    std::string n = dobj.object_name;
//  float probability =  dobj.probability;
    sensor_msgs::msg::RegionOfInterest droi = objs->objects_vector[i].roi;
    cv::Rect2d detected_rect = cv::Rect2d(droi.x_offset, droi.y_offset, droi.width, droi.height);
    /* some trackers do not accept an ROI beyond the size of a Mat*/
    if (!validateROI(mat, droi))
    {
      RCLCPP_WARN(node_->get_logger(), "unexptected ROI [%d %d %d %d] against mat size [%d %d]", droi.x_offset,
                  droi.y_offset, droi.width, droi.height, mat.cols, mat.rows);
      droi.x_offset = droi.x_offset >= static_cast<uint32_t>(mat.cols) ? (mat.cols - 1) : droi.x_offset;
      droi.y_offset = droi.y_offset >= static_cast<uint32_t>(mat.rows) ? (mat.rows - 1) : droi.y_offset;
      droi.width =
          droi.x_offset + droi.width > static_cast<uint32_t>(mat.cols) ? (mat.cols - droi.x_offset) : droi.width;
      droi.height =
          droi.y_offset + droi.height > static_cast<uint32_t>(mat.rows) ? (mat.rows - droi.y_offset) : droi.height;
    }
    cv::Rect2d tracked_rect = cv::Rect2d(droi.x_offset, droi.y_offset, droi.width, droi.height);
    RCLCPP_DEBUG(node_->get_logger(), "detected %s [%d %d %d %d] %.0f%%", n.c_str(), droi.x_offset, droi.y_offset,
                 droi.width, droi.height, dobj.probability * 100);
    std::shared_ptr<Tracking> t;
#pragma omp critical
    {
      /* get matched tracking with the detected object name (class) and its ROI*/
      t = getTracking(n, tracked_rect);
      /* add tracking if new object detected*/
      if (!t)
      {
        t = addTracking(n, dobj.probability, tracked_rect);
      }
      t->setDetected();
    }

    /* rectify tracking ROI with detected ROI*/
    t->rectifyTracker(mat, tracked_rect, detected_rect);
  }

  /* clean up inactive trackings*/
  cleanTrackings();
}

int32_t TrackingManager::getTrackedObjs(const object_analytics_msgs::msg::TrackedObjects::SharedPtr& objs)
{
  for (auto t : trackings_)
  {
    if (!t->isDetected())
    {
  	  RCLCPP_INFO(node_->get_logger(), "****Not detected, escaped");
      continue;
    }
    object_analytics_msgs::msg::TrackedObject tobj;
 // cv::Rect2d r = t->getDetectedRect();
    cv::Rect2d r = t->getTrackedRect();
    tobj.id = t->getTrackingId();
    tobj.object.object_name = t->getObjName();
    tobj.object.probability = t->getObjProbability();
    tobj.roi.x_offset = static_cast<int>(r.x);
    tobj.roi.y_offset = static_cast<int>(r.y);
    tobj.roi.width = static_cast<int>(r.width);
    tobj.roi.height = static_cast<int>(r.height);
    objs->tracked_objects.push_back(tobj);
  }

  return objs->tracked_objects.size();
}

std::shared_ptr<Tracking> TrackingManager::addTracking(const std::string& name, const float& probability,
                                                       const cv::Rect2d& rect)
{
  std::shared_ptr<Tracking> t = std::make_shared<Tracking>(tracking_cnt++, name, probability, rect);
  if (tracking_cnt == -1)
  {
    RCLCPP_WARN(node_->get_logger(), "tracking count overflow");
  }
  RCLCPP_DEBUG(node_->get_logger(), "addTracking[%d] +++", t->getTrackingId());
  t->setAlgo(algo_);
  trackings_.push_back(t);
  return t;
}

void TrackingManager::cleanTrackings()
{
  std::vector<std::shared_ptr<Tracking>>::iterator t = trackings_.begin();
  while (t != trackings_.end())
  {
    if (!(*t)->isActive())
    {
      RCLCPP_DEBUG(node_->get_logger(), "removeTracking[%d] ---", (*t)->getTrackingId());
      t = trackings_.erase(t);
    }
    else
    {
      ++t;
    }
  }
}

/* get matched tracking for each detected object,
 * with the same object name,
 * and the most matching ROI
 */
std::shared_ptr<Tracking> TrackingManager::getTracking(const std::string& obj_name, const cv::Rect2d& rect)
{
  double match = 0;
  std::shared_ptr<Tracking> tracking = std::shared_ptr<Tracking>();

  /* searching over all trackings*/
  for (auto t : trackings_)
  {
    /* seek for the one with the same object name (class), and not yet rectified*/
    if (!t->isDetected() && 0 == obj_name.compare(t->getObjName()))
    {
      cv::Rect2d trect = t->getTrackedRect();
      double m = ObjectUtils::getMatch(trect, rect);
      RCLCPP_DEBUG(node_->get_logger(), "tr[%d] %s [%d %d %d %d]%.2f", t->getTrackingId(), t->getObjName().c_str(),
                   (int)trect.x, (int)trect.y, (int)trect.width, (int)trect.height, m);
      /* seek for the one with the most matching ROI*/
      if (m > match)
      {
        tracking = t;
        match = m;
      }
    }
  }
  /* if matching above the threshold, return the tracking*/
  if (match >= TrackingManager::kMatchThreshold)
  {
    return tracking;
  }
  else
  {
    return std::shared_ptr<Tracking>();
  }
}

bool TrackingManager::validateROI(const cv::Mat& mat, const sensor_msgs::msg::RegionOfInterest& droi)
{
  return (droi.x_offset < static_cast<uint32_t>(mat.cols) && droi.y_offset < static_cast<uint32_t>(mat.rows) &&
          (droi.x_offset + droi.width) <= static_cast<uint32_t>(mat.cols) &&
          (droi.y_offset + droi.height) <= static_cast<uint32_t>(mat.rows));
}

}  // namespace tracker
}  // namespace object_analytics_node
