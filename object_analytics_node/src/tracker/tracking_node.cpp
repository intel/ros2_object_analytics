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
#include <vector>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include "object_analytics_node/tracker/tracking_node.hpp"
#include "object_analytics_node/const.hpp"

namespace object_analytics_node
{
namespace tracker
{
// TrackingNode class implementation
using SubscribeImg = message_filters::Subscriber<sensor_msgs::msg::Image>;
using SubscribeObjs = message_filters::Subscriber<object_msgs::msg::ObjectsInBoxes>;
using Synchronizer = message_filters::TimeSynchronizer<object_msgs::msg::ObjectsInBoxes, sensor_msgs::msg::Image>;

TrackingNode::TrackingNode() : Node("TrackingNode")
{
  auto rgb_callback = [this](const typename sensor_msgs::msg::Image::SharedPtr image) -> void { this->rgb_cb(image); };
  sub_rgb_ = create_subscription<sensor_msgs::msg::Image>(Const::kTopicRgb, rgb_callback);

  auto obj_callback = [this](const typename object_msgs::msg::ObjectsInBoxes::SharedPtr objs) -> void {
    this->obj_cb(objs);
  };
  sub_obj_ = create_subscription<object_msgs::msg::ObjectsInBoxes>(Const::kTopicDetection, obj_callback);

  pub_tracking_ = create_publisher<object_analytics_msgs::msg::TrackedObjects>(Const::kTopicTracking);
  tm_ = std::make_unique<TrackingManager>(this);
  last_detection_ = builtin_interfaces::msg::Time();
  this_detection_ = builtin_interfaces::msg::Time();
  last_obj_ = nullptr;
  this_obj_ = nullptr;

  kRgbQueueSize = 10;
  rgbs_.reserve(kRgbQueueSize);
  tracks_.reserve(kRgbQueueSize);
}

void TrackingNode::rgb_cb(const sensor_msgs::msg::Image::ConstSharedPtr& img)
{
  RCUTILS_LOG_DEBUG("received rgb frameid(%s)!\n", img->header.frame_id.c_str());

  if (this_detection_ != last_detection_)
  {
    cv::Mat mat_cv = cv_bridge::toCvShare(img, "bgr8")->image;
	if(this_detection_ == img->header.stamp)
	{
      	RCLCPP_DEBUG(get_logger(), "rectify in rgb_cb!");
      	tm_->detect(mat_cv, this_obj_);
	} else {	
		tm_->track(mat_cv);
	}
	tracking_publish(img->header);
  }

  rgbs_.push_back(img);
  if(kRgbQueueSize < rgbs_.capacity())
      rgbs_.erase(rgbs_.begin());

}

bool operator<(const builtin_interfaces::msg::Time& left, const builtin_interfaces::msg::Time& right)
{
  if (left.sec < right.sec)
  {
    return true;
  }
  if (left.sec == right.sec && left.nanosec < right.nanosec)
  {
    return true;
  }
  return false;
}

void TrackingNode::obj_cb(const object_msgs::msg::ObjectsInBoxes::ConstSharedPtr& objs)
{
  last_detection_ = this_detection_;
  this_detection_ = objs->header.stamp;
  last_obj_ = this_obj_;
  this_obj_ = objs;

  RCUTILS_LOG_DEBUG("received obj detection frame_id(%s)!\n", objs->header.frame_id.c_str());
  std::vector<sensor_msgs::msg::Image::ConstSharedPtr>::iterator rgb = rgbs_.begin();
  while (rgb != rgbs_.end())
  {
    if ((*rgb)->header.stamp < this_detection_)
    {
      RCLCPP_DEBUG(get_logger(), "slower, dropped");
      rgb = rgbs_.erase(rgb);
      continue;
    }
    cv::Mat mat_cv = cv_bridge::toCvShare(*rgb, "bgr8")->image;
    if ((*rgb)->header.stamp == this_detection_)
    {
      RCLCPP_DEBUG(get_logger(), "rectify!");
	  if(check_rectify(objs))
	  {
        tm_->detect(mat_cv, this_obj_);
      } else {
  		RCUTILS_LOG_DEBUG("rectify frame_id(%s) cancelled!\n", objs->header.frame_id.c_str());
      }
//    tracking_publish((*rgb)->header);
//    rgb = rgbs_.erase(rgb);
	  break;
    }

	rgb++;
  }

}

void TrackingNode::tracking_publish(const std_msgs::msg::Header& header)
{
  object_analytics_msgs::msg::TrackedObjects::SharedPtr msg =
      std::make_shared<object_analytics_msgs::msg::TrackedObjects>();
  msg->header = header;

  tracks_.push_back(msg);
  if(kRgbQueueSize < tracks_.capacity())
      tracks_.erase(tracks_.begin());

  if(tm_->getTrackedObjs(msg) > 0)
      pub_tracking_->publish(msg);
}


bool TrackingNode::check_rectify(const object_msgs::msg::ObjectsInBoxes::ConstSharedPtr& objs)
{
  builtin_interfaces::msg::Time detect_frame = objs->header.stamp;
  bool res = true;

  std::vector<object_analytics_msgs::msg::TrackedObjects::SharedPtr> ::iterator track = tracks_.begin();
  while (track != tracks_.end())
  {
    if ((*track)->header.stamp < detect_frame)
    {
      track = tracks_.erase(track);
      continue;
    }
    if ((*track)->header.stamp == detect_frame)
    {
	 //TBD: Compare each objects box in track and detection,
	 //1. if new object appear, break to rectify,
	 //2. if any object in track not overlay 70% with detection, break to rectify,
	 //3. else escape from rectify.
	   for (int i = 0; i < objs->objects_vector.size(); i++)
	   {
		 object_msgs::msg::Object dobj = objs->objects_vector[i].object;
#if 0
		 if (dobj.probability < kProbabilityThreshold)
		 {
		   continue;
		 }
#endif
		 std::string n = dobj.object_name;
//	     float probability =  dobj.probability;
		 sensor_msgs::msg::RegionOfInterest droi = objs->objects_vector[i].roi;
		 cv::Rect2d detected_rect = cv::Rect2d(droi.x_offset, droi.y_offset, droi.width, droi.height);
		 auto tobj = (*track)->tracked_objects.begin();

		 for(; tobj != (*track)->tracked_objects.end(); tobj++)
		 {
		   if(n == tobj->object.object_name)
		   {
		     cv::Rect2d tracked_rect = cv::Rect2d(tobj->roi.x_offset, tobj->roi.y_offset, tobj->roi.width, tobj->roi.height);
             double intersectArea = (tracked_rect & detected_rect).area();
             double unionArea = (tracked_rect | detected_rect).area();
			 double precision = unionArea/intersectArea;
			 if(precision > 0.7/*&& probability < 0.8*/)
			 {
				RCUTILS_LOG_DEBUG("Tracked correct, no need to rectify!!!!\n");
			 } else {
				return res;
			 }
			 break;
		   }
		 }

		 if (tobj == (*track)->tracked_objects.end())
			return res; 

	    }
	  break;
    }

	track++;
  }

  if( track != tracks_.end())
  {
	res = false;
  }

  return res;
}


}  // namespace tracker
}  // namespace object_analytics_node

#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(object_analytics_node::tracker::TrackingNode, rclcpp::Node)
