// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "object_msgs/msg/objects_in_boxes.hpp"
#include <object_analytics_msgs/msg/objects_in_boxes3_d.hpp>
#include <object_analytics_msgs/msg/tracked_object.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>
#include <object_analytics_msgs/msg/tracked_objects.hpp>
#include <object_analytics_msgs/msg/objects_in_boxes3_d.hpp>

#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <cv_bridge/cv_bridge.h>


using namespace std::chrono_literals;
using std::placeholders::_1;

using ImageMsg = sensor_msgs::msg::Image;
using DetectionMsg = object_msgs::msg::ObjectsInBoxes;
using TrackingMsg = object_analytics_msgs::msg::TrackedObjects;
using LocalizationMsg = object_analytics_msgs::msg::ObjectsInBoxes3D;

using DetectionObject = object_msgs::msg::Object;
using DetectionObjectInBox = object_msgs::msg::ObjectInBox;
using TrackingObjectInBox = object_analytics_msgs::msg::TrackedObject;
using LocalizationObjectInBox = object_analytics_msgs::msg::ObjectInBox3D;

/* This demo code is desiged for showing object analytics result on rviz.
 * Subscribe image/detection/tracking msg, publish tracking 2d box for display. */

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher()
  : Node("image_publisher")
  {

    f_image_sub_= std::make_unique<FilteredLocalization>(this, kTopicImage_);
    f_detection_sub_ = std::make_unique<FilteredDetection>(this, kTopicDetection_);
    f_tracking_sub_ = std::make_unique<FilteredTracking>(this, kTopicTracking_);

    sync_sub_ =
        std::make_unique<FilteredSync>(*f_image_sub_, *f_detection_sub_, *f_tracking_sub_, 10);
    sync_sub_->registerCallback(&ImagePublisher::onObjectsReceived, this);

    image_pub_ = create_publisher<ImageMsg>("/object_analytics/image_publisher");

    RCLCPP_INFO(get_logger(), "Start ImagePublisher ...");

    // subscribe those topics for performance test
    det_subscription_ = this->create_subscription<DetectionMsg>(
      kTopicDetection_, std::bind(&ImagePublisher::det_callback, this, _1));
    tra_subscription_ = this->create_subscription<TrackingMsg>(
      kTopicTracking_, std::bind(&ImagePublisher::tra_callback, this, _1));
    loc_subscription_ = this->create_subscription<LocalizationMsg>(
      kTopicLocalization_, std::bind(&ImagePublisher::loc_callback, this, _1));
  }

private:
  using ObjectRoi = sensor_msgs::msg::RegionOfInterest;
  using FilteredDetection = message_filters::Subscriber<DetectionMsg>;
  using FilteredTracking = message_filters::Subscriber<TrackingMsg>;
  using FilteredLocalization = message_filters::Subscriber<ImageMsg>;
  using FilteredSync =
      message_filters::TimeSynchronizer<ImageMsg, DetectionMsg, TrackingMsg>;

  std::unique_ptr<FilteredDetection> f_detection_sub_;
  std::unique_ptr<FilteredTracking> f_tracking_sub_;
  std::unique_ptr<FilteredLocalization> f_image_sub_;
  std::unique_ptr<FilteredSync> sync_sub_;

  const std::string kTopicDetection_ = "/movidius_ncs_stream/detected_objects";
  const std::string kTopicImage_ = "/object_analytics/rgb";
  const std::string kTopicTracking_ = "/object_analytics/tracking";
  const std::string kTopicLocalization_ = "/object_analytics/localization";

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<DetectionMsg>::SharedPtr det_subscription_;
  rclcpp::Subscription<TrackingMsg>::SharedPtr tra_subscription_;
  rclcpp::Subscription<LocalizationMsg>::SharedPtr loc_subscription_;
  rclcpp::Publisher<ImageMsg>::SharedPtr image_pub_;

  float det_latency_;
  float det_fps_;
  float tra_latency_;
  float tra_fps_;
  float loc_latency_;
  float loc_fps_;

  /* after messages filter, receive msgs from img/det/tra three topics */
  void onObjectsReceived(const ImageMsg::SharedPtr& img,
                         const DetectionMsg::SharedPtr& det,
                         const TrackingMsg::SharedPtr& tra)
  {

    RCLCPP_DEBUG(this->get_logger(), "[Object Vectors size: I=(%d,%d), T=%d, L=%d]",
                img->width, img->height, det->objects_vector.size(),
                tra->tracked_objects.size());

    if (img->header.stamp != tra->header.stamp || tra->header.stamp != det->header.stamp ||
        img->header.stamp != det->header.stamp)
    {
      RCLCPP_WARN(get_logger(), "...Doesn't meet the stamp check, do nothing for the current \
      messages");
      RCLCPP_WARN(get_logger(), "......D==%ld.%ld, T==%ld.%ld, L==%ld.%ld", det->header.stamp.sec,
                  det->header.stamp.nanosec, tra->header.stamp.sec, tra->header.stamp.nanosec,
                  img->header.stamp.sec, img->header.stamp.nanosec);
      return;
    }
    if (img->header.frame_id != tra->header.frame_id ||
        tra->header.frame_id != det->header.frame_id ||
        img->header.frame_id != det->header.frame_id)
    {
      RCLCPP_WARN(get_logger(), "...Doesn't meet the frame_id check, do nothing for the current \
      messages");
      return;
    }


    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
    if (cv_ptr->image.size != 0 && tra->tracked_objects.size() != 0 &&
        det->objects_vector.size() != 0)
    {
      cv_ptr->header = det->header;
      std::vector<DetectionObjectInBox> objects_detected;
      std::vector<TrackingObjectInBox> objects_tracked;
      objects_detected = det->objects_vector;
      objects_tracked = tra->tracked_objects;

      findObject(cv_ptr, objects_detected, objects_tracked);
    }
  }

  /* Find object with same ROI */
  void findObject(cv_bridge::CvImagePtr cv_ptr,
                  std::vector<DetectionObjectInBox> det_objects,
                  std::vector<TrackingObjectInBox> tra_objects)
  {
    // make sure all the msgs are none-empty
    for(auto det : det_objects)
    {
      ObjectRoi roi = det.roi;
      for(auto tra : tra_objects)
      {
        if(roi.x_offset == tra.roi.x_offset && roi.y_offset == tra.roi.y_offset &&
            roi.width == tra.roi.width && roi.height == tra.roi.height)
        {

          std::string obj_name = det.object.object_name;
          drawObject(cv_ptr, roi, obj_name, tra.id);
          // print performance date
          //RCLCPP_INFO(this->get_logger(),
          //            "Performance: [L] fps %.3f hz, latency %.3f sec [D] fps %.3f hz, latency %.3f sec [T] fps %.3f hz, latency %.3f sec",
          //            loc_fps_, loc_latency_, det_fps_, det_latency_, tra_fps_, tra_latency_);
        }
      }
    }
  }

  /* publish object_name, object_id, mix points, max points, 3d box bounaries*/
  void drawObject(cv_bridge::CvImagePtr& cv_ptr, ObjectRoi roi,
                     std::string obj_name, int32_t obj_id)
  {
        RCLCPP_DEBUG(this->get_logger(), "Draw: name=%s, id=%d, roi(%d,%d,%d,%d), img(%d,%d)",
        obj_name.c_str(), obj_id, roi.x_offset, roi.y_offset, roi.height, roi.width, cv_ptr->image.cols, cv_ptr->image.rows);

        //Draw rectangle same size and position as roi.
        cv::Point bottom_left = cv::Point(roi.x_offset, roi.y_offset);
        cv::Point middle_left = cv::Point(roi.x_offset, (roi.y_offset + roi.height/2));
        cv::Point top_right = cv::Point(roi.x_offset + roi.width, roi.y_offset + roi.height);
        cv::rectangle(cv_ptr->image, bottom_left, top_right, cv::Scalar(255, 255, 0), 2, 8, 0);

        //Draw roi text on the top left position.
        std::stringstream ss_roi;
        ss_roi << "ROI[" << roi.x_offset << "," << roi.y_offset << "," << roi.width << "," << roi.height << "]";
        cv::putText(cv_ptr->image, ss_roi.str(), bottom_left, cv::FONT_HERSHEY_SIMPLEX, 1,
                    cv::Scalar(0, 255, 0), 1);

        //Draw object name and tracking id together in the middle left of the rectangle
        std::stringstream ss_name_id;
        ss_name_id << obj_name << "(#" << obj_id << ")";
        cv::putText(cv_ptr->image, ss_name_id.str(), middle_left, cv::FONT_HERSHEY_SIMPLEX, 1,
                    cv::Scalar(0, 255, 0), 1);

        // Draw measure result on the left up
        char ss_det[100];
        snprintf(ss_det, sizeof(ss_det), "Detection:fps=%.2fHz,latency=%.2fSec", det_fps_, det_latency_);
        cv::putText(cv_ptr->image, ss_det, cvPoint(2, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(255, 0, 0), 1);
        char ss_tra[100];
        snprintf(ss_tra, sizeof(ss_tra), "Tracking:fps=%.2fHz,latency=%.2fSec", tra_fps_, tra_latency_);
        cv::putText(cv_ptr->image, ss_tra, cvPoint(2, 15*2), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(255, 0, 0), 1);
        char ss_loc[100];
        snprintf(ss_loc, sizeof(ss_loc), "Localization:fps=%.2fHz,latency=%.2fSec", loc_fps_, loc_latency_);
        cv::putText(cv_ptr->image, ss_loc, cvPoint(2, 15*3), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(255, 0, 0), 1);

        image_pub_->publish(cv_ptr->toImageMsg());
  }

  /* detection callback for performance test */
  void det_callback(const DetectionMsg::SharedPtr msg)
  {
    struct timespec time_start={0, 0};
    clock_gettime(CLOCK_REALTIME, &time_start);
    static double last_sec = 0;
    static double last_nsec = 0;
    static int count = 0;
    double interval = 0;
    double current_sec = time_start.tv_sec;
    double current_nsec = time_start.tv_nsec;
    double msg_sec = msg->header.stamp.sec;
    double msg_nsec = msg->header.stamp.nanosec;

    count++;
    interval = (current_sec - last_sec) + ((current_nsec - last_nsec)/1000000000);
    if(last_sec == 0)
    {
       last_sec = current_sec;
       last_nsec = current_nsec;
       return;
    }

    if(interval >= 1.0)
    {
        double latency = (current_sec - msg_sec) + ((current_nsec - msg_nsec)/1000000000);
        double fps = count/interval;
        count = 0;
        last_sec = current_sec;
        last_nsec = current_nsec;
        RCLCPP_DEBUG(this->get_logger(), "D: fps %.3f hz, latency %.3f sec", fps, latency)
        det_fps_ = fps;
        det_latency_ = latency;
    }
  }

  /* localization callback for performance test */
  void loc_callback(const LocalizationMsg::SharedPtr msg)
  {
    struct timespec time_start={0, 0};
    clock_gettime(CLOCK_REALTIME, &time_start);
    static double last_sec = 0;
    static double last_nsec = 0;
    static int count = 0;
    double interval = 0;
    double current_sec = time_start.tv_sec;
    double current_nsec = time_start.tv_nsec;
    double msg_sec = msg->header.stamp.sec;
    double msg_nsec = msg->header.stamp.nanosec;

    count++;
    interval = (current_sec - last_sec) + ((current_nsec - last_nsec)/1000000000);
    if(last_sec == 0)
    {
       last_sec = current_sec;
       last_nsec = current_nsec;
       return;
    }

    if(interval >= 1.0)
    {
        double latency = (current_sec - msg_sec) + ((current_nsec - msg_nsec)/1000000000);
        double fps = count/interval;
        count = 0;
        last_sec = current_sec;
        last_nsec = current_nsec;
        RCLCPP_DEBUG(this->get_logger(), "L: fps %.3f hz, latency %.3f sec", fps, latency)
        loc_fps_ = fps;
        loc_latency_ = latency;
    }
  }

  /* tracking callback for performance test */
  void tra_callback(const TrackingMsg::SharedPtr msg)
  {
    struct timespec time_start={0, 0};
    clock_gettime(CLOCK_REALTIME, &time_start);
    static double last_sec = 0;
    static double last_nsec = 0;
    static int count = 0;
    double interval = 0;
    double current_sec = time_start.tv_sec;
    double current_nsec = time_start.tv_nsec;
    double msg_sec = msg->header.stamp.sec;
    double msg_nsec = msg->header.stamp.nanosec;

    count++;
    interval = (current_sec - last_sec) + ((current_nsec - last_nsec)/1000000000);
    if(last_sec == 0)
    {
       last_sec = current_sec;
       last_nsec = current_nsec;
       return;
    }

    if(interval >= 1.0)
    {
        double latency = (current_sec - msg_sec) + ((current_nsec - msg_nsec)/1000000000);
        double fps = count/interval;
        count = 0;
        last_sec = current_sec;
        last_nsec = current_nsec;
        RCLCPP_DEBUG(this->get_logger(), "T: fps %.3f hz, latency %.3f sec", fps, latency)
        tra_fps_ = fps;
        tra_latency_ = latency;
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}
