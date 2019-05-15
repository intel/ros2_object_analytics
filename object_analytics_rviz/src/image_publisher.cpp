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

#include <cv_bridge/cv_bridge.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <object_analytics_msgs/msg/tracked_object.hpp>
#include <object_analytics_msgs/msg/tracked_objects.hpp>

#include <chrono>
#include <string>
#include <vector>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

using ImageMsg = sensor_msgs::msg::Image;
using TrackingMsg = object_analytics_msgs::msg::TrackedObjects;

using DetectionObject = object_msgs::msg::Object;
using DetectionObjectInBox = object_msgs::msg::ObjectInBox;
using TrackingObjectInBox = object_analytics_msgs::msg::TrackedObject;

/* This demo code is desiged for showing object analytics result on rviz.
 * Subscribe image/tracking msg, publish tracking 2d box for display. */

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher()
  : Node("image_publisher")
  {
    rclcpp::Node::SharedPtr node = std::shared_ptr<rclcpp::Node>(this);
    f_image_sub_ = std::make_unique<FilteredImage>(node, kTopicImage_);
    f_tracking_sub_ = std::make_unique<FilteredTracking>(node, kTopicTracking_);

    sync_sub_ =
      std::make_unique<FilteredSync>(*f_image_sub_, *f_tracking_sub_, 10);
    sync_sub_->registerCallback(&ImagePublisher::onObjectsReceived, this);

    image_pub_ = create_publisher<ImageMsg>("/object_analytics/image_publisher");


    RCLCPP_INFO(get_logger(), "Start ImagePublisher ...");

    // subscribe those topics for performance test
    tra_subscription_ = this->create_subscription<TrackingMsg>(
      kTopicTracking_, std::bind(&ImagePublisher::tra_callback, this, _1));
  }

private:
  using ObjectRoi = sensor_msgs::msg::RegionOfInterest;
  using FilteredTracking = message_filters::Subscriber<TrackingMsg>;
  using FilteredImage = message_filters::Subscriber<ImageMsg>;
  using FilteredSync =
    message_filters::TimeSynchronizer<ImageMsg, TrackingMsg>;

  std::unique_ptr<FilteredTracking> f_tracking_sub_;
  std::unique_ptr<FilteredImage> f_image_sub_;
  std::unique_ptr<FilteredSync> sync_sub_;

  const std::string kTopicImage_ = "/object_analytics/rgb";
  const std::string kTopicTracking_ = "/object_analytics/tracking";

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<TrackingMsg>::SharedPtr tra_subscription_;
  rclcpp::Publisher<ImageMsg>::SharedPtr image_pub_;

  float tra_latency_;
  float tra_fps_;

  /* after messages filter, receive msgs from img/tra three topics */
  void onObjectsReceived(
    const ImageMsg::SharedPtr & img, const TrackingMsg::SharedPtr & tra)
  {
    if (img->header.stamp != tra->header.stamp) {
      RCLCPP_WARN(get_logger(), "timestamp not match, do nothing");
      RCLCPP_WARN(get_logger(), "...... T==%ld.%ld, L==%ld.%ld",
        tra->header.stamp.sec, tra->header.stamp.nanosec,
        img->header.stamp.sec, img->header.stamp.nanosec);
      return;
    }
    if (img->header.frame_id != tra->header.frame_id) {
      RCLCPP_WARN(get_logger(), "frame_id not match, do nothing");
      return;
    }

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
    if (cv_ptr->image.size != 0 && tra->tracked_objects.size() != 0) {
      cv_ptr->header = tra->header;
      std::vector<TrackingObjectInBox> objects_tracked;
      objects_tracked = tra->tracked_objects;

      findObject(cv_ptr, objects_tracked);
      image_pub_->publish(cv_ptr->toImageMsg());
    }
  }

  /* Find object with ROI */
  void findObject(
    cv_bridge::CvImagePtr cv_ptr, std::vector<TrackingObjectInBox> tra_objects)
  {
    // make sure all the msgs are none-empty
    for (auto tra : tra_objects) {
      ObjectRoi roi = tra.roi;
      if (roi.x_offset != 0 && roi.y_offset != 0 && roi.width != 0 && roi.height != 0) {
        std::string obj_name = tra.object.object_name;
        drawObject(cv_ptr, roi, obj_name, tra.id);
      }
    }
  }

  /* publish object_name, object_id, mix points, max points, 3d box bounaries*/
  void drawObject(
    cv_bridge::CvImagePtr & cv_ptr, ObjectRoi roi, std::string obj_name, int32_t obj_id)
  {
    RCLCPP_DEBUG(this->get_logger(), "Draw: name=%s, id=%d, roi(%d,%d,%d,%d), img(%d,%d)",
      obj_name.c_str(), obj_id, roi.x_offset, roi.y_offset,
      roi.height, roi.width, cv_ptr->image.cols, cv_ptr->image.rows);

    // Draw rectangle same size and position as roi.
    cv::Point bottom_left = cv::Point(roi.x_offset, roi.y_offset);
    cv::Point middle_left = cv::Point(roi.x_offset, (roi.y_offset + roi.height / 2));
    cv::Point top_right = cv::Point(roi.x_offset + roi.width, roi.y_offset + roi.height);
    cv::rectangle(cv_ptr->image, bottom_left, top_right, cv::Scalar(255, 255, 0), 2, 8, 0);

    // Draw roi text on the top left position.
    std::stringstream ss_roi;
    ss_roi << "ROI[" << roi.x_offset << "," << roi.y_offset << "," << roi.width << "," <<
      roi.height << "]";
    cv::putText(cv_ptr->image, ss_roi.str(), bottom_left, cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 0), 2);

    // Draw object name and tracking id together in the middle left of the rectangle
    std::stringstream ss_name_id;
    ss_name_id << obj_name << "(#" << obj_id << ")";
    cv::putText(cv_ptr->image, ss_name_id.str(), middle_left, cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 0), 2);

    // Draw measure result on the left up
    char ss_tra[100];
    snprintf(ss_tra, sizeof(ss_tra), "Tracking:fps=%.2fHz,latency=%.2fSec",
      tra_fps_, tra_latency_);
    cv::putText(cv_ptr->image, ss_tra, cvPoint(2, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
      cv::Scalar(0, 0, 255), 2);
  }

  /* tracking callback for performance test */
  void tra_callback(const TrackingMsg::SharedPtr msg)
  {
    struct timespec time_start = {0, 0};
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
    interval = (current_sec - last_sec) + ((current_nsec - last_nsec) / 1000000000);
    if (last_sec == 0) {
      last_sec = current_sec;
      last_nsec = current_nsec;
      return;
    }

    if (interval >= 1.0) {
      double latency = (current_sec - msg_sec) + ((current_nsec - msg_nsec) / 1000000000);
      double fps = count / interval;
      count = 0;
      last_sec = current_sec;
      last_nsec = current_nsec;
      RCLCPP_DEBUG(this->get_logger(), "T: fps %.3f hz, latency %.3f sec", fps, latency);
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
