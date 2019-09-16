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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <object_analytics_msgs/msg/tracked_objects.hpp>
#include <object_analytics_msgs/msg/objects_in_boxes3_d.hpp>
#include <object_analytics_msgs/msg/tracked_object.hpp>

#include <chrono>
#include <string>
#include <vector>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

using TrackingMsg = object_analytics_msgs::msg::TrackedObjects;
using LocalizationMsg = object_analytics_msgs::msg::ObjectsInBoxes3D;
using TrackingObjectInBox = object_analytics_msgs::msg::TrackedObject;
using LocalizationObjectInBox = object_analytics_msgs::msg::ObjectInBox3D;

const int kMsgQueueSize = 10;
/* This demo code is desiged for showing object analytics result on rviz.
 * Subscribe localization/tracking msg, publish box_3d_markers for display. */

class MarkerPublisher : public rclcpp::Node
{
public:
  MarkerPublisher()
  : Node("marker_publisher")
  {
    loc_performance_subscription_ = this->create_subscription<LocalizationMsg>(
      "/object_analytics/localization",
      std::bind(&MarkerPublisher::loc_performance_callback, this, _1));
    loc_marker_subscription_ = this->create_subscription<LocalizationMsg>(
      "/object_analytics/localization", std::bind(&MarkerPublisher::loc_marker_callback, this, _1));
    marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/object_analytics/marker_publisher");


    RCLCPP_INFO(get_logger(), "Start MarkerPublisher ...");
  }

private:
  using ObjectRoi = sensor_msgs::msg::RegionOfInterest;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<LocalizationMsg>::SharedPtr loc_marker_subscription_;
  rclcpp::Subscription<LocalizationMsg>::SharedPtr loc_performance_subscription_;
  rclcpp::Subscription<TrackingMsg>::SharedPtr tra_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  float loc_latency_;
  float loc_fps_;
  float tra_latency_;
  float tra_fps_;

  /* create 3d boxes of objects */
  void loc_marker_callback(
    const LocalizationMsg::SharedPtr loc)
  {
    if (loc->objects_in_boxes.size() != 0) {
      std::vector<LocalizationObjectInBox> objects_localized;
      std_msgs::msg::Header header = loc->header;
      objects_localized = loc->objects_in_boxes;
      MarkerPublisher::createMarker(header, objects_localized);
    }
  }

  void createMarker(
    std_msgs::msg::Header header,
    std::vector<LocalizationObjectInBox> loc_objects)
  {
    visualization_msgs::msg::MarkerArray marker_array_loc;
    visualization_msgs::msg::Marker marker_clear;
    marker_array_loc.markers = std::vector<visualization_msgs::msg::Marker>();
    marker_clear.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_clear.header = header;
    marker_array_loc.markers.emplace_back(marker_clear);
    int marker_id = 0;
    for (auto loc : loc_objects) {
      if (loc.min.x == 0 && loc.min.y == 0 && loc.min.z == 0 &&
        loc.max.x == 0 && loc.max.y == 0 && loc.max.z == 0)
      {
        break;
      }
      geometry_msgs::msg::Point box_min;
      box_min.x = loc.min.x;
      box_min.y = loc.min.y;
      box_min.z = loc.min.z;
      geometry_msgs::msg::Point box_max;
      box_max.x = loc.max.x;
      box_max.y = loc.max.y;
      box_max.z = loc.max.z;
      std::string obj_name = loc.object.object_name;
      MarkerPublisher::addMarker(marker_array_loc, header, box_min, box_max,
        obj_name, marker_id);
    }
    MarkerPublisher::addPerformanceMarker(marker_array_loc, header, loc_fps_, loc_latency_,
      ++marker_id);
    marker_pub_->publish(marker_array_loc);
  }

  void addPerformanceMarker(
    visualization_msgs::msg::MarkerArray & marker_array,
    std_msgs::msg::Header header, float loc_fps_, float loc_latency_,
    int & marker_id)
  {
    auto marker = visualization_msgs::msg::Marker();
    marker.header = header;
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;

    char performance_text[100];
    snprintf(performance_text, sizeof(performance_text), "Localization:fps=%.2fHz,latency=%.2fSec",
      loc_fps_, loc_latency_);
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.text = performance_text;
    marker.pose.position.x = 0;
    marker.pose.position.y = -0.5;
    marker.pose.position.z = 0.6;
    marker_array.markers.emplace_back(marker);
  }
  /* add the marker composed by object_name, object_id, mix points, max points, 3d box bounaries*/
  void addMarker(
    visualization_msgs::msg::MarkerArray & marker_array, std_msgs::msg::Header header,
    geometry_msgs::msg::Point box_min, geometry_msgs::msg::Point box_max,
    std::string obj_name, int & marker_id)
  {
    auto name_id_text_marker =
      createNameIDMarker(header, box_min, box_max, obj_name, ++marker_id);
    auto min_text_marker = createTextMarker(header, box_min, "Min", ++marker_id);
    auto max_text_marker = createTextMarker(header, box_max, "Max", ++marker_id);
    auto box_line_marker = createBoxLineMarker(header, box_min, box_max, ++marker_id);

    marker_array.markers.emplace_back(min_text_marker);
    marker_array.markers.emplace_back(max_text_marker);
    marker_array.markers.emplace_back(name_id_text_marker);
    marker_array.markers.emplace_back(box_line_marker);
    RCLCPP_DEBUG(this->get_logger(),
      "Marker: name=%s, min(%.2f,%.2f,%.2f),max(%.2f,%.2f,%.2f)",
      obj_name.c_str(), box_min.x, box_min.y, box_min.z, box_max.x, box_max.y, box_max.z);
  }

  /* Name and ID marker */
  visualization_msgs::msg::Marker createNameIDMarker(
    std_msgs::msg::Header header,
    geometry_msgs::msg::Point box_min,
    geometry_msgs::msg::Point box_max,
    std::string & name, int & marker_id)
  {
    auto marker = visualization_msgs::msg::Marker();
    marker.header = header;
    marker.id = marker_id;

    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.z = 0.05;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    std::string name_text = name;
    marker.text = name_text;

    marker.pose.position.x = (box_min.x + box_max.x) / 2;
    marker.pose.position.y = (box_min.y + box_max.y) / 2;
    marker.pose.position.z = (box_min.z + box_max.z) / 2;
    return marker;
  }

  /* Min and Max Text Marker */
  visualization_msgs::msg::Marker createTextMarker(
    std_msgs::msg::Header header,
    geometry_msgs::msg::Point position,
    const std::string & name, int & marker_id)
  {
    auto marker = visualization_msgs::msg::Marker();
    marker.header = header;
    marker.id = marker_id;

    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.z = 0.05;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    char pos[3][20];
    snprintf(pos[0], sizeof(pos[0]), "%.2f", position.x);
    snprintf(pos[1], sizeof(pos[1]), "%.2f", position.y);
    snprintf(pos[2], sizeof(pos[2]), "%.2f", position.z);

    std::string pos_x = pos[0];
    std::string pos_y = pos[1];
    std::string pos_z = pos[2];
    std::string _text = name + "[" + pos_x + "," + pos_y + "," + pos_z + "]";
    marker.text = _text;

    marker.pose.position.x = position.x;
    marker.pose.position.y = position.y;
    marker.pose.position.z = position.z;

    return marker;
  }

  /* Box Line Marker */
  visualization_msgs::msg::Marker createBoxLineMarker(
    std_msgs::msg::Header header,
    geometry_msgs::msg::Point position_min,
    geometry_msgs::msg::Point position_max,
    int & marker_id)
  {
    auto marker = visualization_msgs::msg::Marker();
    marker.header = header;
    marker.ns = "marker_test_line_list";
    marker.id = marker_id;

    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.005;

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    geometry_msgs::msg::Point p1 = position_min;

    geometry_msgs::msg::Point p2 = p1;
    p2.x = position_max.x;

    geometry_msgs::msg::Point p3 = p2;
    p3.z = position_max.z;

    geometry_msgs::msg::Point p4 = p3;
    p4.x = position_min.x;

    geometry_msgs::msg::Point p5 = p1;
    p5.y = position_max.y;

    geometry_msgs::msg::Point p6 = p2;
    p6.y = position_max.y;

    geometry_msgs::msg::Point p7 = p3;
    p7.y = position_max.y;

    geometry_msgs::msg::Point p8 = p4;
    p8.y = position_max.y;

    // draw lines p1 -> p2 -> p3 -> p4
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p2);
    marker.points.push_back(p3);
    marker.points.push_back(p3);
    marker.points.push_back(p4);
    marker.points.push_back(p4);
    marker.points.push_back(p1);

    // draw lines p5 -> p6 -> p7 -> p8
    marker.points.push_back(p5);
    marker.points.push_back(p6);
    marker.points.push_back(p6);
    marker.points.push_back(p7);
    marker.points.push_back(p7);
    marker.points.push_back(p8);
    marker.points.push_back(p8);
    marker.points.push_back(p5);

    // draw lines 1:5, 2:6, 3:7, 4:8
    marker.points.push_back(p1);
    marker.points.push_back(p5);
    marker.points.push_back(p2);
    marker.points.push_back(p6);
    marker.points.push_back(p3);
    marker.points.push_back(p7);
    marker.points.push_back(p4);
    marker.points.push_back(p8);

    return marker;
  }

  /* localization callback for performance test */
  void loc_performance_callback(const LocalizationMsg::SharedPtr msg)
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
      RCLCPP_DEBUG(this->get_logger(), "L: fps %.3f hz, latency %.3f sec", fps, latency);
      loc_fps_ = fps;
      loc_latency_ = latency;
    }
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
  rclcpp::spin(std::make_shared<MarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}
