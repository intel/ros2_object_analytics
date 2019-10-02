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
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <class_loader/class_loader.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <thread>
#include <mutex>
#include <condition_variable>

#include "object_analytics_node/const.hpp"
#include "object_analytics_node/dataset/track_dataset.hpp"
#include "object_analytics_node/tracker/tracking_node.hpp"

#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/string.hpp"

#include "object_analytics_node/util/file_parser.hpp"

#include "opencv2/core/utility.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/plot.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/videoio.hpp"

#ifdef __clang__
namespace fs
{
class path
{
public:
  explicit path(const std::string & p)
  : path_(p) {}
  bool is_absolute() {return path_[0] == '/';}

private:
  std::string path_;
};
}  // namespace fs
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

// using namespace std;
// using namespace cv;
// using namespace datasets;

void show_usage()
{
  RCUTILS_LOG_INFO("Usage for tracker_regression:\n");
  RCUTILS_LOG_INFO(
    "tracker_regression [-a algorithm] [-p dataset_path] [-t dataset_type] [-n "
    "dataset_name] [-h]\n");
  RCUTILS_LOG_INFO("options:\n");
  RCUTILS_LOG_INFO("-h : Print this help function.\n");
  RCUTILS_LOG_INFO(
    "-a algorithm_name : Specify the tracking algorithm in the tracker.\n");
  RCUTILS_LOG_INFO(
    "   supported algorithms: KCF,TLD,BOOSTING,MEDIAN_FLOW,MIL,GOTURN\n");
  RCUTILS_LOG_INFO(
    "-p dataset_path : Specify the tracking datasets location.\n");
  RCUTILS_LOG_INFO(
    "-t dataset_type : Specify the dataset type: st_video,st_image,mt_video,mt_image.\n");
  RCUTILS_LOG_INFO("-n dataset_name : Specify the dataset name.\n");
}

class Streamer_node : public rclcpp::Node
{
public:
  Streamer_node()
  : Node("streamer")
  {
    // Create a publisher with a custom Quality of Service profile.
    pub_2d_ = this->create_publisher<sensor_msgs::msg::Image>(
      object_analytics_node::Const::kTopicRgb);

    pub_detected_objects_ =
      this->create_publisher<object_msgs::msg::ObjectsInBoxes>(
      object_analytics_node::Const::kTopicDetection);

    auto track_callback =
      [this](
      const typename object_analytics_msgs::msg::TrackedObjects::SharedPtr
      objs) -> void {this->track_cb(objs);};

    track_obj_ =
      create_subscription<object_analytics_msgs::msg::TrackedObjects>(
      object_analytics_node::Const::kTopicTracking, track_callback);



    // Use a timer to schedule periodic message publishing.
    std::chrono::milliseconds m_play(1000);
    std::chrono::milliseconds m_detect(34);
  }

  std::string mat_type2encoding(int mat_type)
  {
    switch (mat_type) {
      case CV_8UC1:
        return "mono8";
      case CV_8UC3:
        return "bgr8";
      case CV_16SC1:
        return "mono16";
      case CV_8UC4:
        return "rgba8";
      default:
        RCUTILS_LOG_DEBUG("Unsupported encoding type\n");

        return "";
    }
  }

  void track_cb(
    const object_analytics_msgs::msg::TrackedObjects::SharedPtr & objs);

  void initialDataset(
    std::string path, datasets::dsType type,
    std::string dsName)
  {
    ds_ = ds_->create(type);
    ds_->load(path);
    ds_->initDataset(dsName);

    emitThread = std::thread(&Streamer_node::emitFrame, this); 

    cv::namedWindow(window, cv::WINDOW_AUTOSIZE);
  }

  void draw(cv::Mat frame)
  {
    frame.copyTo(image_);
    for (auto t : ds_->getIdxGT(ds_->getFrameIdx())) {
      rectangle(image_, t.bb, gtColor, 2,
        cv::LINE_8);
    }
    imshow(window, image_);
    cv::waitKey(1);
  }

  void statu()
  {
    RCUTILS_LOG_INFO(
      "\r---------------------Overall status "
      "report-----------------------\n");
    RCUTILS_LOG_INFO("Overlap > 0 count(%d)\n", num_corr_);
    RCUTILS_LOG_INFO("Overlap > 0.5 count(%d)\n", num_corr_thd_);
    RCUTILS_LOG_INFO("present NUM(%d), response NUM(%d)\n", num_present_,
      num_response_);

    double precision =
      static_cast<double>(num_corr_thd_) / static_cast<double>(num_response_);
    double recall =
      static_cast<double>(num_corr_thd_) / static_cast<double>(num_present_);
    RCUTILS_LOG_INFO("Precision(%lf), recall(%lf)\n", precision, recall);
    RCUTILS_LOG_INFO(
      "------------------------------End----------------------------------"
      "\n");
  }

public:
  void emitFrame();
  void emitDetect();

  std::thread emitThread;
	std::mutex g_runlock;
	std::condition_variable g_frame_signal;

protected:
  const std::string window = "Tracking API";
  const cv::Scalar gtColor = cv::Scalar(0, 255, 0);
  cv::Ptr<datasets::trDataset> ds_;
  cv::Mat frame_, image_;

  int num_present_ = 0;
  int num_response_ = 0;
  int num_corr_ = 0;
  int num_corr_thd_ = 0;

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_2d_;
  rclcpp::Publisher<object_msgs::msg::ObjectsInBoxes>::SharedPtr
    pub_detected_objects_;
  rclcpp::Subscription<object_analytics_msgs::msg::TrackedObjects>::SharedPtr
    track_obj_;

};

void Streamer_node::emitFrame()
{
  while(true) {  
		std::unique_lock<std::mutex> locker(g_runlock);
	  g_frame_signal.wait(locker);

    if (ds_->getNextFrame(frame_)) {
      RCUTILS_LOG_DEBUG("track frame(%d)\n", ds_->getFrameIdx());
      sensor_msgs::msg::Image::SharedPtr image_br =
        std::make_shared<sensor_msgs::msg::Image>();


      cv_bridge::CvImage out_msg;
      builtin_interfaces::msg::Time stamp;

      int frameId = ds_->getFrameIdx();

      std::cout << "\n regression rgb frameId:" << frameId << "\n" << std::endl;
      stamp.sec = frameId/1000 + 1;
      stamp.nanosec = (frameId%1000)*1e6;

      out_msg.header.stamp = stamp;
      out_msg.header.frame_id = std::to_string(frameId);
      out_msg.encoding = mat_type2encoding(frame_.type());
      out_msg.image = frame_;

      image_br = out_msg.toImageMsg();
      pub_2d_->publish(image_br);


      if ((((frameId % 3) == 0) && (frameId > 2)) || (frameId ==1)) {

          if (frameId > 1) frameId -= 2;
          builtin_interfaces::msg::Time stamp;
          stamp.sec = frameId/1000 + 1;
          stamp.nanosec = (frameId%1000)*1e6;

          auto objs_in_boxes =
            std::make_shared<object_msgs::msg::ObjectsInBoxes>();

          std::cout << "\n regression detection frameId:" << frameId << "\n" << std::endl;
          for (auto t : ds_->getIdxGT(frameId)) {
            object_msgs::msg::ObjectInBox obj;
            obj.object.object_name = "test_traj";
            obj.object.probability = t.confidence * 100;

            cv::Rect2d roi = t.bb;
            obj.roi.x_offset = roi.x;
            obj.roi.y_offset = roi.y;
            obj.roi.width = roi.width;
            obj.roi.height = roi.height;

            objs_in_boxes->objects_vector.push_back(obj);
            RCUTILS_LOG_DEBUG("detect frame(%d),x(%d), y(%d), w(%d). h(%d)",
              frameId, obj.roi.x_offset, obj.roi.y_offset,
              obj.roi.width, obj.roi.height);
          }

          objs_in_boxes->header.frame_id = std::to_string(frameId);
          objs_in_boxes->header.stamp = stamp;
          objs_in_boxes->inference_time_ms = 10;
          pub_detected_objects_->publish(objs_in_boxes);

      }

      std::cout << "\n regression frameId:" << frameId << ", finished\n" << std::endl;
      num_present_++;
      
    } else {
      RCUTILS_LOG_DEBUG("-----------------Test ended!-----------------\n");
      statu();
      rclcpp::shutdown();
    }
  };

}

void Streamer_node::emitDetect()
{
    if (((ds_->getFrameIdx() % 4) == 0) && (ds_->getFrameIdx() > 3)) {
      int frameId = ds_->getFrameIdx() - 3;
      builtin_interfaces::msg::Time stamp;
      stamp.sec = frameId/1000;
      stamp.nanosec = (frameId%1000)*1e6;

      auto objs_in_boxes =
        std::make_shared<object_msgs::msg::ObjectsInBoxes>();

      for (auto t : ds_->getIdxGT(frameId)) {
        object_msgs::msg::ObjectInBox obj;
        obj.object.object_name = "test_traj";
        obj.object.probability = t.confidence * 100;

        cv::Rect2d roi = t.bb;
        obj.roi.x_offset = roi.x;
        obj.roi.y_offset = roi.y;
        obj.roi.width = roi.width;
        obj.roi.height = roi.height;

        objs_in_boxes->objects_vector.push_back(obj);
        RCUTILS_LOG_DEBUG("detect frame(%d),x(%d), y(%d), w(%d). h(%d)",
          frameId, obj.roi.x_offset, obj.roi.y_offset,
          obj.roi.width, obj.roi.height);
      }

      objs_in_boxes->header.frame_id = std::to_string(frameId);
      objs_in_boxes->header.stamp = stamp;
      objs_in_boxes->inference_time_ms = 10;
      pub_detected_objects_->publish(objs_in_boxes);

    }
}

void Streamer_node::track_cb(
  const object_analytics_msgs::msg::TrackedObjects::SharedPtr & objs)
{

  int frame_id = std::stoi(objs->header.frame_id, nullptr, 0);
  RCUTILS_LOG_INFO("track_cb: get tracked frameId(%s),(%d)\n",
    objs->header.frame_id.c_str(), frame_id);

  if (objs->tracked_objects.size() > 0) {
    num_response_++;
    RCUTILS_LOG_INFO("objs count(%d)", objs->tracked_objects.size());
  }

  ds_->getIdxFrame(image_, frame_id);

  putText(image_, std::to_string(frame_id), cv::Point(0, 15), cv::FONT_HERSHEY_PLAIN, 1 , cv::Scalar(0,0,255), 1, cv::LINE_AA);
  for (auto t : objs->tracked_objects) {
    cv::Rect2d obj_roi(t.roi.x_offset, t.roi.y_offset, t.roi.width,
      t.roi.height);

    // TBD: should check object ID for comparison.
    RCUTILS_LOG_DEBUG(" frame_id (%d)", objs->header.stamp.nanosec);
    RCUTILS_LOG_DEBUG(" obj_id (%d)", t.id);
    RCUTILS_LOG_DEBUG(" probability (%f)", t.object.probability);
    RCUTILS_LOG_DEBUG(" x_offset (%d), y_offset(%d)", t.roi.x_offset, t.roi.y_offset);
    RCUTILS_LOG_DEBUG(" width (%d), height(%d)", t.roi.width, t.roi.height);

    cv::Rect2d track_rect(t.roi.x_offset, t.roi.y_offset, t.roi.width,
      t.roi.height);
    putText(image_, std::to_string(t.id), track_rect.tl(), cv::FONT_HERSHEY_PLAIN, 1 , cv::Scalar(255,0,0), 1, cv::LINE_AA);
    rectangle(image_, track_rect, cv::Scalar(255, 0, 0), 1, cv::LINE_8);

  }

  for (auto it : ds_->getIdxGT(frame_id)) {
    cv::Rect2d gt_roi = it.bb;

    // TBD: should check object ID for comparison.
//    RCUTILS_LOG_INFO(" gt.x(%f), gt.y(%f)", gt_roi.x, gt_roi.y);
//    RCUTILS_LOG_INFO(" gt.width (%f), gt.height(%f)", gt_roi.width, gt_roi.height);

    rectangle(image_, gt_roi, cv::Scalar(0, 255, 0), 1, cv::LINE_8);
  }

  imshow(window, image_);

#if 1
  int key = cv::waitKey(10);
  while(key != 0x20 && key != 0x71)
  {
    key = cv::waitKey(100);
  }

  if (key == 0x71)
    exit(0);
#endif
//	g_frame_signal.notify_all();
}

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Parse the command line options.
  std::string dsPath, dsName, dType;
  datasets::dsType dsTpy;
  std::string algo;

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    show_usage();
    return 0;
  }

  if (rcutils_cli_option_exist(argv, argv + argc, "-a")) {
    algo = rcutils_cli_get_option(argv, argv + argc, "-a");
  }

  if (rcutils_cli_option_exist(argv, argv + argc, "-p")) {
    dsPath = rcutils_cli_get_option(argv, argv + argc, "-p");
  }

  if (rcutils_cli_option_exist(argv, argv + argc, "-t")) {
    dType = rcutils_cli_get_option(argv, argv + argc, "-t");
    if (dType == "st_image") {
      dsTpy = datasets::dsSTImage;
    } else if (dType == "st_video") {
      dsTpy = datasets::dsSTVideo;
    } else if (dType == "mt_image") {
      dsTpy = datasets::dsMTImage;
    } else {
      return 0;
    }
  }

  if (rcutils_cli_option_exist(argv, argv + argc, "-n")) {
    dsName = rcutils_cli_get_option(argv, argv + argc, "-n");
  }

  if (dsPath == "" || dsName == "" || dType == "") {
    RCUTILS_LOG_DEBUG("Please specfic below options:\n");
    show_usage();
    return 0;
  }
  // Initialize any global resources needed by the middleware and the client
  // library.
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  auto t_node = std::make_shared<Streamer_node>();
  t_node->initialDataset(dsPath, dsTpy, dsName);

//  exec.add_node(t_node);

  rclcpp::NodeOptions options;
  // Create track node.
  auto r_node =
    std::make_shared<object_analytics_node::TrackingNode>(options);
  // TBD: Add algo interface to chose algorithm for tracking node, currently use
  //      default algorithm as MEDIAN_FLOW.
  r_node->setAlgo(algo);
//  exec.add_node(r_node);

  // spin will block until work comes in, execute work as it becomes available,
  // and keep blocking. It will only be interrupted by Ctrl-C.
  
  rclcpp::WallRate loop_rate(2);

  //  exec.spin();
  while(rclcpp::ok())
  {
    loop_rate.sleep();
    t_node->g_frame_signal.notify_all();
    rclcpp::spin_some(t_node);
    loop_rate.sleep();
    rclcpp::spin_some(r_node);
  }



  exec.remove_node(r_node);
  exec.remove_node(t_node);

  rclcpp::shutdown();
  return 0;
}
