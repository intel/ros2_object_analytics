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

#include <class_loader/class_loader.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <rclcpp/rclcpp.hpp>

#ifdef __clang__
#include <string>
namespace fs
{
class path
{
public:
  explicit path(const std::string & p)
  : path_(p)
  {
  }
  bool is_absolute()
  {
    return path_[0] == '/';
  }

private:
  std::string path_;
};
}  // namespace fs
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

#include <rcutils/cmdline_parser.h>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include "object_analytics_node/util/file_parser.hpp"

int main(int argc, char * argv[])
{
  // force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("object_analytics_node");

  std::vector<std::pair<std::string, std::string>> clazzes;
  clazzes.push_back(
    std::make_pair("object_analytics_node", "object_analytics_node::splitter::SplitterNode"));
  if (rcutils_cli_option_exist(argv, argv + argc, "--localization")) {
    clazzes.push_back(
      std::make_pair("object_analytics_node", "object_analytics_node::segmenter::SegmenterNode"));
    clazzes.push_back(
      std::make_pair("object_analytics_node", "object_analytics_node::movement::MovementNode"));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, "--tracking")) {
    clazzes.push_back(
      std::make_pair("object_analytics_node", "object_analytics_node::tracker::TrackingNode"));
  }

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  std::vector<class_loader::ClassLoader *> loaders;
  std::vector<std::shared_ptr<rclcpp::Node>> nodes;

  for (auto plugin_clazz : clazzes) {
    const std::string package_name = plugin_clazz.first;
    const std::string clazz_name = plugin_clazz.second;

    std::string content;
    std::string base_path;
    if (!ament_index_cpp::get_resource("node_plugin", package_name, content, &base_path)) {
      RCLCPP_ERROR(
        node->get_logger(), "Could not find requested resource %s in ament index", package_name);
      return 2;
    }

    std::vector<std::string> lines =
      object_analytics_node::util::FileParser::split(content, '\n', true);
    for (auto line : lines) {
      std::vector<std::string> parts = object_analytics_node::util::FileParser::split(line, ';');
      if (parts.size() != 2) {
        RCLCPP_ERROR(node->get_logger(), "Invalid resource entry");
        return 2;
      }

      if (clazz_name != parts[0]) {
        continue;
      }

      std::string library_path = parts[1];
      if (!fs::path(library_path).is_absolute()) {
        library_path = base_path + "/" + library_path;
      }
      RCLCPP_INFO(node->get_logger(), "Load library %s", library_path.c_str());

      try {
        class_loader::ClassLoader * loader = new class_loader::ClassLoader(library_path);
        if (!loader->isClassAvailable<rclcpp::Node>(clazz_name)) {
          RCLCPP_ERROR(node->get_logger(), "%s is not available", clazz_name);
          return 2;
        }
        RCLCPP_INFO(node->get_logger(), "Instantiate class %s", clazz_name.c_str());
        auto node = loader->createInstance<rclcpp::Node>(clazz_name);
        exec.add_node(node);
        nodes.push_back(node);
        loaders.push_back(loader);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load library: %s", ex.what());
        return 3;
      } catch (...) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load library");
        return 3;
      }
    }
  }

  exec.spin();
  for (auto node : nodes) {
    exec.remove_node(node);
  }
  nodes.clear();

  rclcpp::shutdown();
  return 0;
}
