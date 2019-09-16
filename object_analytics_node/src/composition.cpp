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
#include <rclcpp_components/node_factory.hpp>
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
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  std::vector<class_loader::ClassLoader *> loaders;
  std::vector<rclcpp_components::NodeInstanceWrapper> node_wrappers;
  rclcpp::Logger logger = rclcpp::get_logger("OA_Composition");

  std::vector<std::string> libraries;

  if (rcutils_cli_option_exist(argv, argv + argc, "--localization")) {
    libraries.push_back("libsegmenter_component.so");
  }
  if (rcutils_cli_option_exist(argv, argv + argc, "--tracking")) {
    libraries.push_back("libtracking_component.so");
  }

  for (auto library : libraries) {
    RCLCPP_INFO(logger, "Load library %s", library.c_str());
    auto loader = new class_loader::ClassLoader(library);
    auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
    for (auto clazz : classes) {
      RCLCPP_INFO(logger, "Instantiate class %s", clazz.c_str());
      auto node_factory = loader->createInstance<rclcpp_components::NodeFactory>(clazz);
      auto wrapper = node_factory->create_node_instance(options);
      auto node = wrapper.get_node_base_interface();
      node_wrappers.push_back(wrapper);
      exec.add_node(node);
    }
    loaders.push_back(loader);
  }

  exec.spin();
  for (auto wrapper : node_wrappers) {
    exec.remove_node(wrapper.get_node_base_interface());
  }
  node_wrappers.clear();

  rclcpp::shutdown();

  return 0;
}
