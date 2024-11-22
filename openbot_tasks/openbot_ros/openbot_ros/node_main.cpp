/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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


#include "openbot_ros/node.hpp"
#include "openbot_ros/node_constants.hpp"

#include "gflags/gflags.h"
#include "glog/logging.h"

#include "tf2_ros/transform_listener.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");

namespace openbot_ros {
namespace {

void Usage()
{
  LOG(INFO) << "welcome openbot world.";
}

void Run() 
{
  Usage();

  NodeOptions node_options;
  auto node = std::make_shared<openbot_ros::Node>(node_options, FLAGS_collect_metrics);
  rclcpp::spin(node);
}

}  // namespace
}  // namespace openbot_ros

int main(int argc, char** argv) 
{
  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  // CHECK(!FLAGS_configuration_directory.empty())
  //     << "-configuration_directory is missing.";
  // CHECK(!FLAGS_configuration_basename.empty())
  //     << "-configuration_basename is missing.";

  // openbot_ros::ScopedRosLogSink ros_log_sink;
  openbot_ros::Run();
  google::ShutdownGoogleLogging();
  ::rclcpp::shutdown();
}
