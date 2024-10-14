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


#include "gflags/gflags.h"
#include "glog/logging.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace openbot_ros {
namespace {

void Run() {
  rclcpp::Node::SharedPtr openbot_node = rclcpp::Node::make_shared("openbot_node");
  constexpr double kTfBufferCacheTimeInSeconds = 10.;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_shared<tf2_ros::Buffer>(openbot_node->get_clock(), 
        tf2::durationFromSec(kTfBufferCacheTimeInSeconds), openbot_node);

  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // NodeOptions node_options;
  // TrajectoryOptions trajectory_options;
  // std::tie(node_options, trajectory_options) =
  //     LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  // auto map_builder =
  //   cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  // auto node = std::make_shared<cartographer_ros::Node>(
  //   node_options, std::move(map_builder), tf_buffer, cartographer_node,
  //   FLAGS_collect_metrics);
  // if (!FLAGS_load_state_filename.empty()) {
  //   node->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  // }

  // if (FLAGS_start_trajectory_with_default_topics) {
  //   node->StartTrajectoryWithDefaultTopics(trajectory_options);
  // }

  rclcpp::spin(openbot_node);

  // node->FinishAllTrajectories();
  // node->RunFinalOptimization();

  // if (!FLAGS_save_state_filename.empty()) {
  //   node->SerializeState(FLAGS_save_state_filename,
  //                       true /* include_unfinished_submaps */);
  // }
}

}  // namespace
}  // namespace cartographer_ros

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
  ::rclcpp::shutdown();
}
