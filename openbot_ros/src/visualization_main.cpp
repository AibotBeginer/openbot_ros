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


#include "openbot_ros/visualization.hpp"

#include <thread>
#include <chrono>

#include "gflags/gflags.h"
#include "glog/logging.h"

#include "openbot/common/proto/nav_msgs/path.pb.h" 
#include "openbot/common/msgs/msgs.hpp"

namespace openbot_ros {
namespace {

void Run() 
{
 
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("openbot_visualization_node");
  auto visualization = std::make_shared<openbot_ros::Visualization>(node.get());
  
  ::openbot::common::nav_msgs::Path path;
  path.header.frame_id = "map";

  for (int i = 0; i < 100; ++i) {
        ::openbot::common::geometry_msgs::PoseStamped pose;
      // position
      pose.pose.position.x = i * 1.0;
      pose.pose.position.y = i * 1.0;
      pose.pose.position.z = 0.0;


      // orientation
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.z = 1.0;
      path.poses.push_back(pose);
  }

  while (true) {
    ::openbot::common::proto::nav_msgs::Path proto = ::openbot::common::nav_msgs::ToProto(path);
    visualization->PublishPath(proto);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  rclcpp::spin(node);
}

}  // namespace
}  // namespace openbot_ros

int main(int argc, char** argv) 
{
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  openbot_ros::Run();
  google::ShutdownGoogleLogging();
  ::rclcpp::shutdown();
}
