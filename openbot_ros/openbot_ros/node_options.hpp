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

#ifndef OPENBOT_ROS_OPENBOT_ROS_NODE_OPTIONS_HPP
#define OPENBOT_ROS_OPENBOT_ROS_NODE_OPTIONS_HPP

#include <string>
#include <tuple>

#include "openbot/common/utils/lua_parameter_dictionary.hpp"
#include "openbot/common/port.hpp"

namespace openbot_ros {

// Top-level options of Cartographer's ROS integration.
struct NodeOptions 
{
//   ::cartographer::mapping::proto::MapBuilderOptions map_builder_options;
//   std::string map_frame;
//   double lookup_transform_timeout_sec;
//   double submap_publish_period_sec;
//   double pose_publish_period_sec;
//   double trajectory_publish_period_sec;
//   bool publish_to_tf = true;
//   bool publish_tracked_pose = false;
//   bool use_pose_extrapolator = true;
};

NodeOptions CreateNodeOptions(::openbot::common::LuaParameterDictionary* lua_parameter_dictionary);

// std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
//     const std::string& configuration_directory,
//     const std::string& configuration_basename);
}  // namespace openbot_ros

#endif  // OPENBOT_ROS_OPENBOT_ROS_NODE_OPTIONS_HPP