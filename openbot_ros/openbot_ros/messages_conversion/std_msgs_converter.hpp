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

#ifndef OPENBOT_ROS_MESSAGES_CONVERSION_STD_MSGS_CONVERTER_HPP
#define OPENBOT_ROS_MESSAGES_CONVERSION_STD_MSGS_CONVERTER_HPP

#include <string>
#include <tuple>

// std_msgs
#include "std_msgs/msg/header.hpp"

#include "nav_msgs/msg/grid_cells.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "openbot/common/msgs/msgs.hpp"

namespace openbot_ros {

// Header
std_msgs::msg::Header ToRos(const ::openbot::common::std_msgs::Header& data);
::openbot::common::std_msgs::Header FromRos(const std_msgs::msg::Header& ros);


}  // namespace openbot_ros

#endif  // OPENBOT_ROS_MESSAGES_CONVERSION_STD_MSGS_CONVERTER_HPP