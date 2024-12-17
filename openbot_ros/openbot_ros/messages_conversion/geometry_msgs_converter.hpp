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

#ifndef OPENBOT_ROS_MESSAGES_CONVERSION_GEOMETRY_MSGS_CONVERTER_HPP
#define OPENBOT_ROS_MESSAGES_CONVERSION_GEOMETRY_MSGS_CONVERTER_HPP

#include <string>
#include <tuple>

// nav_msgs
#include "nav_msgs/msg/grid_cells.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace openbot_ros {

// // Point
// geometry_msgs::msg::Point ToRos(const ::openbot::common::geometry_msgs::Point& data);
// ::openbot::common::geometry_msgs::Point FromRos(const geometry_msgs::msg::Point& ros);

// // Quaternion
// geometry_msgs::msg::Quaternion ToRos(const ::openbot::common::geometry_msgs::Quaternion& data);
// ::openbot::common::geometry_msgs::Quaternion FromRos(const geometry_msgs::msg::Quaternion& ros);

// // Pose
// geometry_msgs::msg::Pose ToRos(const ::openbot::common::geometry_msgs::Pose& data);
// ::openbot::common::geometry_msgs::Pose FromRos(const geometry_msgs::msg::Pose& ros);

// // PoseStamped
// geometry_msgs::msg::PoseStamped ToRos(const ::openbot::common::geometry_msgs::PoseStamped& data);
// ::openbot::common::geometry_msgs::PoseStamped FromRos(const geometry_msgs::msg::PoseStamped& ros);


}  // namespace openbot_ros

#endif  // OPENBOT_ROS_MESSAGES_CONVERSION_GEOMETRY_MSGS_CONVERTER_HPP