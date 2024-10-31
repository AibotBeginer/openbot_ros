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

#ifndef OPENBOT_ROS_OPENBOT_ROS_TIME_CONVERSION_HPP
#define OPENBOT_ROS_OPENBOT_ROS_TIME_CONVERSION_HPP

#include "openbot/common/time.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

namespace openbot_ros {

rclcpp::Time ToRosTime(::openbot::common::Time time);

::openbot::common::Time FromRosTime(const rclcpp::Time& time);

}  // namespace openbot_ros

#endif  // OPENBOT_ROS_OPENBOT_ROS_TIME_CONVERSION_HPP
