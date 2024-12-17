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

#include "std_msgs_converter.hpp"
#include "builtin_interfaces_converter.hpp"

namespace openbot_ros {

// std_msgs::msg::Header ToRos(const ::openbot::common::std_msgs::Header& data)
// {
//     std_msgs::msg::Header ros;
//     ros.stamp = ToRos(data.stamp);
//     ros.frame_id = data.frame_id;
//     return ros;
// }

// ::openbot::common::std_msgs::Header FromRos(const std_msgs::msg::Header& ros)
// {
//     ::openbot::common::std_msgs::Header data;
//     data.stamp = FromRos(ros.stamp);
//     data.frame_id = ros.frame_id;
//     return data;
// }

}  // namespace openbot_ros