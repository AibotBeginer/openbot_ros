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

#include "geometry_msgs_converter.hpp"
#include "std_msgs_converter.hpp"

namespace openbot_ros {

// geometry_msgs::msg::Point ToRos(const ::openbot::common::geometry_msgs::Point& data)
// {
//     geometry_msgs::msg::Point ros;
//     ros.x = data.x;
//     ros.y = data.y;
//     ros.z = data.z;
//     return ros;
// }

// ::openbot::common::geometry_msgs::Point FromRos(const geometry_msgs::msg::Point& ros)
// {
//     ::openbot::common::geometry_msgs::Point data;
//     data.x = ros.x;
//     data.y = ros.y;
//     data.z = ros.z;
//     return data;
// }

// geometry_msgs::msg::Quaternion ToRos(const ::openbot::common::geometry_msgs::Quaternion& data)
// {
//     geometry_msgs::msg::Quaternion ros;
//     ros.x = data.x;
//     ros.y = data.y;
//     ros.z = data.z;
//     ros.w = data.w;
//     return ros;
// }

// ::openbot::common::geometry_msgs::Quaternion FromRos(const geometry_msgs::msg::Quaternion& ros)
// {
//     ::openbot::common::geometry_msgs::Quaternion data;
//     data.x = ros.x;
//     data.y = ros.y;
//     data.z = ros.z;
//     data.w = ros.w;
//     return data;
// }

// geometry_msgs::msg::Pose ToRos(const ::openbot::common::geometry_msgs::Pose& data)
// {   
//     geometry_msgs::msg::Pose ros;
//     ros.position = ToRos(data.position);
//     ros.orientation = ToRos(data.orientation);
//     return ros;
// }

// ::openbot::common::geometry_msgs::Pose FromRos(const geometry_msgs::msg::Pose& ros)
// {
//     ::openbot::common::geometry_msgs::Pose data;
//     data.position = FromRos(ros.position);
//     data.orientation = FromRos(ros.orientation);
//     return data;
// }

// geometry_msgs::msg::PoseStamped ToRos(const ::openbot::common::geometry_msgs::PoseStamped& data)
// {
//     geometry_msgs::msg::PoseStamped ros;
//     ros.header = ToRos(data.header);
//     ros.pose = ToRos(data.pose);
//     return ros;
// }

// ::openbot::common::geometry_msgs::PoseStamped FromRos(const geometry_msgs::msg::PoseStamped& ros)
// {
//     ::openbot::common::geometry_msgs::PoseStamped data;
//     data.header = FromRos(ros.header);
//     data.pose = FromRos(ros.pose);
//     return data;
// }

}  // namespace openbot_ros