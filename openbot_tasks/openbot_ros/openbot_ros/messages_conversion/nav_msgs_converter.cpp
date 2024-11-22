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

#include "nav_msgs_converter.hpp"
#include "geometry_msgs_converter.hpp"
#include "std_msgs_converter.hpp"

namespace openbot_ros {

// Path
nav_msgs::msg::Path ToRos(const ::openbot::common::nav_msgs::Path& data)
{
    nav_msgs::msg::Path ros;
    ros.header.frame_id = data.header.frame_id;
    for (auto pose : data.poses) {
        ros.poses.push_back(ToRos(pose));
    }
    return ros;
}

// ::openbot::common::nav_msgs::Path FromRos(const nav_msgs::msg::Path& ros)
// {
//     ::openbot::common::nav_msgs::Path data;
//     return data;
// }

}  // namespace openbot_ros