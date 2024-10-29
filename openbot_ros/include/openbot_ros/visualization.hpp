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

#ifndef OPENBOT_ROS_OPENBOT_ROS_VISUALIZATION_HPP
#define OPENBOT_ROS_OPENBOT_ROS_VISUALIZATION_HPP

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "messages_conversion/proto_msgs_to_ros.hpp"
#include "messages_conversion/ros_msgs_to_proto.hpp"

// #include "openbot/common/proto/nav_msgs/grid_cells.pb.h" 
// #include "openbot/common/proto/nav_msgs/map_meta_data.pb.h"
// #include "openbot/common/proto/nav_msgs/occupancy_grid.pb.h" 
// #include "openbot/common/proto/nav_msgs/odometry.pb.h"
// #include "openbot/common/proto/nav_msgs/path.pb.h" 

namespace openbot_ros {

class Visualization
{
public:
    using ProtoPath = ::openbot::common::proto::nav_msgs::Path;

    Visualization(rclcpp::Node* node);
    
    void PublishPath(const ProtoPath& path);

private:
    rclcpp::Node* node_{nullptr};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_{nullptr};
};

}  // namespace openbot_ros

#endif  // OPENBOT_ROS_OPENBOT_ROS_VISUALIZATION_HPP
