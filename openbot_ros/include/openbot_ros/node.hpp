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

#ifndef OPENBOT_ROS_OPENBOT_ROS_NODE_HPP
#define OPENBOT_ROS_OPENBOT_ROS_NODE_HPP

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "openbot_ros/node_options.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace openbot_ros {
class Node 
{
public:
    Node(const NodeOptions& node_options,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        rclcpp::Node::SharedPtr node,
        bool collect_metrics);
    ~Node();

    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

private:
    const NodeOptions node_options_;
};

}  // namespace openbot_ros

#endif  // OPENBOT_ROS_OPENBOT_ROS_NODE_HPP