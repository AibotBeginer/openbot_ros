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

#ifndef OPENBOT_ROS_VISUALIZATION_LOCAL_PLANNER_VISUALIZATION_HPP
#define OPENBOT_ROS_VISUALIZATION_LOCAL_PLANNER_VISUALIZATION_HPP

#include <string>
#include <tuple>

#include <rclcpp/rclcpp.hpp>

#include "openbot_ros/messages_conversion/proto_msgs_to_ros.hpp"
#include "openbot_ros/messages_conversion/ros_msgs_to_proto.hpp"

namespace openbot_ros {

class LocalPlannerVisualizator
{
public:

    /**
     *  @brief SharedPtr typedef
     */
    RCLCPP_SMART_PTR_DEFINITIONS(LocalPlannerVisualizator)

    /**
     *  @brief SharedPtr typedef
     */
    LocalPlannerVisualizator(rclcpp::Node* node);
    

private:
    rclcpp::Node* node_{nullptr};
};

}  // namespace openbot_ros

#endif  // OPENBOT_ROS_VISUALIZATION_LOCAL_PLANNER_VISUALIZATION_HPP