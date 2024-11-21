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

#ifndef OPENBOT_ROS_GLOBAL_PLANNER_HPP
#define OPENBOT_ROS_GLOBAL_PLANNER_HPP

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp> 

#include <memory>
#include <vector>
#include <string>

#include <Eigen/Eigen>

#include "openbot/planning/planner_server.hpp"

#include "openbot_ros/messages_conversion/proto_msgs_to_ros.hpp"
#include "openbot_ros/messages_conversion/ros_msgs_to_proto.hpp"
#include "openbot_ros/map_generator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

namespace openbot_ros {

class GlobalPlanner 
{
public:

    OPENBOT_SMART_PTR_DEFINITIONS(GlobalPlanner)

    GlobalPlanner();

    void AddPose(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

    bool CheckValid(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
    
    std::vector<Eigen::Vector3d>& start_goal();

    nav_msgs::msg::Path CreatePath(const double timeout);

    void CreateGlobalMap(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud);
    
    sensor_msgs::msg::PointCloud2& map_data();

private:
    // planner server
    ::openbot::planning::PlannerServer::SharedPtr planner_{nullptr};
    // start & goal
    std::vector<Eigen::Vector3d> start_goal_;

    // map_generator
    RandomMapGenerator::SharedPtr map_generator_{nullptr};
    sensor_msgs::msg::PointCloud2 map_data_;
};

}  // namespace openbot_ros

#endif  // OPENBOT_ROS_GLOBAL_PLANNER_HPP
