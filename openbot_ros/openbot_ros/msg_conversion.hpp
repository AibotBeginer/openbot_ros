/*
 * Copyright 2016 The Cartographer Authors
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

#ifndef OPENBOT_ROS_OPENBOT_ROS_MSG_CONVERSION_HPP
#define OPENBOT_ROS_OPENBOT_ROS_MSG_CONVERSION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <rclcpp/rclcpp.hpp>

namespace openbot_ros {

// geometry_msgs::msg::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d);

// Eigen::Vector3d ToEigen(const geometry_msgs::msg::Vector3& vector3);

// Eigen::Quaterniond ToEigen(const geometry_msgs::msg::Quaternion& quaternion);


}  // namespace openbot_ros

#endif  // OPENBOT_ROS_OPENBOT_ROS_MSG_CONVERSION_HPP
