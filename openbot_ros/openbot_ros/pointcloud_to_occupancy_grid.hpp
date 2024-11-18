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

#ifndef OPENBOT_ROS_POINTCLOUD_TO_OCCUPANCY_GRID_HPP
#define OPENBOT_ROS_POINTCLOUD_TO_OCCUPANCY_GRID_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp> 
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <octomap/OcTree.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_octomap/GridMapOctomapConverter.hpp>

namespace openbot_ros {

class PointCloudToOccupancyGridNode : public rclcpp::Node
{
public:
  PointCloudToOccupancyGridNode();

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
};

}  // namespace openbot_ros

#endif  // OPENBOT_ROS_POINTCLOUD_TO_OCCUPANCY_GRID_HPP
