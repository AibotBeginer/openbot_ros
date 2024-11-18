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
#include "openbot_ros/pointcloud_to_occupancy_grid.hpp"


namespace openbot_ros
{
  PointCloudToOccupancyGridNode::PointCloudToOccupancyGridNode() : rclcpp::Node("pointcloud_to_occupancy_grid_node")
  {
    // Initialize subscriber and publishers
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "global_map", 10,
        std::bind(&PointCloudToOccupancyGridNode::pointCloudCallback, this, std::placeholders::_1));

    occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
  }

  void PointCloudToOccupancyGridNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
;
  }

} // namespace openbot_ros