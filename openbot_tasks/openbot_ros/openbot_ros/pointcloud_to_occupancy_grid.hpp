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
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace openbot_ros {

class PointCloudToOccupancyGridNode : public rclcpp::Node
{
public:
  PointCloudToOccupancyGridNode();

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void transformPointCloud();
  void simulateRgbdCamera(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_camera_frame_pub_;
  std::string base_frame_id_;  // base of the robot for camera transformation
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  sensor_msgs::msg::PointCloud2::SharedPtr cloud_in_;
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
};

}  // namespace openbot_ros

#endif  // OPENBOT_ROS_POINTCLOUD_TO_OCCUPANCY_GRID_HPP
