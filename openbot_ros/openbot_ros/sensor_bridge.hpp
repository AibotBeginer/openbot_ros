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

#ifndef OPENBOT_ROS_OPENBOT_ROS_SENSOR_BRIDGE_HPP
#define OPENBOT_ROS_OPENBOT_ROS_SENSOR_BRIDGE_HPP

#include <memory>

#include "absl/types/optional.h"
#include "openbot_ros/tf_bridge.hpp"

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace openbot_ros {

// Converts ROS messages into SensorData in tracking frame for the MapBuilder.
class SensorBridge 
{
public:
  explicit SensorBridge(
        const std::string& tracking_frame,
        double lookup_transform_timeout_sec, 
        tf2_ros::Buffer* tf_buffer);

  SensorBridge(const SensorBridge&) = delete;
  SensorBridge& operator=(const SensorBridge&) = delete;

  void HandleOdometryMessage(const std::string& sensor_id, const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

  void HandleImuMessage(const std::string& sensor_id, const sensor_msgs::msg::Imu::ConstSharedPtr& msg);

  void HandleImageMessage(const std::string& sensor_id, const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  void HandleImageMessage(const std::string& sensor_id, 
    const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& right_msg);


  const TfBridge& tf_bridge() const;

private:

     const TfBridge tf_bridge_;
};

}  // namespace openbot_ros

#endif  // OPENBOT_ROS_OPENBOT_ROS_SENSOR_BRIDGE_HPP
