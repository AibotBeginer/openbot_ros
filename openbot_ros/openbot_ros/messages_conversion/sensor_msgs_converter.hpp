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

#ifndef OPENBOT_ROS_MESSAGES_CONVERSION_SENSOR_MSGS_CONVERTER_HPP
#define OPENBOT_ROS_MESSAGES_CONVERSION_SENSOR_MSGS_CONVERTER_HPP

#include <string>
#include <tuple>

// ros sensor_msgs
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/channel_float32.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/illuminance.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"


namespace openbot_ros {

// PointField
sensor_msgs::msg::PointField ToRos(const ::openbot::common::sensor_msgs::PointField& data);
// ::openbot::common::sensor_msgs::PointField FromRos(const sensor_msgs::msg::PointField& ros);

// PointCloud2
sensor_msgs::msg::PointCloud2 ToRos(const ::openbot::common::sensor_msgs::PointCloud2& data);
// ::openbot::common::sensor_msgs::PointCloud2 FromRos(const sensor_msgs::msg::PointCloud2& ros);

}  // namespace openbot_ros

#endif  // OPENBOT_ROS_MESSAGES_CONVERSION_SENSOR_MSGS_CONVERTER_HPP