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

#ifndef OPENBOT_ROS_OPENBOT_PROTO_MSGS_TO_ROS_HPP
#define OPENBOT_ROS_OPENBOT_PROTO_MSGS_TO_ROS_HPP

#include <string>
#include <tuple>

// geometry_msgs
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp" 
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/velocity_stamped.hpp"

// sensor_msgs
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

/////////////////////////////////////////////////////////////////////////////////////////////////////

// geometry_msgs
#include "openbot/common/port.hpp"
#include "openbot/common/proto/std_msgs/header.pb.h"
#include "openbot/common/proto/geometry_msgs/twist.pb.h"  
#include "openbot/common/proto/geometry_msgs/twist_stamped.pb.h"  
#include "openbot/common/proto/geometry_msgs/twist_with_covariance.pb.h"
#include "openbot/common/proto/geometry_msgs/twist_with_covariance_stamped.pb.h"
#include "openbot/common/proto/geometry_msgs/vector3.pb.h"  
#include "openbot/common/proto/geometry_msgs/vector3_stamped.pb.h" 
#include "openbot/common/proto/geometry_msgs/velocity_stamped.pb.h" 
#include "openbot/common/proto/builtin_interfaces/time.pb.h" 
#include "openbot/common/proto/builtin_interfaces/duration.pb.h"

// sensor_msgs
#include "openbot/common/proto/sensor_msgs/camera_info.pb.h" 
#include "openbot/common/proto/sensor_msgs/channel_float32.pb.h" 
#include "openbot/common/proto/sensor_msgs/compressed_image.pb.h"
#include "openbot/common/proto/sensor_msgs/illuminance.pb.h"
#include "openbot/common/proto/sensor_msgs/image.pb.h"
#include "openbot/common/proto/sensor_msgs/imu.pb.h" 
#include "openbot/common/proto/sensor_msgs/laser_scan.pb.h"
#include "openbot/common/proto/sensor_msgs/point_cloud.pb.h"
#include "openbot/common/proto/sensor_msgs/point_cloud2.pb.h"
#include "openbot/common/proto/sensor_msgs/point_field.pb.h"
#include "openbot/common/proto/sensor_msgs/range.pb.h"
#include "openbot/common/proto/sensor_msgs/region_of_interest.pb.h"

namespace openbot_ros {

//------------------------------------------ builtin_interfaces -------------------------------------

// Time
rclcpp::Time ToRos(const ::openbot::common::proto::builtin_interfaces::Time& proto);

//------------------------------------------ geometry_msgs ------------------------------------------

// Twist
geometry_msgs::msg::Twist ToRos(const ::openbot::common::proto::geometry_msgs::Twist& proto);
geometry_msgs::msg::TwistStamped ToRos(const ::openbot::common::proto::geometry_msgs::TwistStamped& proto);
geometry_msgs::msg::TwistWithCovariance ToRos(const ::openbot::common::proto::geometry_msgs::TwistWithCovariance& proto);
geometry_msgs::msg::TwistWithCovarianceStamped ToRos(const ::openbot::common::proto::geometry_msgs::TwistWithCovarianceStamped& proto);

// Vector3
geometry_msgs::msg::Vector3 ToRos(const ::openbot::common::proto::geometry_msgs::Vector3& proto);
geometry_msgs::msg::Vector3Stamped ToRos(const ::openbot::common::proto::geometry_msgs::Vector3Stamped& proto);
geometry_msgs::msg::VelocityStamped ToRos(const ::openbot::common::proto::geometry_msgs::VelocityStamped& proto);

//------------------------------------------ nav_msgs ------------------------------------------


//------------------------------------------ sensor_msgs ------------------------------------------

// CameraInfo
sensor_msgs::msg::CameraInfo ToRos(const ::openbot::common::proto::sensor_msgs::CameraInfo& proto);

// ChannelFloat32
sensor_msgs::msg::ChannelFloat32 ToRos(const ::openbot::common::proto::sensor_msgs::ChannelFloat32& proto);

// CompressedImage
sensor_msgs::msg::CompressedImage ToRos(const ::openbot::common::proto::sensor_msgs::CompressedImage& proto);

// Illuminance
sensor_msgs::msg::Illuminance ToRos(const ::openbot::common::proto::sensor_msgs::Illuminance& proto);

// Image
sensor_msgs::msg::Image ToRos(const ::openbot::common::proto::sensor_msgs::Image& proto);

// Imu
sensor_msgs::msg::Imu ToRos(const ::openbot::common::proto::sensor_msgs::Imu& proto);

// LaserScan
sensor_msgs::msg::LaserScan ToRos(const ::openbot::common::proto::sensor_msgs::LaserScan& proto);

// PointCloud
sensor_msgs::msg::PointCloud ToRos(const ::openbot::common::proto::sensor_msgs::PointCloud& proto);

// PointCloud2
sensor_msgs::msg::PointCloud2 ToRos(const ::openbot::common::proto::sensor_msgs::PointCloud2& proto);

// PointField
sensor_msgs::msg::PointField ToRos(const ::openbot::common::proto::sensor_msgs::PointField& proto);

// Range
sensor_msgs::msg::Range ToRos(const ::openbot::common::proto::sensor_msgs::Range& proto);

// RegionOfInterest
sensor_msgs::msg::RegionOfInterest ToRos(const ::openbot::common::proto::sensor_msgs::RegionOfInterest& proto);

//------------------------------------------ shape_msgs ------------------------------------------


//------------------------------------------ std_msgs --------------------------------------------

// Header
std_msgs::msg::Header ToRos(const ::openbot::common::proto::std_msgs::Header& proto);


//------------------------------------------ vision_msgs --------------------------------------------


}  // namespace openbot_ros

#endif  // OPENBOT_ROS_OPENBOT_PROTO_MSGS_TO_ROS_HPP