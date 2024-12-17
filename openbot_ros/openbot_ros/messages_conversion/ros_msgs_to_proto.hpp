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

#ifndef OPENBOT_ROS_OPENBOT_ROS_MSGS_TO_PROTO_HPP
#define OPENBOT_ROS_OPENBOT_ROS_MSGS_TO_PROTO_HPP

#include <string>
#include <tuple>

#include "rclcpp/rclcpp.hpp"

// geometry_msgs
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/accel_with_covariance.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/inertia.hpp"
#include "geometry_msgs/msg/inertia_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp" 
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/velocity_stamped.hpp"

// nav_msgs
#include "nav_msgs/msg/grid_cells.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"


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

// shape_msgs
#include "shape_msgs/msg/mesh.hpp"
#include "shape_msgs/msg/mesh_triangle.hpp"
#include "shape_msgs/msg/plane.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"


// std_msgs
#include "std_msgs/msg/header.hpp"

// vision_msgs
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box3_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"
#include "vision_msgs/msg/bounding_box3_d_array.hpp"
#include "vision_msgs/msg/classification.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/label_info.hpp"
#include "vision_msgs/msg/object_hypothesis.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"
#include "vision_msgs/msg/point2_d.hpp"
#include "vision_msgs/msg/pose2_d.hpp"
#include "vision_msgs/msg/vision_class.hpp"
#include "vision_msgs/msg/vision_info.hpp"

// geometry_msgs
#include "openbot/common/proto/geometry_msgs/accel.pb.h"  
#include "openbot/common/proto/geometry_msgs/accel_stamped.pb.h"  
#include "openbot/common/proto/geometry_msgs/accel_with_covariance.pb.h"
#include "openbot/common/proto/geometry_msgs/accel_with_covariance_stamped.pb.h"  
#include "openbot/common/proto/geometry_msgs/inertia.pb.h"  
#include "openbot/common/proto/geometry_msgs/inertia_stamped.pb.h"  
#include "openbot/common/proto/geometry_msgs/point.pb.h"  
#include "openbot/common/proto/geometry_msgs/point32.pb.h"  
#include "openbot/common/proto/geometry_msgs/point_stamped.pb.h"  
#include "openbot/common/proto/geometry_msgs/polygon.pb.h"  
#include "openbot/common/proto/geometry_msgs/polygon_stamped.pb.h"
#include "openbot/common/proto/geometry_msgs/pose.pb.h"  
#include "openbot/common/proto/geometry_msgs/pose_2d.pb.h"  
#include "openbot/common/proto/geometry_msgs/pose_array.pb.h"  
#include "openbot/common/proto/geometry_msgs/pose_stamped.pb.h"
#include "openbot/common/proto/geometry_msgs/pose_with_covariance.pb.h"
#include "openbot/common/proto/geometry_msgs/pose_with_covariance_stamped.pb.h"
#include "openbot/common/proto/geometry_msgs/quaternion.pb.h"  
#include "openbot/common/proto/geometry_msgs/quaternion_stamped.pb.h"
#include "openbot/common/proto/geometry_msgs/transform.pb.h"  
#include "openbot/common/proto/geometry_msgs/transform_stamped.pb.h"
#include "openbot/common/proto/geometry_msgs/twist.pb.h"  
#include "openbot/common/proto/geometry_msgs/twist_stamped.pb.h"  
#include "openbot/common/proto/geometry_msgs/twist_with_covariance.pb.h"
#include "openbot/common/proto/geometry_msgs/twist_with_covariance_stamped.pb.h"
#include "openbot/common/proto/geometry_msgs/vector3.pb.h"  
#include "openbot/common/proto/geometry_msgs/vector3_stamped.pb.h" 
#include "openbot/common/proto/geometry_msgs/velocity_stamped.pb.h" 
#include "openbot/common/proto/builtin_interfaces/time.pb.h" 
#include "openbot/common/proto/builtin_interfaces/duration.pb.h"

// nav_msgs
#include "openbot/common/proto/nav_msgs/grid_cells.pb.h" 
#include "openbot/common/proto/nav_msgs/map_meta_data.pb.h"
#include "openbot/common/proto/nav_msgs/occupancy_grid.pb.h" 
#include "openbot/common/proto/nav_msgs/odometry.pb.h"
#include "openbot/common/proto/nav_msgs/path.pb.h" 

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

// shape_msgs
#include "openbot/common/proto/shape_msgs/mesh.pb.h"
#include "openbot/common/proto/shape_msgs/mesh_triangle.pb.h"
#include "openbot/common/proto/shape_msgs/plane.pb.h"
#include "openbot/common/proto/shape_msgs/solid_primitive.pb.h"

// std_msgs
#include "openbot/common/proto/std_msgs/header.pb.h"

// vision_msgs
#include "openbot/common/proto/vision_msgs/bounding_box_2d.pb.h"
#include "openbot/common/proto/vision_msgs/bounding_box_3d.pb.h"
#include "openbot/common/proto/vision_msgs/bounding_box_2d_array.pb.h"
#include "openbot/common/proto/vision_msgs/bounding_box_3d_array.pb.h"
#include "openbot/common/proto/vision_msgs/classification.pb.h"
#include "openbot/common/proto/vision_msgs/detection_2d.pb.h"
#include "openbot/common/proto/vision_msgs/detection_2d_array.pb.h"
#include "openbot/common/proto/vision_msgs/detection_3d_array.pb.h"
#include "openbot/common/proto/vision_msgs/detection_3d_array.pb.h"
#include "openbot/common/proto/vision_msgs/label_info.pb.h"
#include "openbot/common/proto/vision_msgs/object_hypothesis.pb.h"
#include "openbot/common/proto/vision_msgs/object_hypothesis_with_pose.pb.h"
#include "openbot/common/proto/vision_msgs/point_2d.pb.h"
#include "openbot/common/proto/vision_msgs/pose_2d.pb.h"
#include "openbot/common/proto/vision_msgs/vision_class.pb.h"
#include "openbot/common/proto/vision_msgs/vision_info.pb.h"


namespace openbot_ros {

//------------------------------------------ builtin_interfaces ------------------------------------

::openbot::common::proto::builtin_interfaces::Time FromRos(const rclcpp::Time& ros);

//------------------------------------------ geometry_msgs -----------------------------------------

// Accel
::openbot::common::proto::geometry_msgs::Accel FromRos(const geometry_msgs::msg::Accel& ros);
::openbot::common::proto::geometry_msgs::AccelStamped FromRos(const geometry_msgs::msg::AccelStamped& ros);
::openbot::common::proto::geometry_msgs::AccelWithCovariance FromRos(const geometry_msgs::msg::AccelWithCovariance& ros);
::openbot::common::proto::geometry_msgs::AccelWithCovarianceStamped FromRos(const geometry_msgs::msg::AccelWithCovarianceStamped& ros);

// Inertia
::openbot::common::proto::geometry_msgs::Inertia FromRos(const geometry_msgs::msg::Inertia& ros);
::openbot::common::proto::geometry_msgs::InertiaStamped FromRos(const geometry_msgs::msg::InertiaStamped& ros);

// Point
::openbot::common::proto::geometry_msgs::Point FromRos(const geometry_msgs::msg::Point& ros);
::openbot::common::proto::geometry_msgs::Point32 FromRos(const geometry_msgs::msg::Point32& ros);
::openbot::common::proto::geometry_msgs::PointStamped FromRos(const geometry_msgs::msg::PointStamped& ros);

// polygon
::openbot::common::proto::geometry_msgs::Polygon FromRos(const geometry_msgs::msg::Polygon& ros);
::openbot::common::proto::geometry_msgs::PolygonStamped FromRos(const geometry_msgs::msg::PolygonStamped& ros);

// Pose
::openbot::common::proto::geometry_msgs::Pose FromRos(const geometry_msgs::msg::Pose& ros);
::openbot::common::proto::geometry_msgs::Pose2D FromRos(const geometry_msgs::msg::Pose2D& ros);
::openbot::common::proto::geometry_msgs::PoseArray FromRos(const geometry_msgs::msg::PoseArray& ros);
::openbot::common::proto::geometry_msgs::PoseStamped FromRos(const geometry_msgs::msg::PoseStamped& ros);
::openbot::common::proto::geometry_msgs::PoseWithCovariance FromRos(const geometry_msgs::msg::PoseWithCovariance& ros);
::openbot::common::proto::geometry_msgs::PoseWithCovarianceStamped FromRos(const geometry_msgs::msg::PoseWithCovarianceStamped& ros);

// Quaternion
::openbot::common::proto::geometry_msgs::Quaternion FromRos(const geometry_msgs::msg::Quaternion& ros);
::openbot::common::proto::geometry_msgs::QuaternionStamped FromRos(const geometry_msgs::msg::QuaternionStamped& ros);

// transform
::openbot::common::proto::geometry_msgs::Transform FromRos(const geometry_msgs::msg::Transform& ros);
::openbot::common::proto::geometry_msgs::TransformStamped FromRos(const geometry_msgs::msg::TransformStamped & ros);

// Twist
::openbot::common::proto::geometry_msgs::Twist FromRos(const geometry_msgs::msg::Twist& ros);
::openbot::common::proto::geometry_msgs::TwistStamped FromRos(const geometry_msgs::msg::TwistStamped& ros);
::openbot::common::proto::geometry_msgs::TwistWithCovariance FromRos(const geometry_msgs::msg::TwistWithCovariance& ros);
::openbot::common::proto::geometry_msgs::TwistWithCovarianceStamped FromRos(const geometry_msgs::msg::TwistWithCovarianceStamped& ros);

// Vector3
::openbot::common::proto::geometry_msgs::Vector3 FromRos(const geometry_msgs::msg::Vector3& ros);
::openbot::common::proto::geometry_msgs::Vector3Stamped FromRos(const geometry_msgs::msg::Vector3Stamped& ros);
::openbot::common::proto::geometry_msgs::VelocityStamped FromRos(const geometry_msgs::msg::VelocityStamped& ros);

// 

//------------------------------------------ nav_msgs ----------------------------------------------

// GridCells
::openbot::common::proto::nav_msgs::GridCells FromRos(const nav_msgs::msg::GridCells& ros);

// MapMetaData
 ::openbot::common::proto::nav_msgs::MapMetaData FromRos(const nav_msgs::msg::MapMetaData& ros);

// OccupancyGrid
::openbot::common::proto::nav_msgs::OccupancyGrid FromRos(const nav_msgs::msg::OccupancyGrid& ros);

// Odometry
::openbot::common::proto::nav_msgs::Odometry FromRos(const nav_msgs::msg::Odometry& ros);

// Path
::openbot::common::proto::nav_msgs::Path FromRos(const nav_msgs::msg::Path& ros);

//------------------------------------------ sensor_msgs -------------------------------------------

// CameraInfo
::openbot::common::proto::sensor_msgs::CameraInfo FromRos(const sensor_msgs::msg::CameraInfo& ros);

// ChannelFloat32
::openbot::common::proto::sensor_msgs::ChannelFloat32 FromRos(const sensor_msgs::msg::ChannelFloat32& ros);

// CompressedImage
::openbot::common::proto::sensor_msgs::CompressedImage FromRos(const sensor_msgs::msg::CompressedImage& ros);

// Illuminance
::openbot::common::proto::sensor_msgs::Illuminance FromRos(const sensor_msgs::msg::Illuminance& ros);

// Image
::openbot::common::proto::sensor_msgs::Image FromRos(const sensor_msgs::msg::Image& ros);

// Imu
::openbot::common::proto::sensor_msgs::Imu FromRos(const sensor_msgs::msg::Imu& ros);

// LaserScan
::openbot::common::proto::sensor_msgs::LaserScan FromRos(const sensor_msgs::msg::LaserScan& ros);

// PointCloud
::openbot::common::proto::sensor_msgs::PointCloud FromRos(const sensor_msgs::msg::PointCloud& ros);

// PointCloud2
::openbot::common::proto::sensor_msgs::PointCloud2 FromRos(const sensor_msgs::msg::PointCloud2& ros);

// PointField
::openbot::common::proto::sensor_msgs::PointField FromRos(const sensor_msgs::msg::PointField& ros);

// Range
::openbot::common::proto::sensor_msgs::Range FromRos(const sensor_msgs::msg::Range& ros);

// RegionOfInterest
::openbot::common::proto::sensor_msgs::RegionOfInterest FromRos(const sensor_msgs::msg::RegionOfInterest& ros);

//------------------------------------------ shape_msgs --------------------------------------------

// Mesh
::openbot::common::proto::shape_msgs::Mesh FromRos(const shape_msgs::msg::Mesh& ros);

// MeshTriangle
::openbot::common::proto::shape_msgs::MeshTriangle FromRos(const shape_msgs::msg::MeshTriangle& ros);

// Plane
::openbot::common::proto::shape_msgs::Plane FromRos(const shape_msgs::msg::Plane& ros);

// SolidPrimitive
::openbot::common::proto::shape_msgs::SolidPrimitive FromRos(const shape_msgs::msg::SolidPrimitive& ros);

//------------------------------------------ std_msgs ----------------------------------------------
// Header
::openbot::common::proto::std_msgs::Header FromRos(const std_msgs::msg::Header& ros);

}  // namespace openbot_ros

#endif  // OPENBOT_ROS_OPENBOT_ROS_MSGS_TO_PROTO_HPP