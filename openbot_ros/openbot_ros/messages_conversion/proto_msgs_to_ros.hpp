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



namespace openbot_ros {

// //------------------------------------------ builtin_interfaces -------------------------------------

// // Time
// rclcpp::Time ToRos(const ::openbot::common::proto::builtin_interfaces::Time& proto);

// //------------------------------------------ geometry_msgs ------------------------------------------

// // Accel
// geometry_msgs::msg::Accel ToRos(const ::openbot::common::proto::geometry_msgs::Accel& proto);
// geometry_msgs::msg::AccelStamped ToRos(const ::openbot::common::proto::geometry_msgs::AccelStamped& proto);
// geometry_msgs::msg::AccelWithCovariance ToRos(const ::openbot::common::proto::geometry_msgs::AccelWithCovariance& proto);
// geometry_msgs::msg::AccelWithCovarianceStamped ToRos(const ::openbot::common::proto::geometry_msgs::AccelWithCovarianceStamped& proto);

// // Inertia
// geometry_msgs::msg::Inertia ToRos(const ::openbot::common::proto::geometry_msgs::Inertia& proto);
// geometry_msgs::msg::InertiaStamped ToRos(const ::openbot::common::proto::geometry_msgs::InertiaStamped& proto);

// // Point
// geometry_msgs::msg::Point ToRos(const ::openbot::common::proto::geometry_msgs::Point& proto);
// geometry_msgs::msg::Point32 ToRos(const ::openbot::common::proto::geometry_msgs::Point32& proto);
// geometry_msgs::msg::PointStamped ToRos(const ::openbot::common::proto::geometry_msgs::PointStamped& proto);

// // polygon
// geometry_msgs::msg::Polygon ToRos(const ::openbot::common::proto::geometry_msgs::Polygon& proto);
// geometry_msgs::msg::PolygonStamped ToRos(const ::openbot::common::proto::geometry_msgs::PolygonStamped& proto);

// // Pose
// geometry_msgs::msg::Pose ToRos(const ::openbot::common::proto::geometry_msgs::Pose& proto);
// geometry_msgs::msg::Pose2D ToRos(const ::openbot::common::proto::geometry_msgs::Pose2D& proto);
// geometry_msgs::msg::PoseArray ToRos(const ::openbot::common::proto::geometry_msgs::PoseArray& proto);
// geometry_msgs::msg::PoseStamped ToRos(const ::openbot::common::proto::geometry_msgs::PoseStamped& proto);
// geometry_msgs::msg::PoseWithCovariance ToRos(const ::openbot::common::proto::geometry_msgs::PoseWithCovariance& proto);
// geometry_msgs::msg::PoseWithCovarianceStamped ToRos(const ::openbot::common::proto::geometry_msgs::PoseWithCovarianceStamped& proto);

// // Quaternion
// geometry_msgs::msg::Quaternion ToRos(const ::openbot::common::proto::geometry_msgs::Quaternion& proto);
// geometry_msgs::msg::QuaternionStamped ToRos(const ::openbot::common::proto::geometry_msgs::QuaternionStamped& proto);

// // transform
// geometry_msgs::msg::Transform ToRos(const ::openbot::common::proto::geometry_msgs::Transform& proto);
// geometry_msgs::msg::TransformStamped ToRos(const ::openbot::common::proto::geometry_msgs::TransformStamped& proto);

// // Twist
// geometry_msgs::msg::Twist ToRos(const ::openbot::common::proto::geometry_msgs::Twist& proto);
// geometry_msgs::msg::TwistStamped ToRos(const ::openbot::common::proto::geometry_msgs::TwistStamped& proto);
// geometry_msgs::msg::TwistWithCovariance ToRos(const ::openbot::common::proto::geometry_msgs::TwistWithCovariance& proto);
// geometry_msgs::msg::TwistWithCovarianceStamped ToRos(const ::openbot::common::proto::geometry_msgs::TwistWithCovarianceStamped& proto);

// // Vector3
// geometry_msgs::msg::Vector3 ToRos(const ::openbot::common::proto::geometry_msgs::Vector3& proto);
// geometry_msgs::msg::Vector3Stamped ToRos(const ::openbot::common::proto::geometry_msgs::Vector3Stamped& proto);
// geometry_msgs::msg::VelocityStamped ToRos(const ::openbot::common::proto::geometry_msgs::VelocityStamped& proto);

// //------------------------------------------ nav_msgs ------------------------------------------

// // GridCells
// nav_msgs::msg::GridCells ToRos(const ::openbot::common::proto::nav_msgs::GridCells& proto);

// // MapMetaData
// nav_msgs::msg::MapMetaData ToRos(const ::openbot::common::proto::nav_msgs::MapMetaData& proto);

// // OccupancyGrid
// nav_msgs::msg::OccupancyGrid ToRos(const ::openbot::common::proto::nav_msgs::OccupancyGrid& proto);

// // Odometry
// nav_msgs::msg::Odometry ToRos(const ::openbot::common::proto::nav_msgs::Odometry& proto);

// // Path
// nav_msgs::msg::Path ToRos(const ::openbot::common::proto::nav_msgs::Path& proto);


// //------------------------------------------ sensor_msgs ------------------------------------------

// // CameraInfo
// sensor_msgs::msg::CameraInfo ToRos(const ::openbot::common::proto::sensor_msgs::CameraInfo& proto);

// // ChannelFloat32
// sensor_msgs::msg::ChannelFloat32 ToRos(const ::openbot::common::proto::sensor_msgs::ChannelFloat32& proto);

// // CompressedImage
// sensor_msgs::msg::CompressedImage ToRos(const ::openbot::common::proto::sensor_msgs::CompressedImage& proto);

// // Illuminance
// sensor_msgs::msg::Illuminance ToRos(const ::openbot::common::proto::sensor_msgs::Illuminance& proto);

// // Image
// sensor_msgs::msg::Image ToRos(const ::openbot::common::proto::sensor_msgs::Image& proto);

// // Imu
// sensor_msgs::msg::Imu ToRos(const ::openbot::common::proto::sensor_msgs::Imu& proto);

// // LaserScan
// sensor_msgs::msg::LaserScan ToRos(const ::openbot::common::proto::sensor_msgs::LaserScan& proto);

// // PointCloud
// sensor_msgs::msg::PointCloud ToRos(const ::openbot::common::proto::sensor_msgs::PointCloud& proto);

// // PointCloud2
// sensor_msgs::msg::PointCloud2 ToRos(const ::openbot::common::proto::sensor_msgs::PointCloud2& proto);

// // PointField
// sensor_msgs::msg::PointField ToRos(const ::openbot::common::proto::sensor_msgs::PointField& proto);

// // Range
// sensor_msgs::msg::Range ToRos(const ::openbot::common::proto::sensor_msgs::Range& proto);

// // RegionOfInterest
// sensor_msgs::msg::RegionOfInterest ToRos(const ::openbot::common::proto::sensor_msgs::RegionOfInterest& proto);

// //------------------------------------------ shape_msgs ------------------------------------------

// // Mesh
// shape_msgs::msg::Mesh ToRos(const ::openbot::common::proto::shape_msgs::Mesh& proto);

// // MeshTriangle
// shape_msgs::msg::MeshTriangle ToRos(const ::openbot::common::proto::shape_msgs::MeshTriangle& proto);

// // Plane
// shape_msgs::msg::Plane ToRos(const ::openbot::common::proto::shape_msgs::Plane& proto);

// // SolidPrimitive
// shape_msgs::msg::SolidPrimitive ToRos(const ::openbot::common::proto::shape_msgs::SolidPrimitive& proto);


// //------------------------------------------ std_msgs --------------------------------------------

// // Header
// std_msgs::msg::Header ToRos(const ::openbot::common::proto::std_msgs::Header& proto);


// //------------------------------------------ vision_msgs --------------------------------------------


}  // namespace openbot_ros

#endif  // OPENBOT_ROS_OPENBOT_PROTO_MSGS_TO_ROS_HPP