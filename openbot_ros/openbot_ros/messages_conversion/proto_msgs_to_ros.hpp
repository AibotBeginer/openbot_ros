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


// common msgs proto
#include "ros2_msgs/builtin_interfaces.pb.h"
#include "ros2_msgs/diagnostic_msgs.pb.h"
#include "ros2_msgs/geometry_msgs.pb.h"
#include "ros2_msgs/nav_msgs.pb.h"
#include "ros2_msgs/pcl_msgs.pb.h"
#include "ros2_msgs/sensor_msgs.pb.h"
#include "ros2_msgs/shape_msgs.pb.h"
#include "ros2_msgs/std_msgs.pb.h"
#include "ros2_msgs/trajectory_msgs.pb.h"
#include "ros2_msgs/vision_msgs.pb.h"
#include "ros2_msgs/visualization_msgs.pb.h"

namespace openbot_ros {

//------------------------------------------ builtin_interfaces -------------------------------------

// Time
rclcpp::Time ToRos(const ::openbot::ros2_msgs::builtin_interfaces::Time& proto);

//------------------------------------------ geometry_msgs ------------------------------------------

// Accel
geometry_msgs::msg::Accel ToRos(const ::openbot::ros2_msgs::geometry_msgs::Accel& proto);
geometry_msgs::msg::AccelStamped ToRos(const ::openbot::ros2_msgs::geometry_msgs::AccelStamped& proto);
geometry_msgs::msg::AccelWithCovariance ToRos(const ::openbot::ros2_msgs::geometry_msgs::AccelWithCovariance& proto);
geometry_msgs::msg::AccelWithCovarianceStamped ToRos(const ::openbot::ros2_msgs::geometry_msgs::AccelWithCovarianceStamped& proto);
geometry_msgs::msg::Inertia ToRos(const ::openbot::ros2_msgs::geometry_msgs::Inertia& proto);
geometry_msgs::msg::InertiaStamped ToRos(const ::openbot::ros2_msgs::geometry_msgs::InertiaStamped& proto);
geometry_msgs::msg::Point ToRos(const ::openbot::ros2_msgs::geometry_msgs::Point& proto);
geometry_msgs::msg::Point32 ToRos(const ::openbot::ros2_msgs::geometry_msgs::Point32& proto);
geometry_msgs::msg::PointStamped ToRos(const ::openbot::ros2_msgs::geometry_msgs::PointStamped& proto);
geometry_msgs::msg::Polygon ToRos(const ::openbot::ros2_msgs::geometry_msgs::Polygon& proto);
geometry_msgs::msg::PolygonStamped ToRos(const ::openbot::ros2_msgs::geometry_msgs::PolygonStamped& proto);
geometry_msgs::msg::Pose ToRos(const ::openbot::ros2_msgs::geometry_msgs::Pose& proto);
geometry_msgs::msg::Pose2D ToRos(const ::openbot::ros2_msgs::geometry_msgs::Pose2D& proto);
geometry_msgs::msg::PoseArray ToRos(const ::openbot::ros2_msgs::geometry_msgs::PoseArray& proto);
geometry_msgs::msg::PoseStamped ToRos(const ::openbot::ros2_msgs::geometry_msgs::PoseStamped& proto);
geometry_msgs::msg::PoseWithCovariance ToRos(const ::openbot::ros2_msgs::geometry_msgs::PoseWithCovariance& proto);
geometry_msgs::msg::PoseWithCovarianceStamped ToRos(const ::openbot::ros2_msgs::geometry_msgs::PoseWithCovarianceStamped& proto);
geometry_msgs::msg::Quaternion ToRos(const ::openbot::ros2_msgs::geometry_msgs::Quaternion& proto);
geometry_msgs::msg::QuaternionStamped ToRos(const ::openbot::ros2_msgs::geometry_msgs::QuaternionStamped& proto);
geometry_msgs::msg::Transform ToRos(const ::openbot::ros2_msgs::geometry_msgs::Transform& proto);
geometry_msgs::msg::TransformStamped ToRos(const ::openbot::ros2_msgs::geometry_msgs::TransformStamped& proto);
geometry_msgs::msg::Twist ToRos(const ::openbot::ros2_msgs::geometry_msgs::Twist& proto);
geometry_msgs::msg::TwistStamped ToRos(const ::openbot::ros2_msgs::geometry_msgs::TwistStamped& proto);
geometry_msgs::msg::TwistWithCovariance ToRos(const ::openbot::ros2_msgs::geometry_msgs::TwistWithCovariance& proto);
geometry_msgs::msg::TwistWithCovarianceStamped ToRos(const ::openbot::ros2_msgs::geometry_msgs::TwistWithCovarianceStamped& proto);
geometry_msgs::msg::Vector3 ToRos(const ::openbot::ros2_msgs::geometry_msgs::Vector3& proto);
geometry_msgs::msg::Vector3Stamped ToRos(const ::openbot::ros2_msgs::geometry_msgs::Vector3Stamped& proto);
geometry_msgs::msg::VelocityStamped ToRos(const ::openbot::ros2_msgs::geometry_msgs::VelocityStamped& proto);

//------------------------------------------ nav_msgs ------------------------------------------

nav_msgs::msg::GridCells ToRos(const ::openbot::ros2_msgs::nav_msgs::GridCells& proto);
nav_msgs::msg::MapMetaData ToRos(const ::openbot::ros2_msgs::nav_msgs::MapMetaData& proto);
nav_msgs::msg::OccupancyGrid ToRos(const ::openbot::ros2_msgs::nav_msgs::OccupancyGrid& proto);
nav_msgs::msg::Odometry ToRos(const ::openbot::ros2_msgs::nav_msgs::Odometry& proto);
nav_msgs::msg::Path ToRos(const ::openbot::ros2_msgs::nav_msgs::Path& proto);


//------------------------------------------ sensor_msgs ------------------------------------------

sensor_msgs::msg::CameraInfo ToRos(const ::openbot::ros2_msgs::sensor_msgs::CameraInfo& proto);
sensor_msgs::msg::ChannelFloat32 ToRos(const ::openbot::ros2_msgs::sensor_msgs::ChannelFloat32& proto);
sensor_msgs::msg::CompressedImage ToRos(const ::openbot::ros2_msgs::sensor_msgs::CompressedImage& proto);
sensor_msgs::msg::Illuminance ToRos(const ::openbot::ros2_msgs::sensor_msgs::Illuminance& proto);
sensor_msgs::msg::Image ToRos(const ::openbot::ros2_msgs::sensor_msgs::Image& proto);
sensor_msgs::msg::Imu ToRos(const ::openbot::ros2_msgs::sensor_msgs::Imu& proto);
sensor_msgs::msg::LaserScan ToRos(const ::openbot::ros2_msgs::sensor_msgs::LaserScan& proto);
sensor_msgs::msg::PointCloud ToRos(const ::openbot::ros2_msgs::sensor_msgs::PointCloud& proto);
sensor_msgs::msg::PointCloud2 ToRos(const ::openbot::ros2_msgs::sensor_msgs::PointCloud2& proto);
sensor_msgs::msg::PointField ToRos(const ::openbot::ros2_msgs::sensor_msgs::PointField& proto);
sensor_msgs::msg::Range ToRos(const ::openbot::ros2_msgs::sensor_msgs::Range& proto);
sensor_msgs::msg::RegionOfInterest ToRos(const ::openbot::ros2_msgs::sensor_msgs::RegionOfInterest& proto);

//------------------------------------------ shape_msgs ------------------------------------------


shape_msgs::msg::Mesh ToRos(const ::openbot::ros2_msgs::shape_msgs::Mesh& proto);
shape_msgs::msg::MeshTriangle ToRos(const ::openbot::ros2_msgs::shape_msgs::MeshTriangle& proto);
shape_msgs::msg::Plane ToRos(const ::openbot::ros2_msgs::shape_msgs::Plane& proto);
shape_msgs::msg::SolidPrimitive ToRos(const ::openbot::ros2_msgs::shape_msgs::SolidPrimitive& proto);


//------------------------------------------ std_msgs --------------------------------------------

std_msgs::msg::Header ToRos(const ::openbot::ros2_msgs::std_msgs::Header& proto);


//------------------------------------------ vision_msgs --------------------------------------------


}  // namespace openbot_ros

#endif  // OPENBOT_ROS_OPENBOT_PROTO_MSGS_TO_ROS_HPP