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


// common msgs proto
#include "openbot_bridge/ros2_msgs/builtin_interfaces.pb.h"
#include "openbot_bridge/ros2_msgs/diagnostic_msgs.pb.h"
#include "openbot_bridge/ros2_msgs/geometry_msgs.pb.h"
#include "openbot_bridge/ros2_msgs/nav_msgs.pb.h"
#include "openbot_bridge/ros2_msgs/pcl_msgs.pb.h"
#include "openbot_bridge/ros2_msgs/sensor_msgs.pb.h"
#include "openbot_bridge/ros2_msgs/shape_msgs.pb.h"
#include "openbot_bridge/ros2_msgs/std_msgs.pb.h"
#include "openbot_bridge/ros2_msgs/trajectory_msgs.pb.h"
#include "openbot_bridge/ros2_msgs/vision_msgs.pb.h"
#include "openbot_bridge/ros2_msgs/visualization_msgs.pb.h"

namespace openbot_ros {

//------------------------------------------ builtin_interfaces ------------------------------------

::openbot_bridge::ros2_msgs::builtin_interfaces::Time ToProto(const rclcpp::Time& ros);

//------------------------------------------ geometry_msgs -----------------------------------------

::openbot_bridge::ros2_msgs::geometry_msgs::Accel ToProto(const geometry_msgs::msg::Accel& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::AccelStamped ToProto(const geometry_msgs::msg::AccelStamped& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::AccelWithCovariance ToProto(const geometry_msgs::msg::AccelWithCovariance& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::AccelWithCovarianceStamped ToProto(const geometry_msgs::msg::AccelWithCovarianceStamped& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::Inertia ToProto(const geometry_msgs::msg::Inertia& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::InertiaStamped ToProto(const geometry_msgs::msg::InertiaStamped& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::Point ToProto(const geometry_msgs::msg::Point& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::Point32 ToProto(const geometry_msgs::msg::Point32& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::PointStamped ToProto(const geometry_msgs::msg::PointStamped& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::Polygon ToProto(const geometry_msgs::msg::Polygon& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::PolygonStamped ToProto(const geometry_msgs::msg::PolygonStamped& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::Pose ToProto(const geometry_msgs::msg::Pose& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::Pose2D ToProto(const geometry_msgs::msg::Pose2D& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::PoseArray ToProto(const geometry_msgs::msg::PoseArray& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::PoseStamped ToProto(const geometry_msgs::msg::PoseStamped& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::PoseWithCovariance ToProto(const geometry_msgs::msg::PoseWithCovariance& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::PoseWithCovarianceStamped ToProto(const geometry_msgs::msg::PoseWithCovarianceStamped& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::Quaternion ToProto(const geometry_msgs::msg::Quaternion& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::QuaternionStamped ToProto(const geometry_msgs::msg::QuaternionStamped& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::Transform ToProto(const geometry_msgs::msg::Transform& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::TransformStamped ToProto(const geometry_msgs::msg::TransformStamped & ros);
::openbot_bridge::ros2_msgs::geometry_msgs::Twist ToProto(const geometry_msgs::msg::Twist& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::TwistStamped ToProto(const geometry_msgs::msg::TwistStamped& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::TwistWithCovariance ToProto(const geometry_msgs::msg::TwistWithCovariance& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::TwistWithCovarianceStamped ToProto(const geometry_msgs::msg::TwistWithCovarianceStamped& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::Vector3 ToProto(const geometry_msgs::msg::Vector3& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::Vector3Stamped ToProto(const geometry_msgs::msg::Vector3Stamped& ros);
::openbot_bridge::ros2_msgs::geometry_msgs::VelocityStamped ToProto(const geometry_msgs::msg::VelocityStamped& ros);

//------------------------------------------ nav_msgs ----------------------------------------------

::openbot_bridge::ros2_msgs::nav_msgs::GridCells ToProto(const nav_msgs::msg::GridCells& ros);
::openbot_bridge::ros2_msgs::nav_msgs::MapMetaData ToProto(const nav_msgs::msg::MapMetaData& ros);
::openbot_bridge::ros2_msgs::nav_msgs::OccupancyGrid ToProto(const nav_msgs::msg::OccupancyGrid& ros);
::openbot_bridge::ros2_msgs::nav_msgs::Odometry ToProto(const nav_msgs::msg::Odometry& ros);
::openbot_bridge::ros2_msgs::nav_msgs::Path ToProto(const nav_msgs::msg::Path& ros);

//------------------------------------------ sensor_msgs -------------------------------------------

::openbot_bridge::ros2_msgs::sensor_msgs::CameraInfo ToProto(const sensor_msgs::msg::CameraInfo& ros);
::openbot_bridge::ros2_msgs::sensor_msgs::ChannelFloat32 ToProto(const sensor_msgs::msg::ChannelFloat32& ros);
::openbot_bridge::ros2_msgs::sensor_msgs::CompressedImage ToProto(const sensor_msgs::msg::CompressedImage& ros);
::openbot_bridge::ros2_msgs::sensor_msgs::Illuminance ToProto(const sensor_msgs::msg::Illuminance& ros);
::openbot_bridge::ros2_msgs::sensor_msgs::Image ToProto(const sensor_msgs::msg::Image& ros);
::openbot_bridge::ros2_msgs::sensor_msgs::Imu ToProto(const sensor_msgs::msg::Imu& ros);
::openbot_bridge::ros2_msgs::sensor_msgs::LaserScan ToProto(const sensor_msgs::msg::LaserScan& ros);
::openbot_bridge::ros2_msgs::sensor_msgs::PointCloud ToProto(const sensor_msgs::msg::PointCloud& ros);
::openbot_bridge::ros2_msgs::sensor_msgs::PointCloud2 ToProto(const sensor_msgs::msg::PointCloud2& ros);
::openbot_bridge::ros2_msgs::sensor_msgs::PointField ToProto(const sensor_msgs::msg::PointField& ros);
::openbot_bridge::ros2_msgs::sensor_msgs::Range ToProto(const sensor_msgs::msg::Range& ros);
::openbot_bridge::ros2_msgs::sensor_msgs::RegionOfInterest ToProto(const sensor_msgs::msg::RegionOfInterest& ros);

//------------------------------------------ shape_msgs --------------------------------------------

::openbot_bridge::ros2_msgs::shape_msgs::Mesh ToProto(const shape_msgs::msg::Mesh& ros);
::openbot_bridge::ros2_msgs::shape_msgs::MeshTriangle ToProto(const shape_msgs::msg::MeshTriangle& ros);
::openbot_bridge::ros2_msgs::shape_msgs::Plane ToProto(const shape_msgs::msg::Plane& ros);
::openbot_bridge::ros2_msgs::shape_msgs::SolidPrimitive ToProto(const shape_msgs::msg::SolidPrimitive& ros);

//------------------------------------------ std_msgs ----------------------------------------------

::openbot_bridge::ros2_msgs::std_msgs::Header ToProto(const std_msgs::msg::Header& ros);

}  // namespace openbot_ros

#endif  // OPENBOT_ROS_OPENBOT_ROS_MSGS_TO_PROTO_HPP