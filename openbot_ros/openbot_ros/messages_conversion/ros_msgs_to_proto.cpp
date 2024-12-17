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

#include "ros_msgs_to_proto.hpp"

namespace openbot_ros {

::openbot::ros2_msgs::builtin_interfaces::Time ToProto(const rclcpp::Time& ros)
{
    ::openbot::ros2_msgs::builtin_interfaces::Time proto;
    proto.set_seconds(ros.seconds());
    proto.set_nanoseconds(ros.seconds());
    return proto;
}

::openbot::ros2_msgs::std_msgs::Header ToProto(const std_msgs::msg::Header& ros)
{
    ::openbot::ros2_msgs::std_msgs::Header proto;
    *proto.mutable_stamp() = ToProto(ros.stamp);
    proto.set_frame_id(ros.frame_id);
    return proto;
    
}

::openbot::ros2_msgs::geometry_msgs::Accel ToProto(const geometry_msgs::msg::Accel& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::Accel proto;
    *proto.mutable_linear() = ToProto(ros.linear);
    *proto.mutable_angular() = ToProto(ros.angular);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::AccelStamped ToProto(
    const geometry_msgs::msg::AccelStamped& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::AccelStamped proto;
    *proto.mutable_header() = ToProto(ros.header);
    *proto.mutable_accel() = ToProto(ros.accel);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::AccelWithCovariance ToProto(
    const geometry_msgs::msg::AccelWithCovariance& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::AccelWithCovariance proto;
    *proto.mutable_accel() = ToProto(ros.accel);
    for (int i = 0; i < ros.covariance.size(); ++i) {
        proto.set_covariance(i, ros.covariance[i]);
    }
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::AccelWithCovarianceStamped ToProto(
    const geometry_msgs::msg::AccelWithCovarianceStamped& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::AccelWithCovarianceStamped proto;
    *proto.mutable_header() = ToProto(ros.header);
    *proto.mutable_accel() = ToProto(ros.accel);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::Point ToProto(const geometry_msgs::msg::Point& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::Point proto;
    proto.set_x(ros.x);
    proto.set_y(ros.y);
    proto.set_z(ros.z);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::Point32 ToProto(const geometry_msgs::msg::Point32& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::Point32 proto;
    proto.set_x(ros.x);
    proto.set_y(ros.y);
    proto.set_z(ros.z);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::PointStamped ToProto(const geometry_msgs::msg::PointStamped& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::PointStamped proto;
    *proto.mutable_header() = ToProto(ros.header);
    *proto.mutable_point() = ToProto(ros.point);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::Polygon ToProto(const geometry_msgs::msg::Polygon& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::Polygon proto;
    for (int i = 0; i < ros.points.size(); ++i) {
        *proto.mutable_points(i) = ToProto(ros.points[i]);
    }
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::PolygonStamped ToProto(const geometry_msgs::msg::PolygonStamped& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::PolygonStamped proto;
    *proto.mutable_header() = ToProto(ros.header);
    *proto.mutable_polygon() = ToProto(ros.polygon);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::Pose ToProto(const geometry_msgs::msg::Pose& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::Pose proto;
    *proto.mutable_position() = ToProto(ros.position);
    *proto.mutable_orientation() = ToProto(ros.orientation);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::Pose2D ToProto(const geometry_msgs::msg::Pose2D& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::Pose2D proto;
    proto.set_x(ros.x);
    proto.set_y(ros.y);
    proto.set_theta(ros.theta);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::PoseArray ToProto(const geometry_msgs::msg::PoseArray& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::PoseArray proto;
    *proto.mutable_header() = ToProto(ros.header);
    for (int i = 0; i < ros.poses.size(); ++i) {
        *proto.mutable_poses(i) = ToProto(ros.poses[i]);
    }
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::PoseStamped ToProto(const geometry_msgs::msg::PoseStamped& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::PoseStamped proto;
    *proto.mutable_header() = ToProto(ros.header);
    *proto.mutable_pose() = ToProto(ros.pose);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::PoseWithCovariance ToProto(const geometry_msgs::msg::PoseWithCovariance& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::PoseWithCovariance proto;
    *proto.mutable_pose() = ToProto(ros.pose);
    for (int i = 0; i < ros.covariance.size(); ++i) {
        proto.set_covariance(i, ros.covariance[i]);
    }
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::PoseWithCovarianceStamped ToProto(const geometry_msgs::msg::PoseWithCovarianceStamped& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::PoseWithCovarianceStamped proto;
    *proto.mutable_header() = ToProto(ros.header);
    *proto.mutable_pose() = ToProto(ros.pose);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::Quaternion ToProto(const geometry_msgs::msg::Quaternion& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::Quaternion proto;
    proto.set_x(ros.x);
    proto.set_y(ros.y);
    proto.set_z(ros.z);
    proto.set_w(ros.w);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::QuaternionStamped ToProto(const geometry_msgs::msg::QuaternionStamped& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::QuaternionStamped proto;
    *proto.mutable_header() = ToProto(ros.header);
    *proto.mutable_quaternion() = ToProto(ros.quaternion);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::Transform ToProto(const geometry_msgs::msg::Transform& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::Transform proto;
    *proto.mutable_translation() = ToProto(ros.translation);
    *proto.mutable_rotation() = ToProto(ros.rotation);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::TransformStamped ToProto(const geometry_msgs::msg::TransformStamped& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::TransformStamped proto;
    *proto.mutable_header() = ToProto(ros.header);
    proto.set_child_frame_id(ros.child_frame_id);
    *proto.mutable_transform() = ToProto(ros.transform);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::Twist ToProto(const geometry_msgs::msg::Twist& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::Twist proto;
    *proto.mutable_linear() = ToProto(ros.linear);
    *proto.mutable_angular() = ToProto(ros.angular);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::TwistStamped ToProto(const geometry_msgs::msg::TwistStamped& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::TwistStamped proto;
    *proto.mutable_header() = ToProto(ros.header);
    *proto.mutable_twist() = ToProto(ros.twist);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::TwistWithCovariance ToProto(const geometry_msgs::msg::TwistWithCovariance& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::TwistWithCovariance proto;
    *proto.mutable_twist() = ToProto(ros.twist);
    for (int i = 0; i < ros.covariance.size(); ++i) {
        proto.set_covariance(i, ros.covariance[i]);
    }
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::TwistWithCovarianceStamped ToProto(
    const geometry_msgs::msg::TwistWithCovarianceStamped& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::TwistWithCovarianceStamped proto;
    *proto.mutable_header() = ToProto(ros.header);
    *proto.mutable_twist() = ToProto(ros.twist);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::Vector3 ToProto(const geometry_msgs::msg::Vector3& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::Vector3 proto;
    proto.set_x(ros.x);
    proto.set_y(ros.y);
    proto.set_z(ros.z);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::Vector3Stamped ToProto(const geometry_msgs::msg::Vector3Stamped& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::Vector3Stamped proto;
    *proto.mutable_header() = ToProto(ros.header);
    *proto.mutable_vector() = ToProto(ros.vector);
    return proto;
}

::openbot::ros2_msgs::geometry_msgs::VelocityStamped ToProto(const geometry_msgs::msg::VelocityStamped& ros)
{
    ::openbot::ros2_msgs::geometry_msgs::VelocityStamped proto;
    *proto.mutable_header() = ToProto(ros.header);
    proto.set_body_frame_id(ros.body_frame_id);
    proto.set_reference_frame_id(ros.reference_frame_id);
    *proto.mutable_velocity() = ToProto(ros.velocity);
    return proto;
}

::openbot::ros2_msgs::nav_msgs::GridCells ToProto(const nav_msgs::msg::GridCells& ros)
{
    ::openbot::ros2_msgs::nav_msgs::GridCells proto;
    *proto.mutable_header() = ToProto(ros.header);
    proto.set_cell_width(ros.cell_width);
    proto.set_cell_height(ros.cell_height);
    for (int i = 0; i < ros.cells.size(); ++i) {
        *proto.mutable_cells(i) = ToProto(ros.cells[i]);
    }
    return proto;
}

 ::openbot::ros2_msgs::nav_msgs::MapMetaData ToProto(const nav_msgs::msg::MapMetaData& ros)
{
    ::openbot::ros2_msgs::nav_msgs::MapMetaData proto;
    *proto.mutable_map_load_time() = ToProto(ros.map_load_time);
    proto.set_resolution(ros.resolution);
    proto.set_width(ros.width);
    proto.set_height(ros.height);
    *proto.mutable_origin() = ToProto(ros.origin);
    return proto;
}

::openbot::ros2_msgs::nav_msgs::OccupancyGrid ToProto(const nav_msgs::msg::OccupancyGrid& ros)
{
    ::openbot::ros2_msgs::nav_msgs::OccupancyGrid proto;
    *proto.mutable_header() = ToProto(ros.header);
    *proto.mutable_info() = ToProto(ros.info);
    for (int i = 0; i < ros.data.size(); ++i) {
        proto.set_data(i, ros.data[i]);
    }
    return proto;
}

::openbot::ros2_msgs::nav_msgs::Odometry ToProto(const nav_msgs::msg::Odometry& ros)
{
    ::openbot::ros2_msgs::nav_msgs::Odometry proto;
    *proto.mutable_header() = ToProto(ros.header);
    proto.set_child_frame_id(ros.child_frame_id);
    *proto.mutable_pose() = ToProto(ros.pose);
    *proto.mutable_twist() = ToProto(ros.twist);
    return proto;
}

::openbot::ros2_msgs::nav_msgs::Path ToProto(const nav_msgs::msg::Path& ros)
{
    ::openbot::ros2_msgs::nav_msgs::Path proto;
    *proto.mutable_header() = ToProto(ros.header);
    for (int i = 0; i < ros.poses.size(); ++i) {
        *proto.mutable_poses(i) = ToProto(ros.poses[i]);
    }
    return proto;
}

::openbot::ros2_msgs::sensor_msgs::CameraInfo ToProto(const sensor_msgs::msg::CameraInfo& ros)
{
    ::openbot::ros2_msgs::sensor_msgs::CameraInfo proto;
    *proto.mutable_header() = ToProto(ros.header);
    proto.set_width(ros.width);
    proto.set_height(ros.height);
    proto.set_distortion_model(ros.distortion_model);
    for (int i = 0; i < ros.d.size(); ++i) {
        proto.set_d(i, ros.d[i]);
    }

    for (int i = 0; i < ros.k.size(); ++i) {
        proto.set_k(i, ros.k[i]);
    }

    for (int i = 0; i < ros.r.size(); ++i) {
        proto.set_r(i, ros.r[i]);
    }

    for (int i = 0; i < ros.p.size(); ++i) {
        proto.set_p(i, ros.p[i]);
    }
    proto.set_binning_x(ros.binning_x);
    proto.set_binning_y(ros.binning_y);
    *proto.mutable_roi() = ToProto(ros.roi);
    return proto;
}

::openbot::ros2_msgs::sensor_msgs::ChannelFloat32 ToProto(const sensor_msgs::msg::ChannelFloat32& ros)
{
    ::openbot::ros2_msgs::sensor_msgs::ChannelFloat32 proto;
    proto.set_name(ros.name);
    for (int i = 0; i < ros.values.size(); ++i) {
        proto.set_values(i, ros.values[i]);
    }
    return proto;
}

::openbot::ros2_msgs::sensor_msgs::CompressedImage ToProto(const sensor_msgs::msg::CompressedImage& ros)
{
    ::openbot::ros2_msgs::sensor_msgs::CompressedImage proto;
    *proto.mutable_header() = ToProto(ros.header);
    proto.set_format(ros.format);
    for (int i = 0; i < ros.data.size(); ++i) {
        proto.set_data(i, ros.data[i]);
    }
    return proto;
}

::openbot::ros2_msgs::sensor_msgs::Illuminance ToProto(const sensor_msgs::msg::Illuminance& ros)
{
    ::openbot::ros2_msgs::sensor_msgs::Illuminance proto;
    *proto.mutable_header() = ToProto(ros.header);
    proto.set_illuminance(ros.illuminance);
    proto.set_variance(ros.variance);
    return proto;
}

::openbot::ros2_msgs::sensor_msgs::Image ToProto(const sensor_msgs::msg::Image& ros)
{
    ::openbot::ros2_msgs::sensor_msgs::Image proto;
    *proto.mutable_header() = ToProto(ros.header);
    proto.set_width(ros.width);
    proto.set_height(ros.height);
    proto.set_encoding(ros.encoding);
    proto.set_is_bigendian(ros.is_bigendian);
    proto.set_step(ros.step);
    for (int i = 0; i < ros.data.size(); ++i) {
        proto.set_data(i, ros.data[i]);
    }
    return proto;
}

::openbot::ros2_msgs::sensor_msgs::Imu ToProto(const sensor_msgs::msg::Imu& ros)
{
    ::openbot::ros2_msgs::sensor_msgs::Imu proto;
    *proto.mutable_header() = ToProto(ros.header);
    *proto.mutable_orientation() = ToProto(ros.orientation);
    for (int i = 0; i < ros.orientation_covariance.size(); ++i) {
        proto.set_orientation_covariance(i, ros.orientation_covariance[i]);
    }
    *proto.mutable_angular_velocity() = ToProto(ros.angular_velocity);
    for (int i = 0; i < ros.angular_velocity_covariance.size(); ++i) {
        proto.set_angular_velocity_covariance(i, ros.angular_velocity_covariance[i]);
    }
    *proto.mutable_linear_acceleration() = ToProto(ros.linear_acceleration);
    for (int i = 0; i < ros.linear_acceleration_covariance.size(); ++i) {
        proto.set_linear_acceleration_covariance(i, ros.linear_acceleration_covariance[i]);
    }
    return proto;
}

::openbot::ros2_msgs::sensor_msgs::LaserScan ToProto(const sensor_msgs::msg::LaserScan& ros)
{
    ::openbot::ros2_msgs::sensor_msgs::LaserScan proto;
    *proto.mutable_header() = ToProto(ros.header);
    proto.set_angle_min(ros.angle_min);
    proto.set_angle_max(ros.angle_max);
    proto.set_angle_increment(ros.angle_increment);
    proto.set_time_increment(ros.time_increment);
    proto.set_scan_time(ros.scan_time);
    proto.set_range_min(ros.range_min);
    proto.set_range_max(ros.range_max);
    return proto;
}

::openbot::ros2_msgs::sensor_msgs::PointCloud ToProto(const sensor_msgs::msg::PointCloud& ros)
{
    ::openbot::ros2_msgs::sensor_msgs::PointCloud proto;
    *proto.mutable_header() = ToProto(ros.header);
    for (int i = 0; i < ros.points.size(); ++i) {
        *proto.mutable_points(i) = ToProto(ros.points[i]);
    }
    for (int i = 0; i < ros.channels.size(); ++i) {
        *proto.mutable_channels(i) = ToProto(ros.channels[i]);
    }
    return proto;
}

::openbot::ros2_msgs::sensor_msgs::PointCloud2 ToProto(const sensor_msgs::msg::PointCloud2& ros)
{
    ::openbot::ros2_msgs::sensor_msgs::PointCloud2 proto;
    *proto.mutable_header() = ToProto(ros.header);
    proto.set_width(ros.width);
    proto.set_height(ros.height);
    for (int i = 0; i < ros.fields.size(); ++i) {
        *proto.mutable_fields(i) = ToProto(ros.fields[i]);
    }
    proto.set_is_bigendian(ros.is_bigendian);
    proto.set_point_step(ros.point_step);
    proto.set_row_step(ros.row_step);
    for (int i = 0; i < ros.data.size(); ++i) {
        proto.set_data(i, ros.data[i]);
    }
    proto.set_is_dense(ros.is_dense);
    return proto;
}

::openbot::ros2_msgs::sensor_msgs::PointField ToProto(const sensor_msgs::msg::PointField& ros)
{
    ::openbot::ros2_msgs::sensor_msgs::PointField proto;
    proto.set_name(ros.name);
    proto.set_offset(ros.offset);
    proto.set_datatype(ros.datatype);
    proto.set_count(ros.count);
    return proto;
}

::openbot::ros2_msgs::sensor_msgs::Range ToProto(const sensor_msgs::msg::Range& ros)
{
    ::openbot::ros2_msgs::sensor_msgs::Range proto;
    *proto.mutable_header() = ToProto(ros.header);
    proto.set_radiation_type(ros.radiation_type);
    proto.set_field_of_view(ros.field_of_view);
    proto.set_min_range(ros.min_range);
    proto.set_max_range(ros.max_range);
    proto.set_range(ros.range);
    return proto;
}

::openbot::ros2_msgs::sensor_msgs::RegionOfInterest ToProto(const sensor_msgs::msg::RegionOfInterest& ros)
{
    ::openbot::ros2_msgs::sensor_msgs::RegionOfInterest proto;
    proto.set_x_offset(ros.x_offset);
    proto.set_y_offset(ros.y_offset);
    proto.set_height(ros.height);
    proto.set_width(ros.width);
    proto.set_do_rectify(ros.do_rectify);
    return proto;
}

::openbot::ros2_msgs::shape_msgs::Mesh ToProto(const shape_msgs::msg::Mesh& ros)
{
    ::openbot::ros2_msgs::shape_msgs::Mesh proto;
    for (int i = 0; i < ros.triangles.size(); ++i) {
        *proto.mutable_triangles(i) = ToProto(ros.triangles[i]);
    }
    for (int i = 0; i < ros.vertices.size(); ++i) {
        *proto.mutable_vertices(i) = ToProto(ros.vertices[i]);
    }
    return proto;
}

::openbot::ros2_msgs::shape_msgs::MeshTriangle ToProto(const shape_msgs::msg::MeshTriangle& ros)
{
    ::openbot::ros2_msgs::shape_msgs::MeshTriangle proto;
    for (int i = 0; i < ros.vertex_indices.size(); ++i) {
        proto.set_vertex_indices(i, ros.vertex_indices[i]);
    }
    return proto;
}

::openbot::ros2_msgs::shape_msgs::Plane ToProto(const shape_msgs::msg::Plane& ros)
{
    ::openbot::ros2_msgs::shape_msgs::Plane proto;
    for (int i = 0; i < ros.coef.size(); ++i) {
        proto.set_coef(i, ros.coef[i]);
    }
    return proto;
}

::openbot::ros2_msgs::shape_msgs::SolidPrimitive ToProto(const shape_msgs::msg::SolidPrimitive& ros)
{
    ::openbot::ros2_msgs::shape_msgs::SolidPrimitive proto;
    proto.set_type(ros.type);
    for (int i = 0; i < ros.dimensions.size(); ++i) {
        proto.set_dimensions(i, ros.dimensions[i]);
    }
    *proto.mutable_polygon() = ToProto(ros.polygon);
    return proto;
}

}  // namespace openbot_ros