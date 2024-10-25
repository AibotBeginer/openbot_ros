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

#include "messages_conversion/proto_msgs_to_ros.hpp"


namespace openbot_ros {

rclcpp::Time ToRos(const ::openbot::common::proto::builtin_interfaces::Time& proto)
{
    return rclcpp::Time {proto.sec(), proto.nanosec()};
}

std_msgs::msg::Header ToRos(const ::openbot::common::proto::std_msgs::Header& proto)
{
    std_msgs::msg::Header data;
    data.stamp = ToRos(proto.stamp());
    data.frame_id = proto.frame_id();
    return data;
}  

geometry_msgs::msg::Accel ToRos(const ::openbot::common::proto::geometry_msgs::Accel& proto)
{
    geometry_msgs::msg::Accel data;
    data.linear = ToRos(proto.linear());
    data.angular = ToRos(proto.angular());
    return data;
}

geometry_msgs::msg::AccelStamped ToRos(const ::openbot::common::proto::geometry_msgs::AccelStamped& proto)
{
    geometry_msgs::msg::AccelStamped data;
    data.header = ToRos(proto.header());
    data.accel = ToRos(proto.accel());
    return data;
}

geometry_msgs::msg::AccelWithCovariance ToRos(const ::openbot::common::proto::geometry_msgs::AccelWithCovariance& proto)
{
    geometry_msgs::msg::AccelWithCovariance data;
    data.accel = ToRos(proto.accel());

    for (int i = 0; i < proto.covariance_size(); ++i) {
        data.covariance[i] = proto.covariance(i);
    }
    return data;
}

geometry_msgs::msg::AccelWithCovarianceStamped ToRos(
    const ::openbot::common::proto::geometry_msgs::AccelWithCovarianceStamped& proto)
{
    geometry_msgs::msg::AccelWithCovarianceStamped data;
    data.header = ToRos(proto.header());
    data.accel = ToRos(proto.accel());
    return data;
}

geometry_msgs::msg::Inertia ToRos(const ::openbot::common::proto::geometry_msgs::Inertia& proto)
{
    geometry_msgs::msg::Inertia data;
    data.m = proto.m();
    data.com = ToRos(proto.com());
    data.ixx = proto.ixx();
    data.ixy = proto.ixy();
    data.ixz = proto.ixz();
    data.iyy = proto.iyy();
    data.iyz = proto.iyz();
    data.izz = proto.izz();
    return data;
}

geometry_msgs::msg::InertiaStamped ToRos(const ::openbot::common::proto::geometry_msgs::InertiaStamped& proto)
{
    geometry_msgs::msg::InertiaStamped data;
    data.header = ToRos(proto.header());
    data.inertia = ToRos(proto.inertia());
    return data;
}

geometry_msgs::msg::Point ToRos(const ::openbot::common::proto::geometry_msgs::Point& proto)
{
    geometry_msgs::msg::Point data;
    data.x = proto.x();
    data.y = proto.y();
    data.z = proto.z();
    return data;
}

geometry_msgs::msg::Point32 ToRos(const ::openbot::common::proto::geometry_msgs::Point32& proto)
{
    geometry_msgs::msg::Point32 data;
    data.x = proto.x();
    data.y = proto.y();
    data.z = proto.z();
    return data;
}

geometry_msgs::msg::PointStamped ToRos(const ::openbot::common::proto::geometry_msgs::PointStamped& proto)
{
    geometry_msgs::msg::PointStamped data;
    data.header = ToRos(proto.header());
    data.point = ToRos(proto.point());
    return data;
}

geometry_msgs::msg::Polygon ToRos(const ::openbot::common::proto::geometry_msgs::Polygon& proto)
{
    geometry_msgs::msg::Polygon data;
    for (int i = 0 ; i < proto.points_size(); ++i) {
        data.points[i] = ToRos(proto.points(i));
    }
    return data;
}

geometry_msgs::msg::PolygonStamped ToRos(const ::openbot::common::proto::geometry_msgs::PolygonStamped& proto)
{
    geometry_msgs::msg::PolygonStamped data;
    data.header = ToRos(proto.header());
    data.polygon = ToRos(proto.polygon());
    return data;
}

geometry_msgs::msg::Pose ToRos(const ::openbot::common::proto::geometry_msgs::Pose& proto)
{
    geometry_msgs::msg::Pose data;
    data.position = ToRos(proto.position());
    data.orientation = ToRos(proto.orientation());
    return data;
}

geometry_msgs::msg::Pose2D ToRos(const ::openbot::common::proto::geometry_msgs::Pose2D& proto)
{
    geometry_msgs::msg::Pose2D data;
    data.x = proto.x();
    data.y = proto.y();
    data.theta = proto.theta();
    return data;
}

geometry_msgs::msg::PoseArray ToRos(const ::openbot::common::proto::geometry_msgs::PoseArray& proto)
{
    geometry_msgs::msg::PoseArray data;
    data.header = ToRos(proto.header());
    for (int i = 0; i < proto.poses_size(); ++i) {
        data.poses[i] = ToRos(proto.poses(i));
    }
    return data;
}

geometry_msgs::msg::PoseStamped ToRos(const ::openbot::common::proto::geometry_msgs::PoseStamped& proto)
{
    geometry_msgs::msg::PoseStamped data;
    data.header = ToRos(proto.header());
    data.pose = ToRos(proto.pose());
    return data;
}

geometry_msgs::msg::PoseWithCovariance ToRos(const ::openbot::common::proto::geometry_msgs::PoseWithCovariance& proto)
{
    geometry_msgs::msg::PoseWithCovariance data;
    data.pose = ToRos(proto.pose());
    for (int i = 0; i < proto.covariance_size(); ++i) {
        data.covariance[i] = proto.covariance(i);
    }
    return data;
}

geometry_msgs::msg::PoseWithCovarianceStamped ToRos(
    const ::openbot::common::proto::geometry_msgs::PoseWithCovarianceStamped& proto)
{
    geometry_msgs::msg::PoseWithCovarianceStamped data;
    data.header = ToRos(proto.header());
    data.pose = ToRos(proto.pose());
    return data;
}

geometry_msgs::msg::Quaternion ToRos(const ::openbot::common::proto::geometry_msgs::Quaternion& proto)
{
    geometry_msgs::msg::Quaternion data;
    data.x = proto.x();
    data.y = proto.y();
    data.z = proto.z();
    data.w = proto.w();
    return data;
}

geometry_msgs::msg::QuaternionStamped ToRos(const ::openbot::common::proto::geometry_msgs::QuaternionStamped& proto)
{
    geometry_msgs::msg::QuaternionStamped data;
    data.header = ToRos(proto.header());
    data.quaternion = ToRos(proto.quaternion());
    return data;
}

geometry_msgs::msg::Transform ToRos(const ::openbot::common::proto::geometry_msgs::Transform& proto)
{
    geometry_msgs::msg::Transform data;
    data.translation = ToRos(proto.translation());
    data.rotation = ToRos(proto.rotation());
    return data;
}

geometry_msgs::msg::TransformStamped ToRos(const ::openbot::common::proto::geometry_msgs::TransformStamped& proto)
{
    geometry_msgs::msg::TransformStamped data;
    data.header = ToRos(proto.header());
    data.child_frame_id = proto.child_frame_id();
    data.transform = ToRos(proto.transform());
    return data;
}

geometry_msgs::msg::Twist ToRos(const ::openbot::common::proto::geometry_msgs::Twist& proto)
{
    geometry_msgs::msg::Twist data;
    data.linear = ToRos(proto.linear());
    data.angular = ToRos(proto.angular());
    return data;
}

geometry_msgs::msg::TwistStamped ToRos(const ::openbot::common::proto::geometry_msgs::TwistStamped& proto)
{
    geometry_msgs::msg::TwistStamped data;
    data.header = ToRos(proto.header());
    data.twist = ToRos(proto.twist());
    return data;
}

geometry_msgs::msg::Vector3 ToRos(const ::openbot::common::proto::geometry_msgs::Vector3& proto)
{
    geometry_msgs::msg::Vector3 data;
    data.x = proto.x();
    data.y = proto.y();
    data.z = proto.z();
    return data;
}

geometry_msgs::msg::Vector3Stamped ToRos(const ::openbot::common::proto::geometry_msgs::Vector3Stamped& proto)
{
    geometry_msgs::msg::Vector3Stamped data;
    data.header = ToRos(proto.header());
    data.vector = ToRos(proto.vector());
    return data;
}

geometry_msgs::msg::TwistWithCovariance ToRos(const ::openbot::common::proto::geometry_msgs::TwistWithCovariance& proto)
{
    geometry_msgs::msg::TwistWithCovariance data;
    data.twist = ToRos(proto.twist());
    for (int i = 0; i < proto.covariance_size(); ++i) {
        data.covariance[i] = proto.covariance(i);
    }
    return data;
}

geometry_msgs::msg::TwistWithCovarianceStamped ToRos(
    const ::openbot::common::proto::geometry_msgs::TwistWithCovarianceStamped& proto)
{
    geometry_msgs::msg::TwistWithCovarianceStamped data;
    data.header = ToRos(proto.header());
    data.twist = ToRos(proto.twist());
    return data;
}

geometry_msgs::msg::VelocityStamped ToRos(const ::openbot::common::proto::geometry_msgs::VelocityStamped& proto)
{
    geometry_msgs::msg::VelocityStamped data;
    data.header = ToRos(proto.header());
    data.velocity = ToRos(proto.velocity());
    return data;
}

nav_msgs::msg::GridCells ToRos(const ::openbot::common::proto::nav_msgs::GridCells& proto)
{
    nav_msgs::msg::GridCells data;
    data.header = ToRos(proto.header());
    data.cell_width = proto.cell_width();
    data.cell_height = proto.cell_height();
    for (int i = 0; i < proto.cells_size(); ++i) {
        data.cells[i] = ToRos(proto.cells(i));
    }
    return data;
}

nav_msgs::msg::MapMetaData ToRos(const ::openbot::common::proto::nav_msgs::MapMetaData& proto)
{
    nav_msgs::msg::MapMetaData data;
    data.map_load_time = ToRos(proto.map_load_time());
    data.resolution = proto.resolution();
    data.width = proto.width();
    data.height = proto.height();
    data.origin = ToRos(proto.origin());
    return data;
}

nav_msgs::msg::OccupancyGrid ToRos(const ::openbot::common::proto::nav_msgs::OccupancyGrid& proto)
{
    nav_msgs::msg::OccupancyGrid data;
    data.header = ToRos(proto.header());
    data.info = ToRos(proto.info());
    for (int i = 0; i < proto.data_size(); ++i) {
        data.data[i] = proto.data(i);
    }
    return data;
}

nav_msgs::msg::Odometry ToRos(const ::openbot::common::proto::nav_msgs::Odometry& proto)
{
    nav_msgs::msg::Odometry data;
    data.header = ToRos(proto.header());
    data.child_frame_id = proto.child_frame_id();
    data.pose = ToRos(proto.pose());
    data.twist = ToRos(proto.twist());
    return data;
}

nav_msgs::msg::Path ToRos(const ::openbot::common::proto::nav_msgs::Path& proto)
{
    nav_msgs::msg::Path data;
    data.header = ToRos(proto.header());
    for (int i = 0; i < proto.poses_size(); ++i) {
        data.poses[i] = ToRos(proto.poses(i));
    }
    return data;
}

sensor_msgs::msg::CameraInfo ToRos(const ::openbot::common::proto::sensor_msgs::CameraInfo& proto)
{
    sensor_msgs::msg::CameraInfo data;
    data.header = ToRos(proto.header());
    data.height = proto.height();
    data.width = proto.width();

    for (int i = 0; i < proto.d_size(); ++i) {
        data.d[i] = proto.d(i);
    }

    for (int i = 0; i < proto.k_size(); ++i) {
        data.k[i] = proto.k(i);
    }

    for (int i = 0; i < proto.r_size(); ++i) {
        data.r[i] = proto.r(i);
    }

    for (int i = 0; i < proto.p_size(); ++i) {
        data.p[i] = proto.p(i);
    }

    data.binning_x = proto.binning_x();
    data.binning_y = proto.binning_y();
    data.roi = ToRos(proto.roi());
    return data;
}

sensor_msgs::msg::ChannelFloat32 ToRos(const ::openbot::common::proto::sensor_msgs::ChannelFloat32& proto)
{
    sensor_msgs::msg::ChannelFloat32 data;
    data.name = proto.name();
    for (int i = 0; i < proto.values_size(); ++i) {
        data.values[i] = proto.values(i);
    }
    return data;
}

sensor_msgs::msg::CompressedImage ToRos(const ::openbot::common::proto::sensor_msgs::CompressedImage& proto)
{
    sensor_msgs::msg::CompressedImage data;
    data.header = ToRos(proto.header());
    data.format = proto.format();
    for (int i = 0; i < proto.data_size(); ++i) {
        data.data[i] = proto.data(i);
    }
    return data;
}

sensor_msgs::msg::Illuminance ToRos(const ::openbot::common::proto::sensor_msgs::Illuminance& proto)
{
    sensor_msgs::msg::Illuminance data;
    data.header = ToRos(proto.header());
    data.illuminance = proto.illuminance();
    data.variance = proto.variance();
    return data;
}

sensor_msgs::msg::Image ToRos(const ::openbot::common::proto::sensor_msgs::Image& proto)
{
    sensor_msgs::msg::Image data;
    data.header = ToRos(proto.header());
    data.height = proto.height();
    data.width = proto.width();
    data.encoding = proto.encoding();
    data.is_bigendian = proto.is_bigendian();
    data.step = proto.step();
    for (int i = 0; i < proto.data_size(); ++i) {
        data.data[i] = proto.data(i);
    }
    return data;
}

sensor_msgs::msg::Imu ToRos(const ::openbot::common::proto::sensor_msgs::Imu& proto)
{
    sensor_msgs::msg::Imu data;
    data.header = ToRos(proto.header());
    data.orientation = ToRos(proto.orientation());

    for (int i = 0; i < proto.orientation_covariance_size(); ++i) {
        data.orientation_covariance[i] = proto.orientation_covariance(i);
    }
    data.angular_velocity = ToRos(proto.angular_velocity());
    return data;
}

// LaserScan
sensor_msgs::msg::LaserScan ToRos(const ::openbot::common::proto::sensor_msgs::LaserScan& proto)
{
    sensor_msgs::msg::LaserScan data;
    data.header = ToRos(proto.header());
    data.angle_min = proto.angle_min();
    data.angle_max = proto.angle_max();
    data.angle_increment = proto.angle_increment();
    data.time_increment = proto.time_increment();
    data.scan_time = proto.scan_time();
    data.range_min = proto.range_min();
    data.range_max = proto.range_max();

    for (int i = 0; i < proto.ranges_size(); ++i) {
        data.ranges[i] = proto.ranges(i);
    }

    for (int i = 0; i < proto.intensities_size(); ++i) {
        data.intensities[i] = proto.intensities(i);
    }
    return data;
}

sensor_msgs::msg::PointCloud ToRos(const ::openbot::common::proto::sensor_msgs::PointCloud& proto)
{
    sensor_msgs::msg::PointCloud data;
    data.header = ToRos(proto.header());

    for (int i = 0; i < proto.points_size(); ++i) {
        data.points[i] = ToRos(proto.points(i));
    }

    for (int i = 0; i < proto.channels_size(); ++i) {
        data.channels[i] = ToRos(proto.channels(i));
    }
    
    return data;
}

sensor_msgs::msg::PointCloud2 ToRos(const ::openbot::common::proto::sensor_msgs::PointCloud2& proto)
{
    sensor_msgs::msg::PointCloud2 data;
    data.header = ToRos(proto.header());
    data.height = proto.height();
    data.width = proto.width();
    for (int i = 0; i < proto.fields_size(); ++i) {
        data.fields[i] = ToRos(proto.fields(i));
    }
    data.is_bigendian = proto.is_bigendian();
    data.point_step = proto.point_step();
    for (int i = 0; i < proto.data_size(); ++i) {
        data.data[i] = proto.data(i);
    }
    data.is_dense = proto.is_dense();
    return data;
}

sensor_msgs::msg::PointField ToRos(const ::openbot::common::proto::sensor_msgs::PointField& proto)
{
    sensor_msgs::msg::PointField data;
    data.name = proto.name();
    data.offset = proto.offset();
    data.datatype = proto.datatype();
    data.count = proto.count();
    return data;
}

sensor_msgs::msg::Range ToRos(const ::openbot::common::proto::sensor_msgs::Range& proto)
{
    sensor_msgs::msg::Range data;
    data.header = ToRos(proto.header());
    data.radiation_type = proto.radiation_type();
    data.field_of_view = proto.field_of_view();
    data.min_range = proto.min_range();
    data.max_range = proto.max_range();
    data.range = proto.range();
    return data;
}

sensor_msgs::msg::RegionOfInterest ToRos(const ::openbot::common::proto::sensor_msgs::RegionOfInterest& proto)
{
    sensor_msgs::msg::RegionOfInterest data;
    data.x_offset = proto.x_offset();
    data.y_offset = proto.y_offset();
    data.height = proto.height();
    data.width = proto.width();
    data.do_rectify = proto.do_rectify();
    return data;
}


}  // namespace openbot_ros