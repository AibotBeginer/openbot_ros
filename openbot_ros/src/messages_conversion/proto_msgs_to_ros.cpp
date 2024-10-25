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
    // data.roi = ToRos(proto.roi());
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
    // data.orientation = ToRos(proto.orientation());

    // for (int i = 0; i < proto.orientation_covariance_size(); ++i) {
    //     data.orientation_covariance[i] = proto.orientation_covariance(i);
    // }
    // data.angular_velocity = ToRos(proto.angular_velocity());
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

    // for (int i = 0; i < proto.points_size(); ++i) {
    //     data.points[i] = proto.points(i);
    // }

    // for (int i = 0; i < proto.channels_size(); ++i) {
    //     data.channels[i] = proto.channels(i);
    // }
    
    return data;
}

sensor_msgs::msg::PointCloud2 ToRos(const ::openbot::common::proto::sensor_msgs::PointCloud2& proto)
{
    sensor_msgs::msg::PointCloud2 data;
    data.header = ToRos(proto.header());
    data.height = proto.height();
    data.width = proto.width();
    // for (int i = 0; i < proto.fields_size(); ++i) {
    //     data.fields[i] = proto.fields(i);
    // }
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