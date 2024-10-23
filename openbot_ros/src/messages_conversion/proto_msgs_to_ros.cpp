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

}  // namespace openbot_ros