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

#include "openbot_ros/sensor_bridge.hpp"

#include "glog/logging.h"
#include "gtest/gtest.h"

#include "absl/memory/memory.h"
#include "openbot_ros/msg_conversion.hpp"
#include "openbot_ros/time_conversion.hpp"

namespace openbot_ros {

namespace {

const std::string& CheckNoLeadingSlash(const std::string& frame_id) {
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id
                               << " should not start with a /. See 1.7 in "
                                  "http://wiki.ros.org/tf2/Migration.";
  }
  return frame_id;
}

}  // namespace

SensorBridge::SensorBridge(
    const std::string& tracking_frame,
    const double lookup_transform_timeout_sec, tf2_ros::Buffer* const tf_buffer)
    : tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer)
{}

void SensorBridge::HandleOdometryMessage(const std::string& sensor_id, const nav_msgs::msg::Odometry::ConstSharedPtr& msg) 
{
  // std::unique_ptr<carto::sensor::OdometryData> odometry_data = ToOdometryData(msg);
  // if (odometry_data != nullptr) {
  //   trajectory_builder_->AddSensorData(
  //       sensor_id,
  //       carto::sensor::OdometryData{odometry_data->time, odometry_data->pose});
  // }
}

void SensorBridge::HandleImuMessage(const std::string& sensor_id, const sensor_msgs::msg::Imu::ConstSharedPtr& msg) 
{
  // std::unique_ptr<carto::sensor::ImuData> imu_data = ToImuData(msg);
  // if (imu_data != nullptr) {
  //   trajectory_builder_->AddSensorData(
  //       sensor_id,
  //       carto::sensor::ImuData{imu_data->time, imu_data->linear_acceleration,
  //                              imu_data->angular_velocity});
  // }
}

void SensorBridge::HandleImageMessage(const std::string& sensor_id, 
  const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{

}

void SensorBridge::HandleImageMessage(const std::string& sensor_id, 
    const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& right_msg)
{

}

const TfBridge& SensorBridge::tf_bridge() const { return tf_bridge_; }


}  // namespace openbot_ros
