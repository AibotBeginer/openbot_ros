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

#ifndef OPENBOT_ROS_OPENBOT_ROS_MOCKAMAP_GENERATOR_HPP
#define OPENBOT_ROS_OPENBOT_ROS_MOCKAMAP_GENERATOR_HPP

#include "glog/logging.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "openbot/map/rand_map_generator.hpp"

namespace openbot_ros {

class MockamapGenerator : public rclcpp::Node
{
public:
    MockamapGenerator();
    ~MockamapGenerator();

private:
    void PublishPointCloud2();

    openbot::map::mockamap::MapOption InitOption();

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    openbot::map::MapGenerator::SharedPtr generator_{nullptr};

    sensor_msgs::msg::PointCloud2 ros_pcl_data_;
    openbot::common::sensor_msgs::PointCloud2 non_ros_pcl_data_;
    bool finished_{false};

};

}  // namespace openbot_ros

#endif  // OPENBOT_ROS_OPENBOT_ROS_MOCKAMAP_GENERATOR_HPP