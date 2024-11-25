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

#ifndef OPENBOT_SIMULATOR_OPENBOT_DRIVE_HPP_
#define OPENBOT_SIMULATOR_OPENBOT_DRIVE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5

#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1
#define TB3_RIGHT_TURN    2
#define TB3_LEFT_TURN     3

namespace openbot_ros {

class OpenbotDrive : public rclcpp::Node
{
public:
  OpenbotDrive();
  ~OpenbotDrive();

private:
  // Function prototypes
  void update_callback();
  void update_cmd_vel(double linear, double angular);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_{nullptr};

  // ROS topic subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_{nullptr};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_{nullptr};

  // Variables
  double robot_pose_;
  double prev_robot_pose_;
  double scan_data_[3];

  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_{nullptr};
};

}  // namespace openbot_ros 

#endif  // OPENBOT_SIMULATOR_OPENBOT_DRIVE_HPP_
