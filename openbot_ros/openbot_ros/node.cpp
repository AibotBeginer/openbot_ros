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

#include "openbot_ros/node.hpp"

#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "absl/memory/memory.h"

#include "glog/logging.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace openbot_ros {
namespace {

    openbot_msgs::msg::SensorTopics DefaultSensorTopics()
    {
      openbot_msgs::msg::SensorTopics topics;
      topics.laser_scan_topic = kLaserScanTopic;
      topics.multi_echo_laser_scan_topic = kMultiEchoLaserScanTopic;
      topics.point_cloud2_topic = kPointCloud2Topic;
      topics.imu_topic = kImuTopic;
      topics.odometry_topic = kOdometryTopic;
      topics.nav_sat_fix_topic = kNavSatFixTopic;
      topics.landmark_topic = kLandmarkTopic;
      return topics;
    }

    // Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
    // calls 'handler' on the 'node' to handle messages. Returns the subscriber.
    template <typename MessageType>
    ::rclcpp::SubscriptionBase::SharedPtr SubscribeWithHandler(
        void (Node::*handler)(const std::string &, typename MessageType::ConstSharedPtr),
        const std::string &topic,
        ::rclcpp::Node::SharedPtr node_handle,
        Node *const node)
    {
      return node_handle->create_subscription<MessageType>(
          topic, rclcpp::SensorDataQoS(),
          [node, handler, topic](const typename MessageType::ConstSharedPtr msg)
          {
            (node->*handler)(topic, msg);
          });
    }

    // Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
    // calls 'handler' on the 'node' to handle messages. Returns the subscriber.
    template <typename MessageType>
    ::rclcpp::SubscriptionBase::SharedPtr SubscribeWithHandler(
        void (Node::*handler)(typename MessageType::ConstSharedPtr),
        const std::string &topic,
        ::rclcpp::Node::SharedPtr node_handle,
        Node *const node)
    {
      return node_handle->create_subscription<MessageType>(
          topic, rclcpp::SensorDataQoS(),
          [node, handler, topic](const typename MessageType::ConstSharedPtr msg)
          {
            (node->*handler)(msg);
          });
    }

} // namespace

Node::Node(const NodeOptions &node_options, const bool collect_metrics)
    : rclcpp::Node("openbot_ros_node"),
      node_options_(node_options)
{
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  LaunchSubscribers(DefaultSensorTopics());
}

Node::~Node()
{
}

void Node::HandleMapMessageCallBack(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  
}

void Node::HandleTargetPoseCallBack(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  if (msg == nullptr)
  {
    return;
  }
  LOG(INFO) << "Received Target Pose Callback: x: " << msg->pose.position.x << " y: " << msg->pose.position.y << " z: " << msg->pose.position.z;
}

::rclcpp::Node::SharedPtr Node::node_handle()
{
  return node_handle_;
}

void Node::LaunchSubscribers(const openbot_msgs::msg::SensorTopics &topics)
{
  subscribers_.push_back(
      {SubscribeWithHandler<sensor_msgs::msg::PointCloud2>(&Node::HandleMapMessageCallBack, kGlobalMapTopic, node_handle_, this),
        kGlobalMapTopic});

  // topic: "/goal_pose";
  subscribers_.push_back({SubscribeWithHandler<geometry_msgs::msg::PoseStamped>(&Node::HandleTargetPoseCallBack, kTargetTopic, node_handle_, this),
                          kTargetTopic});
}



} // namespace openbot_ros