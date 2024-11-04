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
#include "openbot/common/configuration_file_resolver.hpp"
#include "openbot/common/lua_parameter_dictionary.hpp"
#include "openbot/common/port.hpp"
#include "openbot/common/time.hpp"

#include "glog/logging.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace openbot_ros {

namespace {

openbot_ros_msgs::msg::SensorTopics DefaultSensorTopics() 
{
  openbot_ros_msgs::msg::SensorTopics topics;
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
    void (Node::*handler)(const std::string&, typename MessageType::ConstSharedPtr), 
    const std::string& topic,
    ::rclcpp::Node::SharedPtr node_handle, 
    Node* const node) 
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
    const std::string& topic,
    ::rclcpp::Node::SharedPtr node_handle, 
    Node* const node) 
{
  return node_handle->create_subscription<MessageType>(
      topic, rclcpp::SensorDataQoS(),
      [node, handler, topic](const typename MessageType::ConstSharedPtr msg) 
      {
          (node->*handler)(msg);
      });
}

}  // namespace

Node::Node(const NodeOptions& node_options, const bool collect_metrics) 
    : rclcpp::Node("openbot_ros_node"),
      node_options_(node_options)
{
    node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
    global_planner_ = std::make_shared<GlobalPlanner>();

    global_planner_visualizator_ = std::make_shared<GlobalPlannerVisualizator>(node_handle_.get());
    local_planner_visualizator_ = std::make_shared<LocalPlannerVisualizator>(node_handle_.get());
    LaunchSubscribers(DefaultSensorTopics());

    global_planner_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), [this]() { PublishGlobalPath(); });
}

Node::~Node() 
{
}

void Node::HandleMapMessageCallBack(const std::string& sensor_id, sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{

}

void Node::HandleTargetPoseCallBack(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
    if (msg == nullptr) {
        return;
    }

    global_planner_->AddPose(msg);
    if (global_planner_->start_goal().size() >= 2) {
        global_planner_->start_goal().clear();
    }

    auto goal = std::make_shared<geometry_msgs::msg::PoseStamped>();
    goal->pose.position.x = msg->pose.position.x;
    goal->pose.position.y = msg->pose.position.y;
    goal->pose.position.z = msg->pose.position.z;


    if (!global_planner_->CheckValid(goal)) {
        LOG(WARNING) << "Infeasible Position Selected !!!";
    }
    
    global_planner_visualizator_->VisualizeStartGoal(goal, 0.5, global_planner_->start_goal().size());
}

::rclcpp::Node::SharedPtr Node::node_handle()
{
    return node_handle_; 
}

GlobalPlanner::SharedPtr Node::global_planner()
{
    return global_planner_;
}

void Node::LaunchSubscribers(const openbot_ros_msgs::msg::SensorTopics& topics)
{
    // std::string topic = kGlobalMapTopic;
    // subscribers_.push_back(
    //     {SubscribeWithHandler<nav_msgs::msg::Odometry>(&Node::HandleOdometryMessage, topic, node_handle_, this),
    //      topic}
    // );

    // topic: "/goal_pose";
    subscribers_.push_back({
        SubscribeWithHandler<geometry_msgs::msg::PoseStamped>(&Node::HandleTargetPoseCallBack, kTargetTopic, node_handle_, this),
        kTargetTopic
    });
}

void Node::PublishGlobalPath()
{
    auto data = global_planner_->map_data();
    global_planner_visualizator_->PublishGlobalMap(data);

    auto path = global_planner_->CreatePath();
    LOG(INFO) << "path size: " << path.poses.size();
    global_planner_visualizator_->PublishPath(path);
}

}  // namespace openbot_ros