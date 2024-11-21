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

    // global_planner_timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(1200), [this]() { PublishGlobalPath(); });
    
    base_frame_id_ = declare_parameter("base_frame_id", "base_link");

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
}

Node::~Node() 
{
}

void Node::HandleMapMessageCallBack(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!global_map_created) {
    global_planner_->CreateGlobalMap(msg);
    global_map_created = true;
  }
}

void Node::HandleTargetPoseCallBack(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
    if (msg == nullptr) {
        return;
    }
    LOG(INFO) << "Received Target Pose Callback: x: " << msg->pose.position.x << " y: " << msg->pose.position.y  << " z: " << msg->pose.position.z;

    geometry_msgs::msg::TransformStamped world_to_base_transform_stamped;
    try
    {
      world_to_base_transform_stamped = tf2_buffer_->lookupTransform(
          "map", base_frame_id_, msg->header.stamp,
          rclcpp::Duration::from_seconds(1.0));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }

    global_planner_->start_goal().clear();
    

    auto start = std::make_shared<geometry_msgs::msg::PoseStamped>();
    start->header.frame_id = "map";  // The frame of reference
    start->header.stamp = msg->header.stamp;  // Use the same timestamp as input

    // Set the position from the transform's translation
    start->pose.position.x = world_to_base_transform_stamped.transform.translation.x;
    start->pose.position.y = world_to_base_transform_stamped.transform.translation.y;
    start->pose.position.z = world_to_base_transform_stamped.transform.translation.z;

    // Set the orientation from the transform's rotation
    start->pose.orientation = world_to_base_transform_stamped.transform.rotation;

    auto goal = std::make_shared<geometry_msgs::msg::PoseStamped>();
    goal->pose.position.x = msg->pose.position.x;
    goal->pose.position.y = msg->pose.position.y;
    goal->pose.position.z = msg->pose.position.z;

    if (!global_planner_->CheckValid(goal)) {
        LOG(WARNING) << "Infeasible Position Selected !!!";
        return;
    }
   
    // global_planner_visualizator_->VisualizeStartGoal(goal, 0.25, global_planner_->start_goal().size());
    global_planner_->AddPose(start);
    global_planner_->AddPose(msg);

    std::vector<std::string> colors = {"simple", "red", "blue", "green", "yellow", "purple"};

    // Define the base timeout and increment for each path
    double base_timeout = 0.5;
    double timeout_increment = 1.0;

    for (size_t i = 0; i < colors.size(); ++i)
    {
        double timeout = base_timeout + i * timeout_increment;
        LOG(INFO) << "Publishing path with timeout: " << timeout << " and color: " << colors[i];

        // Publish the global path with the current timeout and color
        PublishGlobalPath(timeout, colors[i]);
    }
}

::rclcpp::Node::SharedPtr Node::node_handle()
{
    return node_handle_; 
}

GlobalPlanner::SharedPtr Node::global_planner()
{
    return global_planner_;
}

void Node::LaunchSubscribers(const openbot_msgs::msg::SensorTopics& topics)
{
    subscribers_.push_back(
        {SubscribeWithHandler<sensor_msgs::msg::PointCloud2>(&Node::HandleMapMessageCallBack, kGlobalMapTopic, node_handle_, this),
         kGlobalMapTopic}
    );

    // topic: "/goal_pose";
    subscribers_.push_back({
        SubscribeWithHandler<geometry_msgs::msg::PoseStamped>(&Node::HandleTargetPoseCallBack, kTargetTopic, node_handle_, this),
        kTargetTopic
    });
}

void Node::PublishGlobalPath(const double timeout, const std::string& color)
{
    // Generate the global path
    auto path = global_planner_->CreatePath(timeout);
    LOG(INFO) << "path size: " << path.poses.size();

    // Publish the path with the specified color
    global_planner_visualizator_->PublishPath(path, color);
}

}  // namespace openbot_ros