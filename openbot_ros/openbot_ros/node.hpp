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

#ifndef OPENBOT_ROS_OPENBOT_ROS_NODE_HPP
#define OPENBOT_ROS_OPENBOT_ROS_NODE_HPP

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "openbot/common/msgs/msgs.hpp"
#include "openbot_ros/ros_log_sink.hpp"
#include "openbot_ros/node_options.hpp"
#include "openbot_ros/node_constants.hpp"
#include "openbot_msgs/msg/sensor_topics.hpp"
#include "openbot_ros/global_planner.hpp"
#include "openbot_ros/local_planner.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "visualization/global_planner_visualization.hpp"
#include "visualization/local_planner_visualization.hpp"

namespace openbot_ros {
class Node : public rclcpp::Node
{
public:
    Node(const NodeOptions& node_options, bool collect_metrics);
    ~Node();

    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    /**
     * @brief Receive `sensor_msgs::PointCloud2` map data as global map
     */
    void HandleMapMessageCallBack(const std::string& sensor_id, sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    /**
     * @brief Receive `RVIZ2 command` tartget pose
     */
    void HandleTargetPoseCallBack(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

    /**
     * @brief Create global path from sensor_msgs::PointCloud2
     */
    void CreateGlobalPath();

    /**
     * @brief node_handle
     */
    rclcpp::Node::SharedPtr node_handle();

    /**
     * @brief Get global_planner
     */
    GlobalPlanner::SharedPtr global_planner();

private:
    struct Subscriber 
    {
        rclcpp::SubscriptionBase::SharedPtr subscriber;

        // ::ros::Subscriber::getTopic() does not necessarily return the same
        // std::string
        // it was given in its constructor. Since we rely on the topic name as the
        // unique identifier of a subscriber, we remember it ourselves.
        std::string topic;
    };

    /**
     * @brief All sensor topics
     */
    void LaunchSubscribers(const openbot_msgs::msg::SensorTopics& topics);

    void PublishGlobalPath();

    // ROS2 Node
    ::rclcpp::Node::SharedPtr node_handle_{nullptr};
    ::rclcpp::TimerBase::SharedPtr global_planner_timer_{nullptr};

    // global_planer_visualizator
    GlobalPlannerVisualizator::SharedPtr global_planner_visualizator_{nullptr};

    // local planner_visualizator
    LocalPlannerVisualizator::SharedPtr local_planner_visualizator_{nullptr};

    // global_planner
    GlobalPlanner::SharedPtr global_planner_{nullptr};

    // local_planner
    LocalPlanner::SharedPtr local_planner_{nullptr};

    std::vector<Subscriber> subscribers_;
    std::unordered_set<std::string> subscribed_topics_;
    const NodeOptions node_options_;
};

}  // namespace openbot_ros

#endif  // OPENBOT_ROS_OPENBOT_ROS_NODE_HPP