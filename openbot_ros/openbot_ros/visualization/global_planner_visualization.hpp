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

#ifndef OPENBOT_ROS_VISUALIZATION_GLOBAL_PLANNER_VISUALIZATION_HPP
#define OPENBOT_ROS_VISUALIZATION_GLOBAL_PLANNER_VISUALIZATION_HPP

#include <string>
#include <tuple>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

#include "openbot_ros/messages_conversion/proto_msgs_to_ros.hpp"
#include "openbot_ros/messages_conversion/ros_msgs_to_proto.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace openbot_ros {

class GlobalPlannerVisualizator
{
public:

    /**
     *  @brief SharedPtr typedef
     */
    RCLCPP_SMART_PTR_DEFINITIONS(GlobalPlannerVisualizator)

    /**
     *  @brief SharedPtr typedef
     */
    GlobalPlannerVisualizator(rclcpp::Node* node);
    
    /**
     *  @brief RVIZ2 show Start & Goal pose
     */
    void VisualizeStartGoal(geometry_msgs::msg::PoseStamped::ConstSharedPtr center, const double &radius, const int sg);

    void PublishGlobalMap(sensor_msgs::msg::PointCloud2& msgs);

    /**
     *  @brief Publish a path on a specific topic
     *  @param msgs The path message to publish
     *  @param color The color suffix to determine the topic ("red", "blue", etc.)
     */
    void PublishPath(const nav_msgs::msg::Path& msgs, const std::string& color);


private:
    rclcpp::Node* node_{nullptr};    // Publishers for each color-specific topic
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_{nullptr};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_red_{nullptr};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_blue_{nullptr};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_green_{nullptr};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_yellow_{nullptr};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_purple_{nullptr};

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sphere_publisher_{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_map_publisher_{nullptr};
};

}  // namespace openbot_ros

#endif  // OPENBOT_ROS_VISUALIZATION_GLOBAL_PLANNER_VISUALIZATION_HPP