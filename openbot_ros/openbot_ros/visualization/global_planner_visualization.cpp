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

#include "global_planner_visualization.hpp"


namespace openbot_ros {

GlobalPlannerVisualizator::GlobalPlannerVisualizator(rclcpp::Node* node)
    : node_{node}
{
    sphere_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/spheres", 1000);
    point_cloud_map_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", 1);
    path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("global_path", 10);
     // Initialize publishers for each color-specific topic
    path_publisher_red_ = node_->create_publisher<nav_msgs::msg::Path>("global_path_red", 10);
    path_publisher_blue_ = node_->create_publisher<nav_msgs::msg::Path>("global_path_blue", 10);
    path_publisher_green_ = node_->create_publisher<nav_msgs::msg::Path>("global_path_green", 10);
    path_publisher_yellow_ = node_->create_publisher<nav_msgs::msg::Path>("global_path_yellow", 10);
    path_publisher_purple_ = node_->create_publisher<nav_msgs::msg::Path>("global_path_purple", 10);
}

void GlobalPlannerVisualizator::VisualizeStartGoal(geometry_msgs::msg::PoseStamped::ConstSharedPtr center, const double &radius, const int sg)
{
    visualization_msgs::msg::Marker sphereMarkers, sphereDeleter;

    sphereMarkers.id = sg;
    sphereMarkers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    sphereMarkers.header.stamp = rclcpp::Clock().now();
    sphereMarkers.header.frame_id = "map";
    sphereMarkers.pose.orientation.w = 1.00;
    sphereMarkers.action = visualization_msgs::msg::Marker::ADD;
    sphereMarkers.ns = "StartGoal";
    sphereMarkers.color.r = 1.00;
    sphereMarkers.color.g = 0.00;
    sphereMarkers.color.b = 0.00;
    sphereMarkers.color.a = 1.00;
    sphereMarkers.scale.x = radius * 2.0;
    sphereMarkers.scale.y = radius * 2.0;
    sphereMarkers.scale.z = radius * 2.0;

    sphereDeleter = sphereMarkers;
    sphereDeleter.action = visualization_msgs::msg::Marker::DELETEALL;

    geometry_msgs::msg::Point point;
    point.x = center->pose.position.x;
    point.y = center->pose.position.y;
    point.z = center->pose.position.z;
    sphereMarkers.points.push_back(point);

    if (sg == 0)
    {
        sphere_publisher_->publish(sphereDeleter);
        sphereMarkers.header.stamp = rclcpp::Clock().now();
    }
    sphere_publisher_->publish(sphereMarkers);
}

void GlobalPlannerVisualizator::PublishGlobalMap(sensor_msgs::msg::PointCloud2& msgs)
{
    point_cloud_map_publisher_->publish(msgs);
}


void GlobalPlannerVisualizator::PublishPath(const nav_msgs::msg::Path& msgs, const std::string& color)
{
    if (color == "red") {
        path_publisher_red_->publish(msgs);
    } else if (color == "blue") {
        path_publisher_blue_->publish(msgs);
    } else if (color == "green") {
        path_publisher_green_->publish(msgs);
    } else if (color == "yellow") {
        path_publisher_yellow_->publish(msgs);
    } else if (color == "purple") {
        path_publisher_purple_->publish(msgs);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Invalid color '%s' specified. No path was published.", color.c_str());
        path_publisher_->publish(msgs);
    }
}

}  // namespace openbot_ros