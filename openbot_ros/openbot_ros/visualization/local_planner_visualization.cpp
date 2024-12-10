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

#include "local_planner_visualization.hpp"


namespace openbot_ros {

LocalPlannerVisualizator::LocalPlannerVisualizator(rclcpp::Node* node)
    : node_{node}
{
    // sphere_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/spheres", 1000);
}

}  // namespace openbot_ros