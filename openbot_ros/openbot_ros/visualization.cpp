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

#include "openbot_ros/visualization.hpp"

#include "glog/log_severity.h"
#include "glog/logging.h"

namespace openbot_ros {


Visualization::Visualization(rclcpp::Node* node)
    : node_(node)
{
    path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("openbot_path", 10);
    point_cloud_map_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", 1);
    map_generator_ = std::make_shared<RandomMapGenerator>();
}
    
void Visualization::PublishPath(const ProtoPath& path)
{
    auto path_msg = ToRos(path);
    path_publisher_->publish(path_msg);
}

void Visualization::PublishGlobalMap()
{
    if (!map_generator_->Finished()) {
        map_generator_->Generate();
    }

    sensor_msgs::msg::PointCloud2 map_data;
    bool success = map_generator_->GetPointCloud2Data(map_data);
    if (!success) {
        RCLCPP_INFO(node_->get_logger(), "Get pointCloud2 data error.");
        return;
    }
    point_cloud_map_publisher_->publish(map_data);
}

}  // namespace openbot_ros
