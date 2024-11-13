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

#include "openbot_ros/mockamap_generator.hpp"
#include "openbot_ros/messages_conversion/sensor_msgs_converter.hpp"

#include "glog/logging.h"

namespace openbot_ros {

MockamapGenerator::MockamapGenerator()
    : rclcpp::Node("mockamap_node")
{
    generator_ = std::make_shared<openbot::map::MapGenerator>(InitOption());
    pcl_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", 1);

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
            std::bind(&MockamapGenerator::PublishPointCloud2, this));
}

MockamapGenerator::~MockamapGenerator()
{

}

void MockamapGenerator::PublishPointCloud2()
{
    if (!finished_) {
       finished_ = generator_->Generate(non_ros_pcl_data_);
       ros_pcl_data_ = ToRos(non_ros_pcl_data_);
    }
    pcl_publisher_->publish(ros_pcl_data_);
}

openbot::map::mockamap::MapOption MockamapGenerator::InitOption()
{
    openbot::map::mockamap::MapOption option;
    this->declare_parameter<int>("seed", 511);
    this->declare_parameter<double>("resolution", 0.1);
    this->declare_parameter<int>("x_length", 10);
    this->declare_parameter<int>("y_length", 10);
    this->declare_parameter<int>("z_length", 3);

    // 1 perlin noise 3D 
    // 2 perlin box random map
    // 3 2d maze 
    // 4 maze 3d
    this->declare_parameter<int>("type", 1);

    // 1 perlin noise parameters 
    // complexity:    base noise frequency, large value will be complex typical 0.0 ~ 0.5 
    // fill:          infill persentage  typical: 0.4 ~ 0.0 
    // fractal:       large value will have more detail
    // attenuation:   for fractal attenuation typical: 0.0 ~ 0.5 
    this->declare_parameter<double>("complexity", 0.03);
    this->declare_parameter<double>("fill", 0.3);
    this->declare_parameter<int>("fractal", 1);
    this->declare_parameter<double>("attenuation", 0.1);

    // 2 perlin box random map
    this->declare_parameter<double>("width_min", 0.6);
    this->declare_parameter<double>("width_max", 1.5);
    this->declare_parameter<int>("obstacle_number", 50);

    // 3 2d maze
    this->declare_parameter<double>("road_width", 0.5);
    this->declare_parameter<int>("add_wall_x", 0);
    this->declare_parameter<int>("add_wall_y", 1);
    this->declare_parameter<int>("maze_type", 1);

    // 4 maze 3d
    this->declare_parameter<int>("num_nodes", 40);
    this->declare_parameter<double>("connectivity", 0.8);
    this->declare_parameter<int>("node_rad", 1);
    this->declare_parameter<int>("road_rad", 10);

    // Retrieve parameters
    this->get_parameter("seed", option.seed);
    this->get_parameter("resolution", option.resolution);
    this->get_parameter("x_length", option.x_length);
    this->get_parameter("y_length", option.y_length);
    this->get_parameter("z_length", option.z_length);
    this->get_parameter("type", option.type);
    this->get_parameter("complexity", option.complexity);
    this->get_parameter("fill", option.fill);
    this->get_parameter("fractal", option.fractal);
    this->get_parameter("attenuation", option.attenuation);
    this->get_parameter("width_min", option.width_min);
    this->get_parameter("width_max", option.width_max);
    this->get_parameter("obstacle_number", option.obstacle_number);
    this->get_parameter("road_width", option.road_width);
    this->get_parameter("add_wall_x", option.add_wall_x);
    this->get_parameter("add_wall_y", option.add_wall_y);
    this->get_parameter("maze_type", option.maze_type);
    this->get_parameter("numNodes", option.num_nodes);
    this->get_parameter("connectivity", option.connectivity);
    this->get_parameter("nodeRad", option.node_rad);
    this->get_parameter("roadRad", option.road_rad);
    return option;
}

}  // namespace openbot_ros
