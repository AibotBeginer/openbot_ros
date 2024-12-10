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

#ifndef OPENBOT_ROS_OPENBOT_ROS_MAP_GENERATOR_HPP
#define OPENBOT_ROS_OPENBOT_ROS_MAP_GENERATOR_HPP

#include <iostream>
#include <cmath>
#include <random>
#include <vector>

#include <Eigen/Eigen>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "glog/logging.h"

namespace openbot_ros {

struct MapOption 
{
    int obs_num;
    int cir_num;
    double x_size;
    double y_size;
    double z_size;
    double init_x;
    double init_y;
    double resolution;
    double sense_rate;
    double x_l;
    double x_h;

    double y_l;
    double y_h;

    double w_l;
    double w_h;

    double h_l;
    double h_h;
    
    double w_c_l;
    double w_c_h;
};

class RandomMapGenerator
{
public:
    /**
     *  @brief SharedPtr typedef
     */
    RCLCPP_SMART_PTR_DEFINITIONS(RandomMapGenerator)


    /**
     * @brief Contructor for RandomMapGenerator
     */
    RandomMapGenerator();

    /**
     * @brief Contructor for RandomMapGenerator
     */
    RandomMapGenerator(const MapOption& config);

    /**
     * @brief Generate sensor_msgs::msg::PointCloud2
     */
    sensor_msgs::msg::PointCloud2& Generate();

    /**
     * @brief Get sensor_msgs::msg::PointCloud2
     */
    bool GetPointCloud2Data(sensor_msgs::msg::PointCloud2& data);

    /**
     * @brief Check finished sensor_msgs::msg::PointCloud2
     */
    bool Finished() { return _has_map; }

private:

    /**
     * @brief Init default map opiton
     */
    void InitDefaultConfig();

    MapOption* default_config_{nullptr};
    bool _has_map  = false;

    sensor_msgs::msg::PointCloud2 globalMap_pcd;
    pcl::PointCloud<pcl::PointXYZ> cloudMap;

    pcl::search::KdTree<pcl::PointXYZ> kdtreeMap;
    std::vector<int>     pointIdxSearch;
    std::vector<float>   pointSquaredDistance;  
};

}  // namespace openbot_ros

#endif  // OPENBOT_ROS_OPENBOT_ROS_MAP_GENERATOR_HPP