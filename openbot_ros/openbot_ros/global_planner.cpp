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

#include "openbot_ros/global_planner.hpp"
#include "openbot/map/voxel_map.hpp"
#include "glog/logging.h"

#include "openbot_ros/messages_conversion/nav_msgs_converter.hpp"

namespace openbot_ros {

GlobalPlanner::GlobalPlanner()
{
    planner_ = std::make_shared<::openbot::planning::PlannerServer>();
    map_generator_ = std::make_shared<RandomMapGenerator>();
    CreateGlobalMap();
}

void GlobalPlanner::AddPose(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
    // const double zGoal = 0.3;
    const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    start_goal_.emplace_back(goal);
}

bool GlobalPlanner::CheckValid(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
    return true;
}

std::vector<Eigen::Vector3d>& GlobalPlanner::start_goal()
{
    return start_goal_;
}

nav_msgs::msg::Path GlobalPlanner::CreatePath()
{
    nav_msgs::msg::Path path;
    if (start_goal_.size() != 2) {
        return path;
    }

    ::openbot::common::geometry_msgs::PoseStamped start;
    start.pose.position.x = start_goal_[0].x();
    start.pose.position.y = start_goal_[0].y();
    // start.pose.position.z = 0.3;
    start.pose.position.z = start_goal_[0].z();

    ::openbot::common::geometry_msgs::PoseStamped goal;
    goal.pose.position.x = start_goal_[1].x();
    goal.pose.position.y = start_goal_[1].y();
    // goal.pose.position.z = 0.3;
    goal.pose.position.z = start_goal_[1].z();

    auto msg = planner_->CreatePlan(start, goal);
    path = ToRos(msg);
    return path;
}

void GlobalPlanner::CreateGlobalMap()
{
    if (!map_generator_->Finished()) {
        map_generator_->Generate();
    }

    bool success = map_generator_->GetPointCloud2Data(map_data_);
    if (!success) {
        LOG(INFO) << "Get pointCloud2 data error.";
        return;
    }

    double voxelWidth = 0.25;
    std::vector<double> mapBound = {-20.0, 20.0, -20.0, 20.0, -20, 20.0};
    const Eigen::Vector3i xyz((mapBound[1] - mapBound[0]) / voxelWidth,
                              (mapBound[3] - mapBound[2]) / voxelWidth,
                              (mapBound[5] - mapBound[4]) / voxelWidth);

    const Eigen::Vector3d offset(mapBound[0], mapBound[2], mapBound[4]);

    auto voxel_map = std::make_shared<::openbot::map::VoxelMap>(xyz, offset, voxelWidth);
    size_t cur = 0;
    const size_t total = map_data_.data.size() / map_data_.point_step;
    float *fdata = (float *)(&map_data_.data[0]);
    for (size_t i = 0; i < total; i++)
    {
        cur = map_data_.point_step / sizeof(float) * i;

        if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
            std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
            std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
        {
            continue;
        }
        voxel_map->SetOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                fdata[cur + 1],
                                                fdata[cur + 2]));
    }

    voxel_map->Dilate(std::ceil(0.25 / voxel_map->GetScale()));
    planner_->InitMap(voxel_map);
}

sensor_msgs::msg::PointCloud2& GlobalPlanner::map_data()
{
    return map_data_;
}

}  // namespace openbot_ros
