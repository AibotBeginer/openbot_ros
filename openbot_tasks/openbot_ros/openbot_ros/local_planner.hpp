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

#ifndef OPENBOT_ROS_LOCAL_PLANNER_HPP
#define OPENBOT_ROS_LOCAL_PLANNER_HPP

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <memory>
#include <vector>
#include <string>

#include <Eigen/Eigen>

#include "openbot/common/macros.hpp"

namespace openbot_ros {

class LocalPlanner 
{
public:

    OPENBOT_SMART_PTR_DEFINITIONS(LocalPlanner)

    LocalPlanner();

private:
};

}  // namespace openbot_ros

#endif  // OPENBOT_ROS_LOCAL_PLANNER_HPP
