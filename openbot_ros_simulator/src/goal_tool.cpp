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

#include "openbot_ros_simulator/goal_tool.hpp"

#include <memory>
#include <string>

#include "openbot_ros_simulator/goal_common.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"

namespace openbot_ros_simulator
{

GoalTool::GoalTool()
: rviz_default_plugins::tools::PoseTool()
{
  shortcut_key_ = 'g';
}

GoalTool::~GoalTool()
{
}

void GoalTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("3D Goal");
  setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/SetGoal.png"));
}

void GoalTool::onPoseSet(double x, double y, double theta)
{
  // Set goal pose on global object GoalUpdater to update nav2 Panel
  GoalUpdater.setGoal(x, y, theta, context_->getFixedFrame());
}

}  // namespace openbot_ros_simulator

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(openbot_ros_simulator::GoalTool, rviz_common::Tool)
