// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "openbot_rviz/planning_panel.hpp"

#include <QtConcurrent/QtConcurrent>
#include <QVBoxLayout>

#include <memory>
#include <vector>
#include <utility>
#include <chrono>
#include <string>

#include "openbot_rviz/goal_common.hpp"
#include "rviz_common/display_context.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

namespace openbot_rviz
{
// Define global GoalPoseUpdater so that the nav2 GoalTool plugin can access to update goal pose
GoalPoseUpdater GoalUpdater;

PlanningPanel::PlanningPanel(QWidget * parent)
: Panel(parent)
{
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args", "--remap", "__node:=rviz_navigation_dialog_action_client", "--"});
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  QObject::connect(
    &GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)),                 // NOLINT
    this, SLOT(onNewGoal(double,double,double,QString)));  // NOLINT
}

PlanningPanel::~PlanningPanel()
{
}

void PlanningPanel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
}


void PlanningPanel::onNewGoal(double x, double y, double theta, QString frame)
{
  (void)x;
  (void)y;
  (void)theta;
  (void)frame;
}

void PlanningPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

void PlanningPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
}


}  // namespace openbot_rviz

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(openbot_rviz::PlanningPanel, rviz_common::Panel)
