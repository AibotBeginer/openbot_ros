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

#ifndef NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_
#define NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_

#include <QtWidgets>
#include <QBasicTimer>
#undef NO_ERROR

#include <memory>
#include <string>
#include <vector>

#include "openbot_rviz/ros_action_qevent.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


class QPushButton;

namespace openbot_rviz
{

class InitialThread;

/// Panel to interface to the nav2 stack
class PlanningPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit PlanningPanel(QWidget * parent = 0);
  virtual ~PlanningPanel();

  void onInitialize() override;

  /// Load and save configuration data
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onNewGoal(double x, double y, double theta, QString frame);

private:

  // The (non-spinning) client node used to invoke the action client
  rclcpp::Node::SharedPtr client_node_;

  QStateMachine state_machine_;
  InitialThread * initial_thread_;

};

class InitialThread : public QThread
{
  Q_OBJECT

public:

  explicit InitialThread() {}

  void run() override
  {

  }

signals:

private:

};

}  // namespace openbot_rviz

#endif  //  NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_
