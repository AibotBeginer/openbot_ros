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

#ifndef OPENBOT_ROS_SIMULATOR__GOAL_POSE_UPDATER_HPP_
#define OPENBOT_ROS_SIMULATOR__GOAL_POSE_UPDATER_HPP_

#include <QObject>

namespace openbot_ros_simulator
{

/// Class to set and update goal pose by emitting signal
class GoalPoseUpdater : public QObject
{
  Q_OBJECT

public:
  GoalPoseUpdater() {}
  ~GoalPoseUpdater() {}

  void setGoal(double x, double y, double theta, QString frame)
  {
    emit updateGoal(x, y, theta, frame);
  }

signals:
  void updateGoal(double x, double y, double theta, QString frame);
};

}  // namespace openbot_ros_simulator

#endif  //  OPENBOT_ROS_SIMULATOR__GOAL_POSE_UPDATER_HPP_
