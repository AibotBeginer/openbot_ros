

#ifndef RVIZ_DEFAULT_PLUGINS__TOOLS__GOAL_3D_NEW_TOOL_HPP_
#define RVIZ_DEFAULT_PLUGINS__TOOLS__GOAL_3D_NEW_TOOL_HPP_

#include "openbot_rviz/pose_3d_tool.hpp"

#include <QObject>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{
class DisplayContext;
namespace properties
{
class StringProperty;
class QosProfileProperty;
}  // namespace properties
}  // namespace rviz_common

namespace openbot_rviz
{
  // ref: https://github.com/ros2/rviz/blob/rolling/rviz_default_plugins/include/rviz_default_plugins/tools/goal_pose/goal_tool.hpp
class RVIZ_DEFAULT_PLUGINS_PUBLIC Goal3DNewTool : public Pose3DTool
{
  Q_OBJECT

public:
  Goal3DNewTool();

  ~Goal3DNewTool() override;

  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double z, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;

  rviz_common::properties::StringProperty * topic_property_;
  rviz_common::properties::QosProfileProperty * qos_profile_property_;

  rclcpp::QoS qos_profile_;
};

}  // namespace openbot_rviz

#endif  // RVIZ_DEFAULT_PLUGINS__TOOLS__GOAL_3D_NEW_TOOL_HPP_