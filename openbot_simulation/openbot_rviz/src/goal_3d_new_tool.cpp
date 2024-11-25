#include "openbot_rviz/goal_3d_new_tool.hpp"

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"

namespace openbot_rviz
{

  // ref: https://github.com/ros2/rviz/blob/rolling/rviz_default_plugins/src/rviz_default_plugins/tools/goal_pose/goal_tool.cpp
  Goal3DNewTool::Goal3DNewTool()
      : Pose3DTool(), qos_profile_(5)
  {
    shortcut_key_ = 'g';

    topic_property_ = new rviz_common::properties::StringProperty(
        "Topic", "goal_pose",
        "The topic on which to publish goals.",
        getPropertyContainer(), SLOT(updateTopic()), this);

    qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
        topic_property_, qos_profile_);
  }

  Goal3DNewTool::~Goal3DNewTool() = default;

  void Goal3DNewTool::onInitialize()
  {
    Pose3DTool::onInitialize();
    qos_profile_property_->initialize(
        [this](rclcpp::QoS profile)
        { this->qos_profile_ = profile; });
    setName("3D Goal Pose");
    updateTopic();
  }

  void Goal3DNewTool::updateTopic()
  {
    rclcpp::Node::SharedPtr raw_node =
        context_->getRosNodeAbstraction().lock()->get_raw_node();
    publisher_ = raw_node->template create_publisher<geometry_msgs::msg::PoseStamped>(
        topic_property_->getStdString(), qos_profile_);
    clock_ = raw_node->get_clock();
  }

  void Goal3DNewTool::onPoseSet(double x, double y, double z, double theta)
  {
    std::string fixed_frame = context_->getFixedFrame().toStdString();

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = clock_->now();
    goal.header.frame_id = fixed_frame;

    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;

    goal.pose.orientation = orientationAroundZAxis(theta);

    logPose("3d goal", goal.pose.position, goal.pose.orientation, theta, fixed_frame);

    publisher_->publish(goal);
  }

} // namespace openbot_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(openbot_rviz::Goal3DNewTool, rviz_common::Tool)