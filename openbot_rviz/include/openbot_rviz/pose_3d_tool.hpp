#ifndef RVIZ_DEFAULT_PLUGINS__TOOLS__POSE_3D_TOOL_HPP_
#define RVIZ_DEFAULT_PLUGINS__TOOLS__POSE_3D_TOOL_HPP_

#include <memory>
#include <string>
#include <vector>

#include <OgreVector3.h>
#include <QCursor>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_rendering {
class Arrow;
}

namespace openbot_rviz
{

// ref: default pose 2d tool https://github.com/ros2/rviz/blob/rolling/rviz_default_plugins/include/rviz_default_plugins/tools/pose/pose_tool.hpp
class RVIZ_DEFAULT_PLUGINS_PUBLIC Pose3DTool : public rviz_common::Tool {
public:
  Pose3DTool();
  ~Pose3DTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

protected:
  virtual void onPoseSet(double x, double y, double z, double theta) = 0;

  geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle);

  void logPose(std::string designation, geometry_msgs::msg::Point position,
               geometry_msgs::msg::Quaternion orientation, double angle, std::string frame);

  std::shared_ptr<rviz_rendering::Arrow> arrow_;
  std::vector<std::shared_ptr<rviz_rendering::Arrow>> arrow_array_;

  enum State { Position, Orientation, Height };
  State state_;
  double angle_;

  Ogre::Vector3 arrow_position_;
  double init_z_;
  double prev_y_;
  double prev_angle_;
  double z_interval_;
  double z_scale_;

  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

private:
  int processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
  int processMouseMoved(rviz_common::ViewportMouseEvent &event, std::pair<bool, Ogre::Vector3> xy_plane_intersection);
  int processMouseLeftButtonReleased();
  void makeArrowVisibleAndSetOrientation(double angle);
  double calculateAngle(Ogre::Vector3 start_point, Ogre::Vector3 end_point);
};

}  // namespace openbot_rviz

#endif  // RVIZ_DEFAULT_PLUGINS__TOOLS__POSE_3D_TOOL_HPP_
