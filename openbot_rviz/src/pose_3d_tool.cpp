#include "openbot_rviz/pose_3d_tool.hpp"

#include <memory>
#include <string>
#include <cmath>

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

namespace openbot_rviz
{

  // ref: https://github.com/ros2/rviz/blob/rolling/rviz_default_plugins/src/rviz_default_plugins/tools/pose/pose_tool.cpp

  Pose3DTool::Pose3DTool()
      : rviz_common::Tool(), angle_(0), z_interval_(0.5), z_scale_(50)
  {
    projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
  }

  Pose3DTool::~Pose3DTool() = default;

  void Pose3DTool::onInitialize()
  {
    arrow_ = std::make_shared<rviz_rendering::Arrow>(
        scene_manager_, nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
    arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
    arrow_->getSceneNode()->setVisible(false);
  }

  void Pose3DTool::activate()
  {
    setStatus("Click and drag mouse to set position/orientation/height.");
    state_ = Position;
  }

  void Pose3DTool::deactivate()
  {
    arrow_->getSceneNode()->setVisible(false);
    arrow_array_.clear();
  }

  int Pose3DTool::processMouseEvent(rviz_common::ViewportMouseEvent &event)
  {
    auto point_projection_on_xy_plane = projection_finder_->getViewportPointProjectionOnXYPlane(
        event.panel->getRenderWindow(), event.x, event.y);

    if (event.leftDown())
    {
      return processMouseLeftButtonPressed(point_projection_on_xy_plane);
    }
    else if (event.type == QEvent::MouseMove && event.left())
    {
      // 重要：按右键进入高度设置模式
      if (event.right() && state_ == Orientation)
      {
        state_ = Height;
        prev_y_ = event.y;
        init_z_ = arrow_position_.z; // Store initial height
      }
      return processMouseMoved(event, point_projection_on_xy_plane);
    }
    else if (event.leftUp())
    {
      return processMouseLeftButtonReleased();
    }

    return 0;
  }

  int Pose3DTool::processMouseMoved(rviz_common::ViewportMouseEvent &event, std::pair<bool, Ogre::Vector3> xy_plane_intersection)
  {
    int flags = 0;
    if (state_ == Orientation && xy_plane_intersection.first)
    {
      angle_ = calculateAngle(xy_plane_intersection.second, arrow_position_);
      makeArrowVisibleAndSetOrientation(angle_);
      flags |= Render;
    }
    else if (state_ == Height)
    {
      // Implement height adjustment logic
      double delta_y = event.y - prev_y_;
      arrow_position_.z = init_z_ - (delta_y / z_scale_);
      arrow_->setPosition(arrow_position_);
      prev_y_ = event.y;
      flags |= Render;
    }
    return flags;
  }

  int Pose3DTool::processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection)
  {
    int flags = 0;
    if (xy_plane_intersection.first && state_ == Position)
    {
      arrow_position_ = xy_plane_intersection.second;
      arrow_->setPosition(arrow_position_);
      state_ = Orientation;
      flags |= Render;
    }
    return flags;
  }

  void Pose3DTool::makeArrowVisibleAndSetOrientation(double angle)
  {
    arrow_->getSceneNode()->setVisible(true);
    Ogre::Quaternion orient_x(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);
    arrow_->setOrientation(Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z) * orient_x);
  }

  int Pose3DTool::processMouseLeftButtonReleased()
  {
    int flags = 0;
    if (state_ == Orientation || state_ == Height)
    {
      onPoseSet(arrow_position_.x, arrow_position_.y, arrow_position_.z, angle_);
      flags |= (Finished | Render);
    }
    return flags;
  }

  double Pose3DTool::calculateAngle(Ogre::Vector3 start_point, Ogre::Vector3 end_point)
  {
    return atan2(start_point.y - end_point.y, start_point.x - end_point.x);
  }

  geometry_msgs::msg::Quaternion Pose3DTool::orientationAroundZAxis(double angle)
  {
    geometry_msgs::msg::Quaternion orientation;
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = sin(angle / 2);
    orientation.w = cos(angle / 2);
    return orientation;
  }

  void Pose3DTool::logPose(
      std::string designation, geometry_msgs::msg::Point position,
      geometry_msgs::msg::Quaternion orientation, double angle, std::string frame)
  {
    RVIZ_COMMON_LOG_INFO_STREAM(
        "Setting " << designation << " pose: Frame: " << frame << ", Position(" << position.x << ", " << position.y << ", " << position.z << "), Orientation(" << orientation.x << ", " << orientation.y << ", " << orientation.z << ", " << orientation.w << ") = Angle: " << angle);
  }

} // namespace openbot_rviz
