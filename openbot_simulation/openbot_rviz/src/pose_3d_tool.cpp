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
      : rviz_common::Tool()
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
    setStatus("Click and drag mouse to set position/orientation/height. Hold left mouse to set orientation. Hold left and right mouse to set height");
    state_ = Position; // 初始模式为位置
  }

  void Pose3DTool::deactivate()
  {
    arrow_->getSceneNode()->setVisible(false);
  }

  // 主函数，根据鼠标事件调用不同的设置函数
  int Pose3DTool::processMouseEvent(rviz_common::ViewportMouseEvent &event)
  {
    // 第一次按下左键 = 设置位置
    if (event.leftDown())
    {
      int ret_flags = onSettingPosePosition(event);
      state_ = Orientation;

      return ret_flags;
    }

    // 设置位置后，按住左键 = 设置旋转角度
    if (event.type == QEvent::MouseMove && event.left() && state_ == Orientation)
    {
      int ret_flags = onSettingPoseOrientation(event);
      if (event.right())
      {
        state_ = Height;
      }

      prev_mouse_y_ = event.y;

      return ret_flags;
    }

    // 设置位置+旋转后，按住左键 + 右键 = 设置高度, state 为 Orientation 或 Height
    if (event.type == QEvent::MouseMove && event.right() && event.left() && state_ == Height)
    {
      return onSettingPoseHeight(event);
    }

    if (event.leftUp())
    {
      return onSettingPoseFinished();
    }

    return 0;
  }

  // 位置设置
  int Pose3DTool::onSettingPosePosition(const rviz_common::ViewportMouseEvent &event)
  {
    auto xy_plane_intersection = projection_finder_->getViewportPointProjectionOnXYPlane(
        event.panel->getRenderWindow(), event.x, event.y);

    int flags = 0;
    assert(state_ == Position);
    if (xy_plane_intersection.first)
    {
      arrow_position_ = xy_plane_intersection.second;
      arrow_->setPosition(arrow_position_);

      flags |= Render;
    }
    return flags;
  }

  // 高度设置
  int Pose3DTool::onSettingPoseHeight(const rviz_common::ViewportMouseEvent &event)
  {
    const double z_scale = 50;

    int flags = 0;
    double diff_y = event.y - prev_mouse_y_;
    prev_mouse_y_ = event.y;

    arrow_position_.z -= diff_y / z_scale;


    arrow_->setPosition(arrow_position_);
    arrow_->getSceneNode()->setVisible(true);

    std::stringstream log_stream;
    log_stream << "Mouse Y Difference: " << diff_y 
              << ", Previous Mouse Y: " << prev_mouse_y_
              << ", Updated Arrow Position Z: " << arrow_position_.z;
    std::string log_message = log_stream.str();

    RVIZ_COMMON_LOG_INFO_STREAM(
        "Setting onSettingPoseHeight: pose: " << log_message << ", Position(" << arrow_position_.x << ", " << arrow_position_.y << ", " << arrow_position_.z << ")");
  

    flags |= Render;
    return flags;
  }

  // 方向设置
  int Pose3DTool::onSettingPoseOrientation(const rviz_common::ViewportMouseEvent &event)
  {
    int flags = 0;

    auto xy_plane_intersection = projection_finder_->getViewportPointProjectionOnXYPlane(
        event.panel->getRenderWindow(), event.x, event.y);

    if (state_ == Orientation && xy_plane_intersection.first)
    {
      arrow_angle_ = calculateAngle(xy_plane_intersection.second, arrow_position_);

      // 渲染 arrow
      arrow_->getSceneNode()->setVisible(true);

      // we need base_orient, since the arrow goes along the -z axis by default
      // (for historical reasons)
      Ogre::Quaternion orient_x = Ogre::Quaternion(
          Ogre::Radian(-Ogre::Math::HALF_PI),
          Ogre::Vector3::UNIT_Y);

      arrow_->setOrientation(Ogre::Quaternion(Ogre::Radian(arrow_angle_), Ogre::Vector3::UNIT_Z) * orient_x);

      flags |= Render;
    }
    return flags;
  }

  // 完成设置
  int Pose3DTool::onSettingPoseFinished()
  {
    int flags = 0;

    if (state_ == Orientation || state_ == Height)
    {
      onPoseSet(arrow_position_.x, arrow_position_.y, arrow_position_.z, arrow_angle_);
      flags |= (Finished | Render);
    }
    return flags;
  }

  double Pose3DTool::calculateAngle(Ogre::Vector3 start_point, Ogre::Vector3 end_point)
  {
    return atan2(start_point.y - end_point.y, start_point.x - end_point.x);
  }

  void Pose3DTool::logPose(
      std::string designation, geometry_msgs::msg::Point position,
      geometry_msgs::msg::Quaternion orientation, double angle, std::string frame)
  {
    RVIZ_COMMON_LOG_INFO_STREAM(
        "Setting " << designation << " pose: Frame: " << frame << ", Position(" << position.x << ", " << position.y << ", " << position.z << "), Orientation(" << orientation.x << ", " << orientation.y << ", " << orientation.z << ", " << orientation.w << ") = Angle: " << angle);
  }

  geometry_msgs::msg::Quaternion Pose3DTool::orientationAroundZAxis(double angle)
  {
    auto orientation = geometry_msgs::msg::Quaternion();
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = sin(angle) / (2 * cos(angle / 2));
    orientation.w = cos(angle / 2);
    return orientation;
  }

} // namespace openbot_rviz
