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

namespace rviz_rendering
{
  class Arrow;
}

namespace openbot_rviz
{

  // ref: default pose 2d tool https://github.com/ros2/rviz/blob/rolling/rviz_default_plugins/include/rviz_default_plugins/tools/pose/pose_tool.hpp
  class RVIZ_DEFAULT_PLUGINS_PUBLIC Pose3DTool : public rviz_common::Tool
  {
  public:
    Pose3DTool();
    ~Pose3DTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    int processMouseEvent(rviz_common::ViewportMouseEvent &event) override;

  protected:
    virtual void onPoseSet(double x, double y, double z, double theta) = 0;

    geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle);

    void logPose(std::string designation, geometry_msgs::msg::Point position,
                 geometry_msgs::msg::Quaternion orientation, double angle, std::string frame);

    enum PoseSettingState
    {
      Position,
      Orientation,
      Height
    };
    PoseSettingState state_;

    std::shared_ptr<rviz_rendering::Arrow> arrow_;
    Ogre::Vector3 arrow_position_;
    double arrow_angle_;

    double prev_mouse_y_;

    std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

  private:
    // 位置设置：根据鼠标事件在XY平面上设置箭头位置
    int onSettingPosePosition(const rviz_common::ViewportMouseEvent &event);

    // 高度设置：根据鼠标事件调整箭头的高度
    int onSettingPoseHeight(const rviz_common::ViewportMouseEvent &event);

    // 方向设置：根据鼠标事件设置箭头的方向
    int onSettingPoseOrientation(const rviz_common::ViewportMouseEvent &event);

    // 完成设置：保存箭头的最终位置和方向
    int onSettingPoseFinished();

    double calculateAngle(Ogre::Vector3 start_point, Ogre::Vector3 end_point);
  };

} // namespace openbot_rviz

#endif // RVIZ_DEFAULT_PLUGINS__TOOLS__POSE_3D_TOOL_HPP_
