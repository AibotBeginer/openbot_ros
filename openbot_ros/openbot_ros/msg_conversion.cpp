/*
 * Copyright 2024 The Cartographer Authors
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

#include "openbot_ros/msg_conversion.hpp"

#include <cmath>

#include "glog/logging.h"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace {

// Sizes of PCL point types have to be 4n floats for alignment, as described in
// http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php
struct PointXYZT 
{
  float x;
  float y;
  float z;
  float time;
};

struct PointXYZIT 
{
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  float unused_padding[2];
};

}  // namespace

namespace openbot_ros {

// Eigen::Vector3d ToEigen(const geometry_msgs::msg::Vector3& vector3) 
// {
//   return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
// }

// Eigen::Quaterniond ToEigen(const geometry_msgs::msg::Quaternion& quaternion) 
// {
//   return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
// }

// geometry_msgs::msg::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d) 
// {
//   geometry_msgs::msg::Point point;
//   point.x = vector3d.x();
//   point.y = vector3d.y();
//   point.z = vector3d.z();
//   return point;
// }


}  // namespace openbot_ros

