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

#include "openbot_ros/urdf_reader.hpp"

#include <string>
#include <vector>

#include "glog/logging.h"
#include "gtest/gtest.h"

#include "openbot_ros/msg_conversion.hpp"
#include "urdf/model.h"

namespace openbot_ros {

std::vector<geometry_msgs::msg::TransformStamped> ReadStaticTransformsFromUrdf(
    const std::string& urdf_filename, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
  urdf::Model model;
  CHECK(model.initFile(urdf_filename));
  std::vector<urdf::LinkSharedPtr> links;
  model.getLinks(links);
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  for (const auto& link : links) {
    if (!link->getParent() || link->parent_joint->type != urdf::Joint::FIXED) {
      continue;
    }

    const urdf::Pose& pose = link->parent_joint->parent_to_joint_origin_transform;
    geometry_msgs::msg::TransformStamped transform;
    // transform.transform =
    //     ToGeometryMsgTransform(openbot::transform::Rigid3d(
    //         Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z),
    //         Eigen::Quaterniond(pose.rotation.w, pose.rotation.x,
    //                            pose.rotation.y, pose.rotation.z)));
    // transform.child_frame_id = link->name;
    // transform.header.frame_id = link->getParent()->name;
    // tf_buffer->setTransform(transform, "urdf", true /* is_static */);
    // transforms.push_back(transform);
  }
  return transforms;
}

}  // namespace openbot_ros
