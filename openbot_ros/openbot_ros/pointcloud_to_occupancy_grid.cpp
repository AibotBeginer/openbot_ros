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
#include "openbot_ros/pointcloud_to_occupancy_grid.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace openbot_ros
{
  PointCloudToOccupancyGridNode::PointCloudToOccupancyGridNode() : rclcpp::Node("pointcloud_to_occupancy_grid_node")
  {
    // Initialize subscriber and publishers
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_in", 10,
        std::bind(&PointCloudToOccupancyGridNode::pointCloudCallback, this, std::placeholders::_1));

    base_frame_id_ = declare_parameter("base_frame_id", "base_link");

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    pointcloud_camera_frame_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", rclcpp::QoS{1});
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&PointCloudToOccupancyGridNode::transformPointCloud, this));
  }

  void PointCloudToOccupancyGridNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    cloud_in_ = cloud;
    transformPointCloud();
  }

  void PointCloudToOccupancyGridNode::transformPointCloud()
  {
    if (cloud_in_ == nullptr)
    {
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> pc; // input cloud for filtering and transformation
    pcl::fromROSMsg(*cloud_in_, pc);

    geometry_msgs::msg::TransformStamped cloud_to_base_transform_stamped;
    RCLCPP_DEBUG(this->get_logger(),
                "[PointCloudToOccupancyGridNode] looking up Transform: base_frame_id_: %s, \n"
                "Timestamp: [%d.%09d]\n"
                "Frame: '%s'\n",
                base_frame_id_.c_str(),
                cloud_in_->header.stamp.sec,
                cloud_in_->header.stamp.nanosec,
                cloud_in_->header.frame_id.c_str());
    try
    {
      cloud_to_base_transform_stamped = tf2_buffer_->lookupTransform(
          base_frame_id_, cloud_in_->header.frame_id, cloud_in_->header.stamp,
          rclcpp::Duration::from_seconds(1.0));

      RCLCPP_DEBUG(this->get_logger(),
                  "[PointCloudToOccupancyGridNode] Transform lookup successful: base_frame_id_: %s, \n"
                  "Timestamp: [%d.%09d]\n"
                  "Frame: '%s' to '%s'\n"
                  "Translation: [x: %f, y: %f, z: %f]\n"
                  "Rotation (quaternion): [x: %f, y: %f, z: %f, w: %f]",
                  base_frame_id_,
                  cloud_to_base_transform_stamped.header.stamp.sec,
                  cloud_to_base_transform_stamped.header.stamp.nanosec,
                  cloud_to_base_transform_stamped.header.frame_id.c_str(),
                  cloud_to_base_transform_stamped.child_frame_id.c_str(),
                  cloud_to_base_transform_stamped.transform.translation.x,
                  cloud_to_base_transform_stamped.transform.translation.y,
                  cloud_to_base_transform_stamped.transform.translation.z,
                  cloud_to_base_transform_stamped.transform.rotation.x,
                  cloud_to_base_transform_stamped.transform.rotation.y,
                  cloud_to_base_transform_stamped.transform.rotation.z,
                  cloud_to_base_transform_stamped.transform.rotation.w);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }


    sensor_msgs::msg::PointCloud2 cloud_to_publish;
    cloud_to_publish.header.frame_id = base_frame_id_;
    cloud_to_publish.header.stamp = cloud_in_->header.stamp;
    tf2::doTransform (*cloud_in_, cloud_to_publish, cloud_to_base_transform_stamped);
    pointcloud_camera_frame_pub_->publish(cloud_to_publish);
    return ;

    // todo(lzl): pcl_ros crashes
    try
    {
      // Check if transform is identity
      // pointcloud_to_occupancy_grid_node-15] Translation: [x: 0.002709, y: -0.000001, z: 0.000000]
      // [pointcloud_to_occupancy_grid_node-15] Rotation (quaternion): [x: 0.000000, y: 0.000000, z: -0.000540, w: 1.000000]
      // [ERROR] [pointcloud_to_occupancy_grid_node-15]: process has died [pid 1249919, exit code -11, cmd '/home/hello/codebase/projects/jdt_openbot/install/openbot_ros/lib/openbot_ros/pointcloud_to_occupancy_grid_node --ros-args --log-level debug --ros-args --params-file /tmp/launch_params_fzg40bzp -r /cloud_in:=/global_map -r /cloud_out:=/camera_point_cloud'].
      pcl::PointCloud<pcl::PointXYZ> transformed_pc;
      if (cloud_to_base_transform_stamped.transform.translation.x == 0.0 &&
          cloud_to_base_transform_stamped.transform.translation.y == 0.0 &&
          cloud_to_base_transform_stamped.transform.translation.z == 0.0 &&
          cloud_to_base_transform_stamped.transform.rotation.x == 0.0 &&
          cloud_to_base_transform_stamped.transform.rotation.y == 0.0 &&
          cloud_to_base_transform_stamped.transform.rotation.z == 0.0 &&
          cloud_to_base_transform_stamped.transform.rotation.w == 1.0)
      {
        RCLCPP_INFO(this->get_logger(), "Identity transform detected. Skipping point cloud transformation.");
        // Publish the input point cloud directly
        transformed_pc = pc;
      }
      else
      {
        // https://github.com/PointCloudLibrary/pcl/pull/5113
        pcl_ros::transformPointCloud(pc, transformed_pc, cloud_to_base_transform_stamped);
        // Eigen::Isometry3d transform_isometry;
        // tf2::transformToEigen(cloud_to_base_transform_stamped.transform, transform_isometry);

        // // Convert to Eigen::Matrix4f for PCL compatibility
        // Eigen::Matrix4f transform = transform_isometry.matrix().cast<float>();

        // // Transform the point cloud
        // pcl::PointCloud<pcl::PointXYZ> transformed_pc;
        // pcl::transformPointCloud(pc, transformed_pc, transform);
      }

      // Publish the modified copy
      sensor_msgs::msg::PointCloud2 cloud_to_publish;
      pcl::toROSMsg(transformed_pc, cloud_to_publish);
      cloud_to_publish.header.frame_id = base_frame_id_;
      cloud_to_publish.header.stamp = cloud_in_->header.stamp;
      pointcloud_camera_frame_pub_->publish(cloud_to_publish);
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Exception caught: %s", ex.what());
      return;
    }
    catch (...)
    {
      RCLCPP_WARN(this->get_logger(), "Unknown exception caught!");
      return;
    }
  }
} // namespace openbot_ros