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


#pragma once

/**

  @mainpage

  @htmlinclude manifest.html

  @b odom_localization Takes in ground truth pose information for a robot
  base (e.g., from a simulator or motion capture system) and republishes
  it as if a localization system were in use.

  <hr>

  @section usage Usage
  @verbatim
  $ fake_localization
  @endverbatim

  <hr>

  @section topic ROS topics

  Subscribes to (name/type):
  - @b "base_pose_ground_truth" nav_msgs/Odometry : robot's odometric pose.  Only the position information is used (velocity is ignored).

  Publishes to (name / type):
  - @b "amcl_pose" geometry_msgs/PoseWithCovarianceStamped : robot's estimated pose in the map, with covariance
  - @b "particlecloud" geometry_msgs/PoseArray : fake set of poses being maintained by the filter (one paricle only).

  <hr>

  @section parameters ROS parameters

  - "~odom_frame_id" (string) : The odometry frame to be used, default: "odom"

 **/

#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <angles/angles.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>




namespace openbot_ros {

class FakeOdomNode : public rclcpp::Node
{
public:
    FakeOdomNode();
    ~FakeOdomNode();

private:
    // ROS Publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlecloud_pub_{nullptr};

    // ROS Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr stuff_sub_{nullptr};

    // tf2
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_server_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
 
    // message_filters
    tf2_ros::MessageFilter<geometry_msgs::msg::PoseWithCovarianceStamped>* init_pose_filter_{nullptr};
    tf2_ros::MessageFilter<nav_msgs::msg::Odometry>* filter_{nullptr};
    
    message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped>* init_pose_sub_{nullptr};
    message_filters::Subscriber<nav_msgs::msg::Odometry>* filter_sub_{nullptr};
    

    double delta_x_;
    double delta_y_;
    double delta_yaw_;
    bool base_pos_received_;
    double transform_tolerance_;

    nav_msgs::msg::Odometry base_pos_msg_;
    geometry_msgs::msg::PoseArray particle_cloud_;
    geometry_msgs::msg::PoseWithCovarianceStamped current_pos_;
    tf2::Transform offset_tf_;

    //parameter for what odom to use
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string global_frame_id_;

public:
    void stuffFilter(const nav_msgs::msg::Odometry::ConstPtr& odom_msg);

    void update(const nav_msgs::msg::Odometry::ConstPtr& message);

    void initPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr& msg);
};

}  // namespace openbot_ros