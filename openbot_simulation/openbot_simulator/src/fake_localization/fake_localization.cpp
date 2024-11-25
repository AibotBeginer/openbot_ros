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

#include "fake_localization/fake_localization.hpp"

namespace openbot_ros {

FakeOdomNode::FakeOdomNode(void)
    : Node("fake_localization_node")
{
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 1);
    particlecloud_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", 1);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            get_node_base_interface(), 
            get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    base_pos_received_ = false; 

    // 声明参数并提供默认值
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("base_frame_id", "base_link");
    this->declare_parameter<std::string>("global_frame_id", "map");
    this->declare_parameter<double>("delta_x", 0.0);
    this->declare_parameter<double>("delta_y", 0.0);
    this->declare_parameter<double>("delta_yaw", 0.0);
    this->declare_parameter<double>("transform_tolerance", 0.1);

    // 获取参数值
    this->get_parameter("odom_frame_id", odom_frame_id_);
    this->get_parameter("base_frame_id", base_frame_id_);
    this->get_parameter("global_frame_id", global_frame_id_);
    this->get_parameter("delta_x", delta_x_);
    this->get_parameter("delta_y", delta_y_);
    this->get_parameter("delta_yaw", delta_yaw_);
    this->get_parameter("transform_tolerance", transform_tolerance_);

    // 打印参数值
    RCLCPP_INFO(this->get_logger(), "odom_frame_id: %s", odom_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "base_frame_id: %s", base_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "global_frame_id: %s", global_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "delta_x: %f", delta_x_);
    RCLCPP_INFO(this->get_logger(), "delta_y: %f", delta_y_);
    RCLCPP_INFO(this->get_logger(), "delta_yaw: %f", delta_yaw_);
    RCLCPP_INFO(this->get_logger(), "transform_tolerance: %f", transform_tolerance_);
    
    particle_cloud_.header.stamp = rclcpp::Clock().now();
    particle_cloud_.header.frame_id = global_frame_id_;
    particle_cloud_.poses.resize(1);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, -delta_yaw_);
    offset_tf_ = tf2::Transform(q, tf2::Vector3(-delta_x_, -delta_y_, 0.0));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    stuff_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "base_pose_ground_truth", qos, std::bind(&FakeOdomNode::stuffFilter, this, std::placeholders::_1));
        
    filter_sub_ = new message_filters::Subscriber<nav_msgs::msg::Odometry>(this, "");
    filter_ = new tf2_ros::MessageFilter<nav_msgs::msg::Odometry>(*filter_sub_, *tf_buffer_, base_frame_id_, 100, 
        this->get_node_logging_interface(), this->get_node_clock_interface());
    filter_->registerCallback([this](auto& msg){ update(msg); });

    // subscription to "2D Pose Estimate" from RViz:
    init_pose_sub_ = new message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped>(this, "initialpose");
    init_pose_filter_ = new tf2_ros::MessageFilter<geometry_msgs::msg::PoseWithCovarianceStamped>(*init_pose_sub_, *tf_buffer_, global_frame_id_, 1, 
        this->get_node_logging_interface(), this->get_node_clock_interface());
    init_pose_filter_->registerCallback([this](auto& msg){ initPoseReceived(msg); });
}

FakeOdomNode::~FakeOdomNode(void)
{
}

void FakeOdomNode::stuffFilter(const nav_msgs::msg::Odometry::ConstPtr& odom_msg)
{
    //we have to do this to force the message filter to wait for transforms
    //from odom_frame_id_ to base_frame_id_ to be available at time odom_msg.header.stamp
    //really, the base_pose_ground_truth should come in with no frame_id b/c it doesn't make sense
    auto stuff_msg = std::make_shared<nav_msgs::msg::Odometry>();
    *stuff_msg = *odom_msg;
    stuff_msg->header.frame_id = odom_frame_id_;
    filter_->add(stuff_msg);
}

void FakeOdomNode::update(const nav_msgs::msg::Odometry::ConstPtr& message)
{
    tf2::Transform txi;
    tf2::convert(message->pose.pose, txi);
    txi = offset_tf_ * txi;

    geometry_msgs::msg::TransformStamped odom_to_map;
    try
    {
        geometry_msgs::msg::TransformStamped txi_inv;
        txi_inv.header.frame_id = base_frame_id_;
        txi_inv.header.stamp = message->header.stamp;
        tf2::convert(txi.inverse(), txi_inv.transform);

        tf_buffer_->transform(txi_inv, odom_to_map, odom_frame_id_);
    }
    catch(tf2::TransformException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to transform to %s from %s: %s\n", odom_frame_id_.c_str(), base_frame_id_.c_str(), e.what());
        return;
    }

    geometry_msgs::msg::TransformStamped trans;
    trans.header.stamp = rclcpp::Clock().now();
    trans.header.frame_id = global_frame_id_;
    trans.child_frame_id = message->header.frame_id;
    tf2::Transform odom_to_map_tf2;
    tf2::convert(odom_to_map.transform, odom_to_map_tf2);
    tf2::Transform odom_to_map_inv = odom_to_map_tf2.inverse();
    tf2::convert(odom_to_map_inv, trans.transform);
    tf_server_->sendTransform(trans);

    tf2::Transform current;
    tf2::convert(message->pose.pose, current);

    //also apply the offset to the pose
    current = offset_tf_ * current;

    geometry_msgs::msg::Transform current_msg;
    tf2::convert(current, current_msg);

    // Publish localized pose
    current_pos_.header = message->header;
    current_pos_.header.frame_id = global_frame_id_;
    tf2::convert(current_msg.rotation, current_pos_.pose.pose.orientation);
    current_pos_.pose.pose.position.x = current_msg.translation.x;
    current_pos_.pose.pose.position.y = current_msg.translation.y;
    current_pos_.pose.pose.position.z = current_msg.translation.z;
    pose_pub_->publish(current_pos_);

    // The particle cloud is the current position. Quite convenient.
    particle_cloud_.header = current_pos_.header;
    particle_cloud_.poses[0] = current_pos_.pose.pose;
    particlecloud_pub_->publish(particle_cloud_);
}

void FakeOdomNode::initPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr& msg)
{
    tf2::Transform pose;
    tf2::convert(msg->pose.pose, pose);

    if (msg->header.frame_id != global_frame_id_){
        RCLCPP_WARN(this->get_logger(), "Frame ID of \"initialpose\" (%s) is different from the global frame %s", 
            msg->header.frame_id.c_str(), global_frame_id_.c_str());
    }

    // set offset so that current pose is set to "initialpose"    
    geometry_msgs::msg::TransformStamped baseInMap;
    try{
        // just get the latest
        baseInMap = tf_buffer_->lookupTransform(base_frame_id_, global_frame_id_, rclcpp::Time(0));
    } catch(tf2::TransformException){
        RCLCPP_WARN(this->get_logger(), "Failed to lookup transform!");
        return;
    }

    tf2::Transform baseInMapTf2;
    tf2::convert(baseInMap.transform, baseInMapTf2);
    tf2::Transform delta = pose * baseInMapTf2;
    offset_tf_ = delta * offset_tf_;
}

}  // namespace openbot_ros

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<openbot_ros::FakeOdomNode>());
    rclcpp::shutdown();
    return 0;
}