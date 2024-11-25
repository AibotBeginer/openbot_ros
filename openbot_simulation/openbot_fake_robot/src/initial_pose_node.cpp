#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "rclcpp/parameter.hpp"

class InitialPosePublisher : public rclcpp::Node
{
public:
  InitialPosePublisher() : Node("initial_pose_publisher")
  {
    // Declare parameters for position and orientation
    this->declare_parameter("x", 0.0);
    this->declare_parameter("y", 0.0);
    this->declare_parameter("z", 0.0);
    this->declare_parameter("qx", 0.0);
    this->declare_parameter("qy", 0.0);
    this->declare_parameter("qz", 0.0);
    this->declare_parameter("qw", 1.0);

    // Get the parameter values
    double x = this->get_parameter("x").as_double();
    double y = this->get_parameter("y").as_double();
    double z = this->get_parameter("z").as_double();
    double qx = this->get_parameter("qx").as_double();
    double qy = this->get_parameter("qy").as_double();
    double qz = this->get_parameter("qz").as_double();
    double qw = this->get_parameter("qw").as_double();

    // Create the PoseStamped message
    pose_msg_.header.stamp = this->get_clock()->now();
    pose_msg_.header.frame_id = "map";  // Use the map frame
    pose_msg_.pose.position.x = x;
    pose_msg_.pose.position.y = y;
    pose_msg_.pose.position.z = z;
    pose_msg_.pose.orientation.x = qx;
    pose_msg_.pose.orientation.y = qy;
    pose_msg_.pose.orientation.z = qz;
    pose_msg_.pose.orientation.w = qw;

    // Create a publisher to the /initialpose topic
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/initialpose", 10);

    // Publish the initial pose
    pose_publisher_->publish(pose_msg_);
    RCLCPP_INFO(this->get_logger(), "Initial pose published.");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  geometry_msgs::msg::PoseStamped pose_msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitialPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
