
// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/exact_time.h>
// #include <message_filters/sync_policies/approximate_time.h>

// typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud, nav_msgs::Path, nav_msgs::Odometry> exact_policy;

#include "surfel_fusion/surfel_map.h"


#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SurfelMap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

