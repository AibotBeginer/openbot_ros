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

