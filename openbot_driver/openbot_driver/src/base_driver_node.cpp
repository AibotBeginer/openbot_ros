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


#include "rclcpp/rclcpp.hpp"

#define BACKWARD_HAS_DW 1

#include "openbot_driver/backward.hpp"
#include "openbot_driver/base_driver.hpp"

namespace backward{
    backward::SignalHandling sh;
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<openbot_ros::Base_Driver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}