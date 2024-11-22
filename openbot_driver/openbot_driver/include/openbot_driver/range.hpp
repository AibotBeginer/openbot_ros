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

#ifndef OPENBOT_DRIVER_RANGE_HPP
#define OPENBOT_DRIVER_RANGE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace openbot_ros {

class Range
{
public:
    Range(rclcpp::Node* node):
        node_handle(node)
    {
        range_msg_sonar.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        range_msg_sonar.field_of_view = 0.87;   // rad,50degree
        range_msg_sonar.min_range = 0.2;
        range_msg_sonar.max_range = 1.5;
        pub_sonar_left_front = node_handle->create_publisher<sensor_msgs::msg::Range>("sonar_left_front", 10);
        pub_sonar_right_front = node_handle->create_publisher<sensor_msgs::msg::Range>("sonar_right_front", 10);
        pub_sonar_left_back = node_handle->create_publisher<sensor_msgs::msg::Range>("sonar_left_back", 10);
        pub_sonar_right_back = node_handle->create_publisher<sensor_msgs::msg::Range>("sonar_right_back", 10);

        range_msg_ir.radiation_type = sensor_msgs::msg::Range::INFRARED;
        range_msg_ir.field_of_view = 0.17;   // 10degree
        range_msg_ir.min_range = 0.2;
        range_msg_ir.max_range = 3.0;
        pub_ir_left = node_handle->create_publisher<sensor_msgs::msg::Range>("cliff_left", 10);
        pub_ir_right = node_handle->create_publisher<sensor_msgs::msg::Range>("cliff_right", 10);
    }
    /**
     * @brief fillAndPubSonarmsg
     * @param data
     * @return
     */
    bool fillAndPubSonarmsg(const std::vector<unsigned short > &data)
    {
        //
        range_msg_sonar.header.stamp = node_handle->get_clock()->now();

        range_msg_sonar.header.frame_id = "sonar_left_front";
        range_msg_sonar.range = data[0]/1000.0;
        range_msg_sonar.range = std::max(range_msg_sonar.min_range, std::min(range_msg_sonar.range, range_msg_sonar.max_range));
        pub_sonar_left_front->publish(range_msg_sonar);
        //
        range_msg_sonar.header.frame_id = "sonar_right_front";
        range_msg_sonar.range = data[1]/1000.0;
        range_msg_sonar.range = std::max(range_msg_sonar.min_range, std::min(range_msg_sonar.range, range_msg_sonar.max_range));
        pub_sonar_right_front->publish(range_msg_sonar);
        //
        range_msg_sonar.header.frame_id = "sonar_left_back";
        range_msg_sonar.range = data[2]/1000.0;
        range_msg_sonar.range = std::max(range_msg_sonar.min_range, std::min(range_msg_sonar.range, range_msg_sonar.max_range));
        pub_sonar_left_back->publish(range_msg_sonar);
        //
        range_msg_sonar.header.frame_id = "sonar_right_back";
        range_msg_sonar.range = data[3]/1000.0;
        range_msg_sonar.range = std::max(range_msg_sonar.min_range, std::min(range_msg_sonar.range, range_msg_sonar.max_range));
        pub_sonar_right_back->publish(range_msg_sonar);
        return true;
    }
    /**
     * @brief fillAndPubIrmsg
     * @return
     */
    bool fillAndPubIrmsg(const std::vector<unsigned short> &data)
    {
        range_msg_ir.header.stamp = node_handle->get_clock()->now();

        range_msg_ir.header.frame_id = "cliff_left";
        range_msg_ir.range = data[0]/100.0;
        range_msg_ir.range = std::max(range_msg_ir.min_range, std::min(range_msg_ir.range, range_msg_ir.max_range));
        pub_ir_left->publish(range_msg_ir);

        range_msg_ir.header.frame_id = "cliff_right";
        range_msg_ir.range = data[2]/100.0;
        range_msg_ir.range = std::max(range_msg_ir.min_range, std::min(range_msg_ir.range, range_msg_ir.max_range));
        pub_ir_right->publish(range_msg_ir);
//        RCLCPP_INFO(node_handle->get_logger(), "cliff: %d %d %d %d %d %d", data[0], data[1], data[2], data[3], data[4], data[5]);
        return true;
    }
private:
    rclcpp::Node*                             node_handle;
    sensor_msgs::msg::Range range_msg_sonar, range_msg_ir;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr   pub_sonar_left_front, pub_sonar_right_front, pub_sonar_left_back, pub_sonar_right_back;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr   pub_ir_left, pub_ir_right;
};

}

#endif // OPENBOT_DRIVER_RANGE_HPP
