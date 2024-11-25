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

#ifndef OPENBOT_DRIVER_BASEINFO_HPP
#define OPENBOT_DRIVER_BASEINFO_HPP

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "openbot_driver/msg/base_info.hpp"

namespace openbot_ros {

class BaseInfo
{
public:
    BaseInfo(rclcpp::Node& node)
    {
        base_info_pub_ = node.create_publisher<openbot_driver::msg::BaseInfo>("base_info",10);
    }
    /**
     * @brief updateData
     * @param data
     * @return
     */
    bool updateData(const std::vector<unsigned char > &data)
    {
        static uint16_t lastmotorleft = 0,lastmotorright = 0;
        static uint8_t stopsta = 0;
        static bool stopinit = false;
        static uint8_t chargingState_last = 0;

        base_info_.connected         = true;
        base_info_.cliff_left        = (((uint16_t)(data[1] << 8) | data[0]) & 0x01)? true: false;
        base_info_.cliff_front_left  = (((uint16_t)(data[1] << 8) | data[0]) & 0x02)? true: false;
        base_info_.cliff_front_right = (((uint16_t)(data[1] << 8) | data[0]) & 0x04)? true: false;
        base_info_.cliff_right       = (((uint16_t)(data[1] << 8) | data[0]) & 0x08)? true: false;

        base_info_.bump_state_left   = (((uint16_t)(data[1] << 8) | data[0]) & 0x10)? true: false;
        base_info_.bump_state_right  = (((uint16_t)(data[1] << 8) | data[0]) & 0x20)? true: false;
        base_info_.charging_vol = ((uint16_t)(data[3] << 8) | data[2]);
        base_info_.charging_state = data[4] & 0x0F;
        if(chargingState_last!=base_info_.charging_state)
            RCLCPP_INFO(rclcpp::get_logger("BaseInfo"),"baseinfo chargingstate is %d (0-nocharge, 2-dock_charging, 4-ac_charging)", base_info_.charging_state);
        chargingState_last = base_info_.charging_state;
        base_info_.charge_ctrl = (data[4] >> 4) & 0x0F;
        base_info_.base_device_state = data[5];
        base_info_.motor_left_err_code = ((uint16_t)(data[7] << 8) | data[6]);
        base_info_.motor_right_err_code = ((uint16_t)(data[9] << 8) | data[8]);
        base_info_.bump_enable = (((uint16_t)(data[11] << 8) | data[10]) & 0x02)? true: false;
        base_info_.cliff_enable = (((uint16_t)(data[11] << 8) | data[10]) & 0x01)? true: false;

        if((base_info_.base_device_state&0x01) != stopsta || stopinit == false)
        {
            stopinit = true;
            RCLCPP_INFO(rclcpp::get_logger("BaseInfo"),"stop change,now is %d", (base_info_.base_device_state&0x01));
        }
        stopsta = base_info_.base_device_state&0x01;
//        dismotor = base_info_.base_device_state&0x10;

        if((base_info_.motor_left_err_code!=lastmotorleft) || (base_info_.motor_right_err_code!=lastmotorright))
        {
            RCLCPP_INFO(rclcpp::get_logger("BaseInfo"),"motor_left_err_code:%d,motor_right_err_code:%d",base_info_.motor_left_err_code, base_info_.motor_right_err_code);
        }
        lastmotorleft = base_info_.motor_left_err_code;
        lastmotorright = base_info_.motor_right_err_code;

        base_info_pub_->publish(base_info_);

        return true;
    }
    bool getBaseInfo(openbot_driver::msg::BaseInfo &data)
    {
        memcpy((char*)&data, (char*)&base_info_, sizeof(base_info_));
        return true;
    }
private:
    openbot_driver::msg::BaseInfo base_info_;
    rclcpp::Publisher<openbot_driver::msg::BaseInfo>::SharedPtr base_info_pub_;
};

} // namespace openbot_ros

#endif // OPENBOT_DRIVER_BASEINFO_HPP
