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

#ifndef OPENBOT_DRIVER_BATTERY_HPP
#define OPENBOT_DRIVER_BATTERY_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace openbot_ros {

class Battery
{
    enum
    {
        OVER_VOL = 1,///过压
        LOW_VOL = 2,///<欠压
        OVER_TEMP=4,///<过温
        LOW_TEMP=8,///<低温
        OVER_CURRENT=16, ///<过流
        SHORT_CIRCUIT=32, ///<短路
        IC_ERR=64, ///<IC错误
        SOFT_MOSLOCK=128 ///<软件锁定MOS
    }BATERRCODE;

public:
    Battery(rclcpp::Node& node)
    {
        battery_.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
        battery_.power_supply_status =sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        battery_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;

        pub_ = node.create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
    }

    /**
     * @brief fillAndPubMsg
     * @param data
     * @param charging_state
     */
    void fillAndPubMsg(const std::vector<unsigned char >& data, uint8_t charging_state)
    {
        uint8_t  tmp_data = 0;
        static uint32_t charge_state_cnt = 0;
        static uint32_t current_state_cnt = 0;
        static uint8_t charge_state_last = 0;

        if(charging_state > 0)
        {
            charge_state_cnt++;
            if(battery_.current < 0.0001)
            {
                current_state_cnt++;
            }
            else
            {
                current_state_cnt = 0;
            }
        }
        else
        {
            current_state_cnt = 0;
            charge_state_cnt = 0;
        }

        battery_.voltage = ((uint16_t)(data[1] << 8) | data[0]) * 0.01;
        battery_.current = (( int16_t)(data[3] << 8) | data[2]) * 0.01;
        battery_.capacity = ((uint16_t)(data[5] << 8) | data[4]) * 0.01;
        battery_.design_capacity = ((uint16_t)(data[7] << 8) | data[6]) * 0.01;
        temp1  = ((uint16_t)(data[9] << 8) | data[8]) * 0.1;
        temp2  = ((uint16_t)(data[11] << 8) | data[10]) * 0.1;
        battery_.temperature = temp1;
        batPretectState = ((uint16_t)(data[15] << 8) | data[14]);
        tmp_data =  ((uint16_t)(data[13] << 8) | data[12]) * 1.0;

        battery_.percentage =  std::min((uint16_t)(tmp_data), (uint16_t)100);
        if(battery_.percentage < 0.001) battery_.percentage = 1.0;
        if((charging_state > 0) && (charge_state_cnt >= 30)) //接触，并且持续30个周期以上(30秒)
        {
            if((current_state_cnt >=30) && (battery_.current < 0.0001) && (battery_.percentage == 100.0)) //0或负值 充满
            {
                battery_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
            }
            else//充电中
            {
                battery_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
            }
        }
        else //脱离接触
        {
            battery_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        }
        if(charge_state_last != battery_.power_supply_status)
        {
            RCLCPP_INFO(rclcpp::get_logger("Battery"),"CHARGE_STATE CHANGE!!! %d (1-CHARGING,2-DISCHARGING,4-FULL)", battery_.power_supply_status);
        }
        charge_state_last = battery_.power_supply_status;

        tmp_data = 0;
        //todo map<state, string>
        if((batPretectState&0x0001) == 0x0001)
        {
//            RCLCPP_INFO_THROTTLE(rclcpp::get_logger("Battery"),10,"BATTERY SINGLE OVER VOL");
            tmp_data |= OVER_VOL;
        }
        if((batPretectState&0x0004) == 0x0004)
        {
//            RCLCPP_INFO_THROTTLE(rclcpp::get_logger("Battery"),10,"BATTERY  WHOLE OVER VOL");
            tmp_data |= OVER_VOL;
        }
        if((batPretectState&0x0002) == 0x0002)
        {
//            RCLCPP_INFO_THROTTLE(rclcpp::get_logger("Battery"),10,"BATTERY  SINGLE LOW VOL");
            tmp_data |= LOW_VOL;
        }
        if((batPretectState&0x0008) == 0x0008)
        {
            tmp_data |= LOW_VOL;
        }
        if((batPretectState&0x0010) == 0x0010)
        {
            tmp_data |= OVER_TEMP;
        }
        if((batPretectState&0x0040) == 0x0040)
        {
            tmp_data |= OVER_TEMP;
        }
        if((batPretectState&0x0020) == 0x0020)
        {
            tmp_data |= LOW_TEMP;
        }
        if((batPretectState&0x0080) == 0x0080)
        {
            tmp_data |= LOW_TEMP;
        }
        if((batPretectState&0x0100) == 0x0100)
        {
            tmp_data |= OVER_CURRENT;
        }
        if((batPretectState&0x0200) == 0x0200)
        {
            tmp_data |= OVER_CURRENT;
        }
        if((batPretectState&0x0400) == 0x0400)
        {
            tmp_data |= SHORT_CIRCUIT;
        }
        if((batPretectState&0x0800) == 0x0800)
        {
            tmp_data |= IC_ERR;
        }
        if((batPretectState&0x1000) == 0x1000)
        {
            tmp_data |= SOFT_MOSLOCK;
        }
        battery_.power_supply_health = tmp_data;

        if(batPretectState)
            battery_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
        else
            battery_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        //unstruct 反序列化

        pub_->publish(battery_);
    }
private:
    double_t temp1, temp2;///<临时变量
    uint16_t batPretectState;///<电池保护状态
    sensor_msgs::msg::BatteryState battery_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_;
};

}  // namespace openbot_ros

#endif // OPENBOT_DRIVER_BATTERY_HPP
