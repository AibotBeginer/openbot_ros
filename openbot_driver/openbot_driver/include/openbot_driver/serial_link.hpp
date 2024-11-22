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

#ifndef OPENBOT_DRIVER_SERIAL_LINK_HPP
#define OPENBOT_DRIVER_SERIAL_LINK_HPP

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <iostream>
#include <iomanip>
#include <thread>
#include <memory>

#define LINK_MSG_HDR1         0xaa
#define LINK_MSG_HDR2         0x55
#define LINK_MSG_VER          0x0a
#define LINK_MSG_MAX_LENTH    256
#define LINK_NO_DATA_LENTH    9
/**
 * @struct LinkStruct_TypeDef
 * @brief 控制板数据协议格式
*/
typedef struct
{
    uint8_t hdr1;///<包头STX0 AA
    uint8_t hdr2;///<包头STX1 55
    uint8_t lenth;///<数据长度len
    uint8_t ver;///<协议版本ver
    uint8_t ack;///<响应ack
    uint8_t msg_id;///<消息msgID
    uint16_t seq;///<序列号SEQ
    uint8_t payload[LINK_MSG_MAX_LENTH];///<消息内容PAYLOAD
    uint8_t sum_chk;///<数据校验DATA_SUM
}LinkStruct_TypeDef;

/**
 * @class SerialLink
 * @brief 串口相关类
*/
class SerialLink
{
public:
    /** @brief 枚举类型，控制命令ID*/
    enum
    {
        ENCODER=30,///<编码器脉冲值
        IMU=31,///<惯性传感器数据
        BASE_INFO=32,///<基本数据
        DOCK_IR=33, ///<红外数据
        SOFTVER=34, ///<当前软件硬件版本
        ADC_VALUE=35, ///<AD值
        SONAR=36, ///<超声传感器
        BAT=37///<电源信息
    }MSGID;
    enum
    {
        VEL_CMD=10, ///<速度指令
        POS_CMD=11,///<位置指令
        MOTOR_CMD=12, ///<电机控制
        CHG_CMD=13, ///<对接控制
        STOP_CMD=14,///<急停锁轴
        SHUTDOWN_CMD=15,///<软关机
        UPDATE_CMD=16, ///<固件更新
        REQ_VERSION=17,///<请求当前软件版本
        FOG_LEVEL=19,///<加湿器雾量控制
        SAFETY_SENSORS=20,///<传感器使能设置
        CLEAR_MOTOR_ALARM = 21, ///<清除电机异常
        BATTERY_CHARGE_CTRL = 22,///<电池充电控制
        LED_STATE = 30,///<灯带控制
        BUMP_SAFETY_SENSOR=23,///<碰撞使能控制
        CLIFF_SAFETY_SENSOR=24,///<跌落使能控制
        CLIFF_THRESHOLD_SENSOR=25, ///<跌落阈值设置
        SONAR_SAFETY_SENSOR=26,///<超声使能控制
        BATTERY_PARAM_CTRL = 31,///<电池参数设置
        MOTORCURRENT_SET = 32,///<电机锁轴电流限制
        IMU_CALIBRATION_CMD = 33,///<imu校准命令
        RESET_CMD = 34///<整机重启指令
    }CTRL_ENUM;
    /** @brief 枚举类型，协议状态机 */
    enum
    {
        IDIE=0,///<空闲
        GOT_HDR1,///<包头1
        GOT_HDR2,///<包头2
        GOT_LENTH,///<数据长度
        GOT_VER,///<协议版本
        GOT_ACK,///<响应
        GOT_SEQ1,///<序号1
        GOT_SEQ2,///<序号2
        GOT_MSG_ID,///<消息ID
        GOT_PAYLOAD,///<消息内容
        GOT_BADCHK///<消息校验和
    }PARAS_STATE;

    SerialLink(std::string pathname, uint32_t rate): PARAS_STATE(IDIE), path_(pathname), baudrate_(rate),  packet_idx(0), recv_num(0){}

    ~SerialLink() {
        if (isOpen() != -1) {
            closePort();
        }
    }
    uint8_t msg_received;///<消息接收完成标志

    int openPort();
    int closePort();

    std::string getDevPath() { return path_; }
    int isOpen() { return fd_ >= 0 ? 0 : -1; }

    inline int readByte(uint8_t *data) { return read(fd_, data, 1); }
    inline int writeData(const uint8_t *buf, uint16_t length) { return write(fd_, buf, length); }

    void MsgRecived(LinkStruct_TypeDef &msg);///<信息接收
    uint8_t SumCheck(uint8_t *buf, uint8_t lenth);///<校验和计算
    bool sendCmd(LinkStruct_TypeDef &sendmsg, std::shared_ptr<SerialLink> port_);///<发送命令
    bool Paras_char(uint8_t c, LinkStruct_TypeDef &msg);///<数据解析

    void Bufinit()
    {
        PARAS_STATE = IDIE;
        packet_idx = 0;
        recv_num = 0;
        return;
    }

private:
    std::string path_;
    uint32_t baudrate_;
    int fd_;
    uint8_t packet_idx;///<消息接收payload个数
    uint8_t recv_num;///<消息接收数据个数
    uint8_t buf[320];///<接收数据缓存区
};

#endif // OPENBOT_DRIVER_SERIAL_LINK_HPP
