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

#include "openbot_driver/serial_link.hpp"

int SerialLink::openPort() 
{
    struct termios termios_opt;
    const char* addr = path_.c_str();
    fd_ = open(addr, O_RDWR | O_NOCTTY| O_NDELAY);

    if (fd_ == -1)
        return -1;

    if ((fcntl(fd_, F_SETFL, 0)) < 0) {
        return -1;
    }
    if (tcgetattr(fd_, &termios_opt) != 0) {
        return -1;
    }

    cfmakeraw(&termios_opt);
    //set speed
    switch( baudrate_ )        //设置波特率
    {
    case 2400:
        cfsetispeed(&termios_opt, B2400);          //设置输入速度
        cfsetospeed(&termios_opt, B2400);           //设置输出速度
        break;
    case 4800:
        cfsetispeed(&termios_opt, B4800);
        cfsetospeed(&termios_opt, B4800);
        break;
    case 9600:
        cfsetispeed(&termios_opt, B9600);
        cfsetospeed(&termios_opt, B9600);
        break;
    case 19200:
        cfsetispeed(&termios_opt, B19200);
        cfsetospeed(&termios_opt, B19200);
        break;
    case 38400:
        cfsetispeed(&termios_opt, B38400);
        cfsetospeed(&termios_opt, B38400);
        break;
    case 57600:
        cfsetispeed(&termios_opt, B57600);
        cfsetospeed(&termios_opt, B57600);
        break;
    case 115200:
        cfsetispeed(&termios_opt, B115200);
        cfsetospeed(&termios_opt, B115200);
        break;
    default:
        RCLCPP_INFO(rclcpp::get_logger("SerialLink::openPort"), "baudrate_ not support, set default 115200");
        cfsetispeed(&termios_opt, B115200);
        cfsetospeed(&termios_opt, B115200);
        break;
    }

    //set databits
    termios_opt.c_cflag |= (CLOCAL|CREAD);
    termios_opt.c_cflag &= ~CSIZE;
    termios_opt.c_cflag |= CS8;

    //set parity
    termios_opt.c_cflag &= ~PARENB;
    termios_opt.c_iflag &= ~INPCK;

    //set stopbits
    termios_opt.c_cflag &= ~CSTOPB;
    termios_opt.c_cc[VTIME] = 0;
    termios_opt.c_cc[VMIN] = 1;
    tcflush(fd_,TCIFLUSH);

    if (tcsetattr(fd_, TCSANOW, &termios_opt) != 0) {
        return -1;
    }

    return 0;
}

int SerialLink::closePort() {
    if (close(fd_) < 0) {
        return -1;
    }
    else {
        return 0;
    }
}

/**
 * @brief SerialLink::SumCheck
 * @param buf
 * @param lenth
 * @return
 */
uint8_t SerialLink::SumCheck(uint8_t *buf, uint8_t lenth)
{
    uint8_t sum=0;

    for(int i=0; i < lenth; i++)
        sum = sum + buf[i];
    return sum;
}

/**
 * @brief SerialLink::sendCmd
 * @param sendmsg
 * @return
 */
bool SerialLink::sendCmd(LinkStruct_TypeDef &sendmsg, std::shared_ptr<SerialLink> port_)
{
    //boost::mutex::scoped_lock lock(mCmdMutex_); //  区域锁

    static uint16_t cmd_seq=0;
    //static uint8_t buf[LINK_NO_DATA_LENTH + LINK_MSG_MAX_LENTH] = {0xaa, 0x55, 0, 0x0a, 0};
    uint8_t buf[256];
    buf[0] = 0xaa;
    buf[1] = 0x55;
    buf[3] = 0x0a;
    cmd_seq++;
    switch (sendmsg.msg_id)
    {
        case VEL_CMD:
        case LED_STATE:
            buf[2] = 4;
            buf[5] = cmd_seq&0xff; buf[6] = (cmd_seq>>8)&0xff;
            buf[7] = sendmsg.msg_id;
            memcpy(&buf[8], &sendmsg.payload[0], 4);
            buf[12] = SumCheck(buf, 12);
            port_->writeData(buf, 13);
            break;
        case POS_CMD:
            break;
        case MOTOR_CMD:
        case CLEAR_MOTOR_ALARM:
        case FOG_LEVEL:
        case CHG_CMD:
        case UPDATE_CMD:
        case STOP_CMD:
        case BUMP_SAFETY_SENSOR:
        case CLIFF_SAFETY_SENSOR:
        case IMU_CALIBRATION_CMD:
        case MOTORCURRENT_SET:
            buf[2] = 1; //数据长度
            buf[5] = cmd_seq&0xff; buf[6] = (cmd_seq>>8)&0xff;
            buf[7] = sendmsg.msg_id;
            memcpy(&buf[8], &sendmsg.payload[0], 1);
            buf[9] = SumCheck(buf, 9);
            port_->writeData(buf, 10);
            break;
        case SAFETY_SENSORS:
        case SHUTDOWN_CMD:
        case CLIFF_THRESHOLD_SENSOR:
        case RESET_CMD:
            buf[2] = 2; //数据长度
            buf[5] = cmd_seq&0xff; buf[6] = (cmd_seq>>8)&0xff;
            buf[7] = sendmsg.msg_id;
            memcpy(&buf[8], &sendmsg.payload[0], 2);
            buf[10] = SumCheck(buf, 10);
            port_->writeData(buf, 11);
            break;
        default:
            break;
    }
    return false;
}

/**
 * @brief Paras_char
 * @param c
 * @param msg
*/
bool SerialLink::Paras_char(uint8_t c, LinkStruct_TypeDef &msg)
{
    char str[1024] = {0};
    uint16_t offset = 0;
    uint8_t n = 0;

    if(PARAS_STATE == IDIE)
        recv_num = 0;
    buf[recv_num++] = c;

    switch (PARAS_STATE)
    {
        case IDIE:
        {
            if(c == LINK_MSG_HDR1)
            {
                msg.hdr1 = c;
                packet_idx = 0;
                PARAS_STATE = GOT_HDR1;
            }
            else
            {
                PARAS_STATE = GOT_BADCHK;
            }
            break;
        }
        case GOT_HDR1:
        {
            if(c == LINK_MSG_HDR2)
            {
                msg.hdr2 = c;
                PARAS_STATE = GOT_HDR2;
            }
            else
            {
                PARAS_STATE = GOT_BADCHK;
            }
            break;
        }
        case GOT_HDR2:
        {
            msg.lenth = c;
            PARAS_STATE = GOT_LENTH;
            break;
        }
        case GOT_LENTH:
        {
            msg.ver = c;
            PARAS_STATE = GOT_VER;
            break;
        }
        case GOT_VER:
        {
            msg.ack = c;
            PARAS_STATE = GOT_ACK;
            break;
        }
        case GOT_ACK:
        {
            msg.seq = c;
            PARAS_STATE = GOT_SEQ1;
            break;
        }
        case GOT_SEQ1:
        {
            msg.seq = (c << 8)|msg.seq;
            PARAS_STATE = GOT_SEQ2;
            break;
        }
        case GOT_SEQ2:
        {
            msg.msg_id = c;
            PARAS_STATE = GOT_MSG_ID;
            break;
        }
        case GOT_MSG_ID:
        {
            msg.payload[packet_idx++] = c;
            if(packet_idx == msg.lenth)
            {
                PARAS_STATE = GOT_PAYLOAD;
            }
            break;
        }
        case GOT_PAYLOAD:
        {
            if(c == SumCheck((uint8_t *)&msg, LINK_NO_DATA_LENTH + msg.lenth -1))
            {
                packet_idx = 0;
                PARAS_STATE = IDIE;
                msg_received = 1;
            }
            else
            {
                memset(str, 0, 1024);
                offset = 0;
                for(size_t m = 0; m< recv_num; m++)
                {
                    n = sprintf(&str[offset], "%02X ", buf[m]);
                    offset += n;
                }
                RCLCPP_ERROR(rclcpp::get_logger("SerialLink::Paras_char"), "____got bad sum check! len:%d str:%s", recv_num, str);
                PARAS_STATE = GOT_BADCHK;
            }
            break;
        }
        default:
            break;
    }
    if(PARAS_STATE == GOT_BADCHK)
    {
        PARAS_STATE = IDIE;
        recv_num = 0;
        if(c == LINK_MSG_HDR1)
        {
            buf[recv_num++] = c;
            msg.hdr1 = c;
            packet_idx = 0;
            PARAS_STATE = GOT_HDR1;
        }
    }
    return msg_received;
}
