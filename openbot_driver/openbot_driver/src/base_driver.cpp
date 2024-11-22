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

#include "openbot_driver/base_driver.hpp"

#include "tf2/LinearMath/Quaternion.h"

namespace openbot_ros {

/**
 *    @brief convert vel(m/s) to motor rpm
 *    @param vel    速度（m/s）
 *    @param diameters 直径（m）
 *    @retval short rpm
 */
short Vel2RPM(double vel, double diameters)
{
    short rpm=0;

    rpm = (short)((vel * 60)/(diameters * M_PI));
    return rpm;
}
Base_Driver::Base_Driver():
    Node("base_driver_node"),
    initialized_imu(false),
    com_wdt_on(false),
    com_wdt_tick(30),
    com_linked(false),
    softswitchcmd(0),
    base_firmware_ver("0.0.0"),
    base_hardware_ver("0"),
    motor_set(0)
{
    //chassis param
    use_pose_ekf =  declare_parameter("use_pose_ekf", true);
    bump_enable = declare_parameter("bump_enable", false);
    cliff_enable = declare_parameter("cliff_enable",false);
    sonar_enable = declare_parameter("sonar_enable", false);
    odom_frame = declare_parameter("odom_frame", "odom");
    base_frame = declare_parameter("base_frame", "base_footprint");

    wheel_base = declare_parameter("wheel_base", 0.4);
    wheel_diameter = declare_parameter("wheel_diameter", 0.169);
    encoder_resolution = declare_parameter("encoder_resolution", 5600);
    gear_reduction = declare_parameter("gear_reduction",1.0);
    base_calibration = declare_parameter("base_calibration",1.0);
    diameter_calibration_l = declare_parameter("diameter_calibration_l", 1.0);
    diameter_calibration_r = declare_parameter("diameter_calibration_r", 1.0);
    imu_type = declare_parameter("imu_type","LPMS-BE2");
    resettime = declare_parameter("reset_time", 10);

    base_firmware_ver = declare_parameter("base_firmware_version",base_firmware_ver);
    base_hardware_ver = declare_parameter("base_hardware_version", base_hardware_ver);

    //serial param
    base_serial_port = declare_parameter("port", "/dev/ttyTHS0");
    base_serial_baud = declare_parameter("bandrate", 115200);

    using std::placeholders::_1;
    using std::placeholders::_2;
    //Subscriber
    cmd_vel_subscriber = create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel",10, std::bind(&Base_Driver::subVelcmd, this, _1));
    emergencyStop_subscriber = create_subscription<std_msgs::msg::Int16>(
                "emergency_stop", 10, std::bind(&Base_Driver::subStopCmd, this, _1));

    //Publisher
    odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom_raw", 10);
    imu_data_publisher = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    encoder_raw_publisher = create_publisher<geometry_msgs::msg::PointStamped>("encoder_raw", 100);

    //Server
    device_operation_srv_ = create_service<openbot_driver::srv::SetDevice>("device_operation",
                                                                             std::bind(&Base_Driver::deviceOpServer,this, _1, _2));

    // Initialize
    baseinfoPtr = std::make_shared<BaseInfo>(*this);
    batteryPtr = std::make_unique<Battery>(*this);
    rangePtr = std::make_unique<Range>(this);

    connect(base_serial_port, base_serial_baud);
    com_wdt_start();
    timerPtr = create_wall_timer(std::chrono::milliseconds(100), std::bind(&Base_Driver::com_timer_callback, this));
}

Base_Driver::~Base_Driver()
{
    std::cout << " close base driver serial" << std::endl;

    seriaLink_->closePort();
}

void Base_Driver::readData()
{
    uint8_t rx_data = 0;
    static rclcpp::Time last_stamp = get_clock()->now();
    while (rclcpp::ok()) {
        auto len = seriaLink_->readByte(&rx_data);
        if (len < 1)
            continue;
        if((get_clock()->now() - last_stamp).seconds() > 0.5)
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1,
                                        "serial received data: "<< recvMsg.msg_id << ", dt "<< (get_clock()->now() - last_stamp).seconds());

        last_stamp = this->get_clock()->now();
        seriaLink_->Paras_char(rx_data,recvMsg);
        if(seriaLink_->msg_received==1)
        {
            com_wdt_feed();
            seriaLink_->msg_received = 0;
            msgDecoderAndPub(recvMsg);
            if(com_linked==false)
            {
                com_linked = true;
                safetySensorsInitial(bump_enable, cliff_enable);//所有产品安全传感器设置方式改成一致
                sonarSafetySensorSet(sonar_enable);
                bumpSafetySensorSet(bump_enable);
                cliffSafetySensorSet(cliff_enable);
                cliffthresholdSet();
            }
        }
    }
}

void Base_Driver::connect(std::string dev_name, uint32_t bouadrate)
{
    seriaLink_ = std::shared_ptr<SerialLink>(new SerialLink(dev_name, bouadrate));
    if (seriaLink_->openPort() == 0) {
        RCLCPP_INFO(rclcpp::get_logger("base_driver"),"serial port open %s baud %d", dev_name.c_str(), bouadrate);
        read_data_thread_ = std::shared_ptr<std::thread>(
            new std::thread(std::bind(&Base_Driver::readData, this)));
    }
    else {
        RCLCPP_ERROR(rclcpp::get_logger("connect"),"Failed to open %s", seriaLink_->getDevPath().c_str());
        seriaLink_->closePort();
    }
}

void Base_Driver::safetySensorsInitial(bool bump_state, bool cliff_state)
{
    sendMsg.msg_id = SerialLink::SAFETY_SENSORS;
    sendMsg.payload[0] = bump_state;
    sendMsg.payload[1] = cliff_state;
    RCLCPP_INFO(rclcpp::get_logger("base_driver"),"safety_sensors setting, bump : %d, cliff: %d",bump_state ,cliff_state);
    ser_mutex.lock();
    seriaLink_->sendCmd(sendMsg, seriaLink_);
    ser_mutex.unlock();
}

void Base_Driver::bumpSafetySensorSet(bool bump_state)
{
    sendMsg.msg_id = SerialLink::BUMP_SAFETY_SENSOR;
    sendMsg.payload[0] = bump_state;
    RCLCPP_INFO(rclcpp::get_logger("base_driver"),"bump safety sensors setting : %d",bump_state);
    ser_mutex.lock();
    seriaLink_->sendCmd(sendMsg, seriaLink_);
    ser_mutex.unlock();
}

void Base_Driver::cliffSafetySensorSet(bool cliff_state)
{
    sendMsg.msg_id = SerialLink::CLIFF_SAFETY_SENSOR;
    sendMsg.payload[0] = cliff_state;
    RCLCPP_INFO(rclcpp::get_logger("base_driver"),"cliff safety sensors setting : %d",cliff_state);
    ser_mutex.lock();
    seriaLink_->sendCmd(sendMsg, seriaLink_);
    ser_mutex.unlock();
}

void Base_Driver::cliffthresholdSet()
{
    sendMsg.msg_id = SerialLink::CLIFF_THRESHOLD_SENSOR;
    sendMsg.payload[0] = (cliffthreshold >> 8)  & 0xff;
    sendMsg.payload[1] = cliffthreshold & 0xff;
    RCLCPP_INFO(rclcpp::get_logger("base_driver"),"cliff threshold setting : %d",cliffthreshold);
    ser_mutex.lock();
    seriaLink_->sendCmd(sendMsg, seriaLink_);
    ser_mutex.unlock();
}

void Base_Driver::sonarSafetySensorSet(bool sonar_state)
{
    sendMsg.msg_id = SerialLink::SONAR_SAFETY_SENSOR;
    sendMsg.payload[0] = sonar_state;
    RCLCPP_INFO(rclcpp::get_logger("base_driver"),"sonar safety sensors setting : %d",sonar_state);
    ser_mutex.lock();
    seriaLink_->sendCmd(sendMsg, seriaLink_);
    ser_mutex.unlock();
}

void Base_Driver::motor_release_Set(uint8_t motor_en)
{
    sendMsg.msg_id = SerialLink::MOTOR_CMD;
    sendMsg.payload[0] = motor_en;
    RCLCPP_INFO(rclcpp::get_logger("base_driver"),"motor setting : %d",motor_en);
    ser_mutex.lock();
    seriaLink_->sendCmd(sendMsg, seriaLink_);
    ser_mutex.unlock();
}

void Base_Driver::msgDecoderAndPub(const LinkStruct_TypeDef &msg)
{
    switch (msg.msg_id)
    {
        case ENCODER:
        {
            updateOdom(msg);
            break;
        }
        case IMU:
        {
            updateImu(msg);
            publishIMU(imu);
            break;
        }
        case BASE_INFO:
        {
            updateBaseInfo(msg);
            break;
        }
        case SOFTVER:
        {
            updateVersionInfo(msg);
            break;
        }
        case SONAR:
        {
            updateSonar(msg);
            break;
        }
        case ADC_VALUE:
        {
            updateADC(msg);
            break;
        }
        case BAT:
        {
            updateBms(msg);
            break;
        }
        default:
            RCLCPP_INFO(this->get_logger(),"ERROR msg_id %d", msg.msg_id);
            break;
    }
}

void Base_Driver::updateOdom(const LinkStruct_TypeDef &msg)
{
    uint32_t stm32_timestamp;
    int32_t encoder_delta_L, encoder_delta_R;
    static int32_t encoder_now_L, encoder_now_R, encoder_last_L=0, encoder_last_R=0;
    static rclcpp::Time time_now = get_clock()->now(), time_last = get_clock()->now();
    static double_t theta=0.0, x=0, y=0; //里程计累计值
    static uint32_t stm32time_last, st_time_delta;
    int32_t motor_vel_L = 0, motor_vel_R = 0;
    static bool initialized;//里程计开机校准

    memcpy(&stm32_timestamp, &msg.payload[0], 4);
    memcpy(&encoder_now_L, &msg.payload[4], 4);
    memcpy(&encoder_now_R, &msg.payload[8], 4);
    memcpy(&motor_vel_L, &msg.payload[12], 4);
    memcpy(&motor_vel_R, &msg.payload[16], 4);

    geometry_msgs::msg::PointStamped encoder_raw_msg;
    encoder_raw_msg.header.stamp = get_clock()->now();
    encoder_raw_msg.point.x = encoder_now_L;
    encoder_raw_msg.point.y = encoder_now_R;
    encoder_raw_msg.point.z = stm32_timestamp;
    encoder_raw_publisher->publish(encoder_raw_msg);

    if(!initialized)                    //重新启动节点 初始化里程计数据
    {
        initialized = true;
        encoder_last_L = encoder_now_L;
        encoder_last_R = encoder_now_R;
        stm32time_last = stm32_timestamp;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("base_driver"), "L " << encoder_now_L << ", R " << encoder_now_R);
    }
    encoder_delta_L = (encoder_now_L - encoder_last_L);
    encoder_delta_R = encoder_now_R - encoder_last_R;
    st_time_delta = GET_TIME_DELTA(stm32_timestamp , stm32time_last);   //ms 溢出处理

    encoder_last_L = encoder_now_L;     //更新里程计历史值
    encoder_last_R = encoder_now_R;
    stm32time_last = stm32_timestamp;

    double_t delta_l = (encoder_delta_L/(encoder_resolution*gear_reduction)) * M_PI * wheel_diameter * diameter_calibration_l;
    double_t delta_r = (encoder_delta_R/(encoder_resolution*gear_reduction)) * M_PI * wheel_diameter * diameter_calibration_r;
    double_t delta_s = (delta_l + delta_r)/2;
    double_t delta_th= (delta_r - delta_l) / (wheel_base * base_calibration);

    time_now = get_clock()->now();
    time_last = time_now;

    double_t delta_x = delta_s * cos(theta + delta_th/2);
    double_t delta_y = delta_s * sin(theta + delta_th/2);
    x = x + delta_x;
    y = y + delta_y;

    theta = theta + delta_th;
    if(theta > M_PI)
        theta -= 2*M_PI;
    else
    {
        if(theta <= -M_PI)
            theta += 2*M_PI;
    }
    odometry.pose.pose.position.x = x;
    odometry.pose.pose.position.y = y;
    odometry.pose.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0,0,theta);
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();
    odometry.twist.twist.linear.x = delta_s/(st_time_delta/1000.0);      //采用32的时间戳  TODO 平滑滤波
    odometry.twist.twist.angular.z = delta_th/(st_time_delta/1000.0);
}

void Base_Driver::updateImu(const LinkStruct_TypeDef &msg)
{
    float roll=0.0, pitch=0.0, yaw=0.0, accx=0.0, accy=0.0, accz=0.0, gyrox=0.0, gyroy=0.0, gyroz=0.0;
    uint8_t type = 0;
    std_msgs::msg::UInt8 cabilation;
    static std_msgs::msg::UInt8 cab;

    if(msg.lenth == 26)
    {
        cabilation.data = msg.payload[24];
        type = msg.payload[25];
        if(type ==1)
        {
            imu_type = "SGM61";
        }
        else if(type ==2)
        {
            imu_type = "JY901";
        }
        else if(type ==3)
        {
            imu_type = "LPMS-BE2";
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("base_driver"),"imu type err");
        }

        set_parameter(rclcpp::Parameter("imu_type", imu_type));
        if(cab.data != cabilation.data)
        {
            RCLCPP_INFO(rclcpp::get_logger("base_driver"),"imu type is %d,cabilation is %d" ,type, cabilation.data);
        }
        cab = cabilation;
    }

    if(msg.lenth == 24 || type == 2)
    {
        RCLCPP_WARN_ONCE(rclcpp::get_logger("base_driver"),"imu type is 2");
        roll = (float)((short(msg.payload[1] << 8|msg.payload[0]))*1.0*ANGLE_SCALE);
        pitch = (float)((short(msg.payload[3] << 8|msg.payload[2]))*1.0*ANGLE_SCALE);
        yaw = (float)((short(msg.payload[5] << 8|msg.payload[4]))*1.0*ANGLE_SCALE);

        accx = (float)((short(msg.payload[7]<<8|msg.payload[6]))*1.0*ACC_SCALE);
        accy = (float)((short(msg.payload[9]<<8|msg.payload[8]))*1.0*ACC_SCALE);
        accz = (float)((short(msg.payload[11]<<8|msg.payload[10]))*1.0*ACC_SCALE);

        gyrox = (float)((short(msg.payload[13]<<8|msg.payload[12]))*1.0*GYRO_SCALE);
        gyroy = (float)((short(msg.payload[15]<<8|msg.payload[14]))*1.0*GYRO_SCALE);
        gyroz = (float)((short(msg.payload[17]<<8|msg.payload[16]))*1.0*GYRO_SCALE);
    }
    else if(type == 1)
    {
        roll = (float)((short(msg.payload[1] << 8|msg.payload[0]))*1.0/100);
        pitch = (float)((short(msg.payload[3] << 8|msg.payload[2]))*1.0/100);
        yaw = (float)((short(msg.payload[5] << 8|msg.payload[4]))*1.0/100);

        accx = (float)((short(msg.payload[7]<<8|msg.payload[6]))*1.0*G/1000);
        accy = (float)((short(msg.payload[9]<<8|msg.payload[8]))*1.0*G/1000);
        accz = (float)((short(msg.payload[11]<<8|msg.payload[10]))*1.0*G/1000);

        gyrox = (float)((short(msg.payload[13]<<8|msg.payload[12]))*1.0/64*degrees2rad);
        gyroy = (float)((short(msg.payload[15]<<8|msg.payload[14]))*1.0/64*degrees2rad);
        gyroz = (float)((short(msg.payload[17]<<8|msg.payload[16]))*1.0/64*degrees2rad);
    }
    else if(type == 3)
    {
        roll = (float)((short(msg.payload[1] << 8|msg.payload[0]))*1.0/100);
        pitch = (float)((short(msg.payload[3] << 8|msg.payload[2]))*1.0/100);
        yaw = (float)((short(msg.payload[5] << 8|msg.payload[4]))*1.0/100);

        accx = (float)((short(msg.payload[7]<<8|msg.payload[6]))*1.0*G/100);
        accy = (float)((short(msg.payload[9]<<8|msg.payload[8]))*1.0*G/100);
        accz = (float)((short(msg.payload[11]<<8|msg.payload[10]))*1.0*G/100);

        gyrox = (float)((short(msg.payload[13]<<8|msg.payload[12]))*1.0/100*degrees2rad);
        gyroy = (float)((short(msg.payload[15]<<8|msg.payload[14]))*1.0/100*degrees2rad);
        gyroz = (float)((short(msg.payload[17]<<8|msg.payload[16]))*1.0/100*degrees2rad);
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("base_driver"),"imu type error");
    }

    imu.linear_acceleration.x = accx;
    imu.linear_acceleration.y = accy;
    imu.linear_acceleration.z = accz;

    imu.angular_velocity.x = gyrox;
    imu.angular_velocity.y = gyroy;
    imu.angular_velocity.z = gyroz;

    //开机 零偏
    if(!initialized_imu)
    {
        initialized_imu = true;
        imu_yaw_offset = yaw;
        RCLCPP_WARN_ONCE(rclcpp::get_logger("base_driver"), "imu_yaw_offset, [ %.2f ]", yaw);
    }
    yaw = yaw - imu_yaw_offset;
    if(yaw > 180)
        yaw -= 2*180;
    else
    {
        if(yaw <= -180)
            yaw += 2*180;
    }

    if(pitch > 180)
        pitch -= 2*180;
    else
    {
        if(pitch <= -180)
            pitch += 2*180;
    }

    if(roll > 180)
        roll -= 2*180;
    else
    {
        if(roll <= -180)
            roll += 2*180;
    }

    double_t imu_yaw_rad = yaw * degrees2rad;
    double_t imu_pitch_rad = pitch * degrees2rad;
    double_t imu_roll_rad = roll * degrees2rad;
    if(imu_yaw_rad > M_PI)
        imu_yaw_rad -= 2*M_PI;
    else
    {
        if(imu_yaw_rad <= -M_PI)
            imu_yaw_rad += 2*M_PI;
    }
    if(imu_pitch_rad > M_PI)
        imu_pitch_rad -= 2*M_PI;
    else
    {
        if(imu_pitch_rad <= -M_PI)
            imu_pitch_rad += 2*M_PI;
    }
    if(imu_roll_rad > M_PI)
        imu_roll_rad -= 2*M_PI;
    else
    {
        if(imu_roll_rad <= -M_PI)
            imu_roll_rad += 2*M_PI;
    }

    tf2::Quaternion q;
    q.setRPY(imu_roll_rad,imu_pitch_rad,imu_yaw_rad);
    imu.orientation.x = q.x();
    imu.orientation.y = q.y();
    imu.orientation.z = q.z();
    imu.orientation.w = q.w();
}

void Base_Driver::updateADC(const LinkStruct_TypeDef &msg)
{
    std::vector<uint16_t> adc_value(10);
    openbot_driver::msg::BaseInfo data;

    memcpy(&adc_value[0], &msg.payload[0], msg.lenth);

    rangePtr->fillAndPubIrmsg(adc_value);
}

void Base_Driver::updateSonar(const LinkStruct_TypeDef &msg)
{
    std::vector<uint16_t> sonar_value(10);

    memcpy(&sonar_value[0], &msg.payload[0], msg.lenth);
    if(false==sonar_enable)
    {
        sonar_value[0]=1500;
        sonar_value[1]=1500;
        sonar_value[2]=1500;
        sonar_value[3]=1500;
    }
    rangePtr->fillAndPubSonarmsg(sonar_value);

}

void Base_Driver::updateBaseInfo(const LinkStruct_TypeDef &msg)
{
    std::vector<uint8_t > baseinfo_value(msg.lenth);
    openbot_driver::msg::BaseInfo data;

    memcpy(&baseinfo_value[0], &msg.payload[0], msg.lenth);

    baseinfoPtr->updateData(baseinfo_value);
}

void Base_Driver::updateBms(const LinkStruct_TypeDef &msg)
{
    std::vector<uint8_t > bms_value(msg.lenth);
    openbot_driver::msg::BaseInfo data;

    memcpy(&bms_value[0], &msg.payload[0], msg.lenth);
    baseinfoPtr->getBaseInfo(data);

    batteryPtr->fillAndPubMsg(bms_value, data.charging_state);
}

void Base_Driver::updateVersionInfo(const LinkStruct_TypeDef &msg)
{
    deb_ver = "0.0.0";
    base_firmware_ver = deb_ver.substr(0,1) + "." + std::to_string(msg.payload[0]) + "." + std::to_string(msg.payload[1]) + "." + std::to_string(msg.payload[2]);
    base_hardware_ver = std::to_string(msg.payload[3]);
    if(msg.lenth==5)
    {
        ctrlboard_reset = msg.payload[4];
        //控制板复位原因0:NONE1:IWDGRST2:SFTRST3:PORRST4:PINRST5:LPWRRST6:BORRST7:WWDGRST
        if(ctrlboard_reset)
            RCLCPP_INFO(rclcpp::get_logger("base_driver"), "Ctrlboad reset , code is %d",ctrlboard_reset);
    }
    this->set_parameter(rclcpp::Parameter("base_firmware_version", base_firmware_ver));
    this->set_parameter(rclcpp::Parameter("base_hardware_version", base_hardware_ver));
    if(deb_ver.size()!=0)
    {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("base_driver"),"robot_version: %s",deb_ver.c_str());
        RCLCPP_INFO_ONCE(rclcpp::get_logger("base_driver"),"base firmware version %s, hardware version %s", base_firmware_ver.c_str(), base_hardware_ver.c_str());
    }
}

void Base_Driver::com_wdt_start()
{
    com_wdt_on =  true;
}

void Base_Driver::com_wdt_stop()
{
    com_wdt_on = false;
}

void Base_Driver::com_wdt_feed()
{
    com_wdt_tick =  30;
}

void Base_Driver::com_wdt_var_dec()
{
    if(com_wdt_on ==  true)
    {
        com_wdt_tick--;
    }
}

void Base_Driver::com_timer_callback()
{
    com_wdt_var_dec();
    if(com_wdt_tick < 0)
    {
        ser_mutex.lock();
        RCLCPP_ERROR(rclcpp::get_logger("base_driver"),"timer callback, openstate:%d, reconnect", seriaLink_->isOpen());
        com_wdt_tick = 30;
        com_linked = false;
        seriaLink_->Bufinit();
        ser_mutex.unlock();
    }

    openbot_driver::msg::BaseInfo data;
    static openbot_driver::msg::BaseInfo lastdata;

    baseinfoPtr->getBaseInfo(data);
    {
        static double sdbegin = 0.0f;//延迟执行shutdown

        if(softswitchcmd == 1)
        {
            softswitchcmd = 2;
            sdbegin  = get_clock()->now().seconds();
        }
        if(softswitchcmd == 2)
        {
            if(get_clock()->now().seconds() - sdbegin > 2)
            {
                softswitchcmd = 3;
                system("whoami | sudo -S ps -ef | grep ros | grep -v grep | awk '{print $2}' | xargs kill -9");
            }
        }
    }
    static uint8_t motor_set_time = 0;//电机设置命令延迟执行时间
    if(motor_set == 1)
    {
        motor_set_time++;
        if((data.base_device_state&0x10)!=0x10)//电机没有下电
        {
            motor_release_Set(1);
        }
        else
        {
            if(motor_set_time>=50)//等待baseinfo更新
            {
                motor_set_time = 0;
                motor_set = 0;
            }
        }
    }
    else if(motor_set == 2)
    {
        motor_set_time++;
        if((data.base_device_state&0x01) == 0x01)//急停按键按下
        {
            if(motor_set_time>=50)//等待baseinfo更新
            {
                motor_set_time = 0;
                motor_set = 0;
            }
        }
        else
        {
            if((data.base_device_state&0x10) != 0x00)//电机没有上电
            {
                motor_release_Set(0);
            }
            else
            {
                if(motor_set_time>=50)//等待baseinfo更新
                {
                    motor_set_time = 0;
                    motor_set = 0;
                }
            }
        }
    }

    lastdata = data;
}

void Base_Driver::subVelcmd(const geometry_msgs::msg::Twist &msg)
{
    geometry_msgs::msg::Twist recv_velCmd = msg;
    short rpm_l, rpm_r;
    double speed_l, speed_r;

    speed_l = (2 * recv_velCmd.linear.x - wheel_base * recv_velCmd.angular.z)/2;    //m/s
    speed_r = (2 * recv_velCmd.linear.x + wheel_base * recv_velCmd.angular.z)/2;    //m/s
    rpm_l = Vel2RPM(speed_l, wheel_diameter);//mm/s    数据类型转换会有精度丢失
    rpm_r = Vel2RPM(speed_r, wheel_diameter);//mm/s

    sendMsg.msg_id = 10;
    sendMsg.payload[0] = (uint8_t)(rpm_l&0xff); sendMsg.payload[1] = (uint8_t)((rpm_l>>8)&0xff);
    sendMsg.payload[2] = (uint8_t)(rpm_r&0xff); sendMsg.payload[3] = (uint8_t)((rpm_r>>8)&0xff);

    ser_mutex.lock();
    seriaLink_->sendCmd(sendMsg, seriaLink_);
    ser_mutex.unlock();
}

void Base_Driver::subStopCmd(const std_msgs::msg::Int16 &msg)
{
    std_msgs::msg::Int16 recv_msg = msg;

    sendMsg.msg_id = 14;
    if(recv_msg.data)
    {
        RCLCPP_INFO(rclcpp::get_logger("base_driver"),"bingo_driver, stop !");
        sendMsg.payload[0] = 1;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("base_driver"),"bingo_driver, recovery !");
        sendMsg.payload[0] = 0;
    }
    ser_mutex.lock();
    seriaLink_->sendCmd(sendMsg, seriaLink_);
    ser_mutex.unlock();
}

bool Base_Driver::deviceOpServer(const std::shared_ptr<openbot_driver::srv::SetDevice::Request> req, std::shared_ptr<openbot_driver::srv::SetDevice::Response> res)
{
    std_msgs::msg::Int16 recv_cmd;

    std::string device_id = req->device_id;
    std::string cmd       = req->cmd;
    int32_t parameter1    = req->parameter1;
    int32_t parameter2    = req->parameter2;
    std::string parameter3    = req->parameter3;
    static int32_t lastledstat = 0;
    uint8_t ret = 0;

    /*---*/
    if(device_id == "motor")
    {
        if(cmd == "release_motor")
        {
            sendMsg.msg_id = SerialLink::MOTOR_CMD;
            if(parameter1 == 1)
            {
                motor_set = 1;
                RCLCPP_INFO(rclcpp::get_logger("base_driver"),"disable motor");
                sendMsg.payload[0] = 1;
            }else if(parameter1 == 0)
            {
                motor_set = 2;
                sendMsg.payload[0] = 0;
                RCLCPP_INFO(rclcpp::get_logger("base_driver"),"enable motor");
            }
            else
            {
                ret = 1;
                RCLCPP_INFO(rclcpp::get_logger("base_driver"),"release_motor param err");
            }
        }
        else if("clear_alarm" == cmd)
        {
            sendMsg.msg_id = SerialLink::CLEAR_MOTOR_ALARM;
            if(1 == parameter1)
            {
                RCLCPP_INFO(rclcpp::get_logger("base_driver"),"clear motor alarm");
                sendMsg.payload[0] = 1;
            }
            else
            {
                ret = 1;
                RCLCPP_INFO(rclcpp::get_logger("base_driver"),"clear_alarm param err");
            }
        }
        else if("set_torque" == cmd)
        {
             sendMsg.msg_id = SerialLink::MOTORCURRENT_SET;
             if((parameter2>=1) && (parameter2<=90))
             {
                RCLCPP_INFO(rclcpp::get_logger("base_driver"),"motor current set %f A",parameter2/10.0);
                sendMsg.payload[0] = parameter2;
             }
            else
            {
                ret = 1;
                RCLCPP_INFO(rclcpp::get_logger("base_driver"),"set_torque param err!!!!");
            }
        }
        else
        {
            ret = 1;
            RCLCPP_INFO(rclcpp::get_logger("base_driver"),"motor cmd err!!!!");
        }
    }
    else if(device_id == "safety_sensors")
    {
        if(cmd == "cliff_enable")
        {
            sendMsg.msg_id = SerialLink::SAFETY_SENSORS;
            sendMsg.payload[0] = 0;
            sendMsg.payload[1] = parameter1?1:0;
            RCLCPP_INFO(this->get_logger(),"safety_sensors setting, cliff: %d",parameter1);
            cliff_enable = parameter1;
            bump_enable = 0;
        }
    }
    else if(device_id == "led")
    {
        if(cmd == "state")
        {
            sendMsg.msg_id = SerialLink::LED_STATE;
            sendMsg.payload[0] = parameter1;
            sendMsg.payload[1] = (parameter2 >> 16) & 0xff;
            sendMsg.payload[2] = (parameter2 >> 8)  & 0xff;
            sendMsg.payload[3] = parameter2 & 0xff;
            if(lastledstat != parameter1)//状态切换时再打印
                RCLCPP_INFO(this->get_logger(),"led ctrl state:%d R:%d G:%d B:%d", sendMsg.payload[0],sendMsg.payload[1],sendMsg.payload[2],sendMsg.payload[3]);
            lastledstat = parameter1;
        }
        else
        {
            ret = 1;
            RCLCPP_INFO(this->get_logger(),"led state cmd err");
        }
    }
    else
        RCLCPP_INFO(rclcpp::get_logger("base_driver")," cmd err!!!!");

    if(ret == 0)
    {
        ser_mutex.lock();
        seriaLink_->sendCmd(sendMsg, seriaLink_);
        ser_mutex.unlock();
    }
    res->result = (ret == 1) ? false : true;
    res->message_code = parameter1;
    res->message = device_id + ", " + cmd;
    return true;
}

void Base_Driver::publishOdom(const nav_msgs::msg::Odometry &data)
{
    nav_msgs::msg::Odometry odom;

    // Header
    odom.header.stamp = get_clock()->now();
    odom.header.frame_id = odom_frame;
    odom.child_frame_id = base_frame;

    // Position
    odom.pose.pose.position.x = data.pose.pose.position.x;
    odom.pose.pose.position.y = data.pose.pose.position.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = data.pose.pose.orientation;

    // Velocity
    odom.twist.twist.linear.x = data.twist.twist.linear.x;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = data.twist.twist.angular.z;

    odom_publisher->publish(odom);
}

void Base_Driver::publishIMU(const sensor_msgs::msg::Imu &data)
{
    sensor_msgs::msg::Imu imu;

    imu.header.stamp = get_clock()->now();
    imu.header.frame_id = "imu";

    imu.linear_acceleration.x = data.linear_acceleration.x;
    imu.linear_acceleration.y = data.linear_acceleration.y;
    imu.linear_acceleration.z = data.linear_acceleration.z;
    imu.linear_acceleration_covariance = {0.04, 0, 0,
                                          0, 0.04, 0,
                                          0, 0, 0.04};

    imu.angular_velocity.x = data.angular_velocity.x;
    imu.angular_velocity.y = data.angular_velocity.y;
    imu.angular_velocity.z = data.angular_velocity.z;
    imu.angular_velocity_covariance = {0.02, 0, 0,
                                       0, 0.02, 0,
                                       0, 0, 0.02};

    imu.orientation.x = data.orientation.x;
    imu.orientation.y = data.orientation.y;
    imu.orientation.z = data.orientation.z;
    imu.orientation.w = data.orientation.w;
    //roll pitch yaw
    imu.orientation_covariance = {1e6, 0, 0,
                                  0, 1e6, 0,
                                  0, 0, 0.05};

    imu_data_publisher->publish(imu);
}

}  // namespace openbot_ros
