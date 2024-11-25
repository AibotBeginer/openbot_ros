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

#ifndef OPENBOT_DRIVER_BASE_DRIVER_HPP
#define OPENBOT_DRIVER_BASE_DRIVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "openbot_driver/msg/base_info.hpp"
#include "openbot_driver/srv/set_device.hpp"

#include "openbot_driver/baseinfo.hpp"
#include "openbot_driver/battery.hpp"
#include "openbot_driver/range.hpp"
#include "openbot_driver/serial_link.hpp"

#define GET_TIME_DELTA(now, last) (now > last)?(now - last):(std::numeric_limits<uint32_t>::max()-last + now+1) ///<计算时间差

#define degrees2rad     3.1415926/180.0             ///<角度转转速系数
#define G               9.8                         ///<加速度
#define ACC_SCALE       16*G/32768                  ///<加速度系数
#define GYRO_SCALE      2000.0*degrees2rad/32768    ///<°/s，计算速度
#define ANGLE_SCALE     180.0/32768                 ///<度,计算rpy

namespace openbot_ros {
/**
 * @class Base_Driver
 * @brief 传感器相关类
*/
class Base_Driver:public rclcpp::Node
{
    /** @brief 消息ID*/
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
public:
    Base_Driver();///<构造函数
    ~Base_Driver();///<析构函数
    void spin();///<回调函数
private:

    std::string         cfg_path;///<配置文件路径
    bool                use_pose_ekf;///<位置获取来源开关
    bool                bump_enable;///<碰撞检测开关
    bool                cliff_enable;///<跌落检测开关
    bool                sonar_enable;///<超声检测开关
    std::string         odom_frame;///<里程计坐标系
    std::string         base_frame;///<基础坐标系

    std::string   base_serial_port;///<串口号
    int           base_serial_baud;///<波特率
    bool           initialized_imu;///<imu初始化标志
    double_t        imu_yaw_offset;///< 偏航角偏移
    double_t            wheel_base;///<轮距
    double_t        wheel_diameter;///<轮径
    double_t        encoder_resolution;///<编码器分辨率
    double_t        gear_reduction;///<转换系数
    double_t        base_calibration;///<轮距校准比例
    double_t        diameter_calibration_l;///<左轮径校准比列
    double_t        diameter_calibration_r;///<右轮径校准比例
    std::string     imu_type;///<imu型号
    int cliffthreshold;///<防跌落阈值
    int resettime;///<整机重启时间

    std::thread recv_thread;///<接收线程句柄

    void readData();
    void connect(std::string dev_name, uint32_t bouadrate);

    /**
     * @brief Base_Driver::safetySensorsInitial
     * @param bump_state
     * @param cliff_state
     */
    void safetySensorsInitial(bool bump_state, bool cliff_state);
    /**
     * @brief 碰撞传感器使能设置
     * @param bump_state false=失能，true=使能
     */
    void bumpSafetySensorSet(bool bump_state);
    /**
     * @brief 跌落传感器使能设置
     * @param cliff_state false=失能，true=使能
     */
    void cliffSafetySensorSet(bool cliff_state);

    void cliffthresholdSet(void);
    /**
     * @brief 超声波传感器使能设置
     * @param sonar_state false=失能，true=使能
     */
    void sonarSafetySensorSet(bool sonar_state);
    void motor_release_Set(uint8_t motor_en);
    /*********************
    ** msg handle
    **********************/

    LinkStruct_TypeDef       sendMsg; ///<待发送消息
    LinkStruct_TypeDef       recvMsg; //<接收待解码消息
    std::shared_ptr<SerialLink> seriaLink_;
    std::shared_ptr<std::thread> read_data_thread_;

    bool com_wdt_on;///<串口超时检测开关
    int com_wdt_tick;///<串口超时检测计时器
    bool com_linked;///<串口连接标识
    uint8_t softswitchcmd;///<软关机命令下发标志
    std::mutex ser_mutex;///<互斥信号量
    std::string base_firmware_ver;///<固件版本号
    std::string base_hardware_ver;///<硬件版本号
    std::string deb_ver;
    int ctrlboard_reset;///<控制板重启原因
    uint8_t motor_set;///电机设置命令

    void msgDecoderAndPub(const LinkStruct_TypeDef &msg);///<控制板数据解析
    void updateOdom(const LinkStruct_TypeDef &msg);///<里程计数据更新
    void updateImu(const LinkStruct_TypeDef &msg);///<IMU数据更新
    void updateADC(const LinkStruct_TypeDef &msg);///<跌落数据更新
    void updateSonar(const LinkStruct_TypeDef &msg);///<超声波数据更新
    void updateBaseInfo(const LinkStruct_TypeDef &msg);///<baseinfo数据更新
    void updateBms(const LinkStruct_TypeDef &msg);///<bms数据更新
    void updateVersionInfo(const LinkStruct_TypeDef &msg);///<版本信息更新
    void com_wdt_start(void);///<串口超时检测开始
    void com_wdt_stop(void);///<串口超时检测停止
    void com_wdt_feed(void);///<串口超时检测时重载
    void com_wdt_var_dec(void);///<串口超时检测计时器更新
    void com_timer_callback();///<回调函数


    /*********************
     ** Ros Comms
     **********************/
    sensor_msgs::msg::Imu       imu;///<惯导元件数据
    nav_msgs::msg::Odometry     odometry;///<里程计数据

    std::shared_ptr<Range>      rangePtr;///<RANGE_SENSOR类指针变量
    std::shared_ptr<Battery>    batteryPtr;///<BATTERY类指针变量
    std::shared_ptr<BaseInfo>   baseinfoPtr;///<BaseInfo类指针变量

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;///<里程计发布句柄
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_publisher;///<imu发布句柄
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr  encoder_raw_publisher;
//    rclcpp::Publisher<> imu_calibration_publisher;///imu校准状态发布

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;///<命令速度订阅句柄
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr emergencyStop_subscriber;///<急停订阅句柄

    rclcpp::Service<openbot_driver::srv::SetDevice>::SharedPtr  device_operation_srv_;///<设备操作服务句柄
//    ros::ServiceClient  device_Client;
    rclcpp::TimerBase::SharedPtr timerPtr;///<时钟管理指针

    /**
     * @brief 速度下发
     * @details 将订阅的cmd_vel_topic的速度值换算之后下发给控制板
     * @param msg 速度值，包括线速度和角速度
     */
    void subVelcmd(const geometry_msgs::msg::Twist &msg);
    /**
     * @brief 急停下发
     * @details 将订阅的emergency_stop信号下发给控制板
     * @param msg 急停信号0=急停恢复1=急停按下
     */
    void subStopCmd(const std_msgs::msg::Int16 &msg);
    /**
     * @brief deviceOpServer
     * @param req
     * @param res
     * @return
     */
    bool deviceOpServer(const std::shared_ptr<openbot_driver::srv::SetDevice::Request> req,
                        std::shared_ptr<openbot_driver::srv::SetDevice::Response> res);
    /**
     * @brief 里程计topic发布
     * @param data
    */
    void publishOdom(const nav_msgs::msg::Odometry &data);
    /**
     * @brief IMU topic发布
     * @param data
    */
    void publishIMU(const sensor_msgs::msg::Imu &data);
};

}  // namespace openbot_ros

#endif // OPENBOT_DRIVER_BASE_DRIVER_HPP
