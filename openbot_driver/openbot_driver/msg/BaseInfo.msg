# msg definition
# base_device_state enum
uint8   DEVICE_EMERGENCY    = 1 # 急停按下
uint8   ROS_NO_CMD          = 2 # 未收到ros指令
uint8   DRIVER_LOST         = 4 # 驱动器无响应
uint8   IMU_LOST            = 8 # imu无响应
uint8   RELEASE_MOTOR       = 16 # 电机下电失能
uint8   BMS_LOST            = 32 # bms无响应

# charging State enum
uint8   UNCHARGING          = 0 # 未充电
uint8   CHARGING            = 1 # 充电中
uint8   DOCK_IN             = 2 # 接触，未充电
uint8   CHARGE_OK           = 3 # 充电完成
uint8   DC_CHARGING         = 4 # 直充-充电中
uint8   DC_DOCK_IN          = 5 # 直充-接触，未充电
uint8   DC_CHARGE_OK        = 6 # 直充-充电完成

bool        connected               #   底盘连接状态
uint8       base_device_state       #   底盘基本外设状态
bool        cliff_left              #   左跌落
bool        cliff_front_left        #   前左
bool        cliff_front_right       #   前右
bool        cliff_right             #   右跌落
bool        bump_state_left         #   左碰撞
bool        bump_state_right        #   右碰撞
uint16      charging_vol             #   充电电压  10mv
uint8       charging_state          #   充电状态
uint8       charge_ctrl             #   充电开关控制状态，未使用
uint16      motor_left_err_code     #   电机错误码
uint16      motor_right_err_code    #
bool        bump_enable             #   碰撞传感器使能
bool        cliff_enable            #   跌落使能
bool        sonar_enable            #
#uint32[]    adc_channal             #   ADC传感器通道值，跌落传感器等
#uint32[]    range_sensors           #   超声 tof数据等
