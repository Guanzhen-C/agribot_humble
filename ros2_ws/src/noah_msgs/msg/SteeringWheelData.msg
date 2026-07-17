
# 方向盘状态信息

std_msgs/Header header
 
#当前角度传感器类型 0：IMU 1：同心轴传感器 2:无传感器
uint8 cur_angle_type

#旋转速度，单位弧度
float64 rotational_speed
#标定值，单位弧度
float64 calibration
#死区值，单位弧度
float64 dead_zone
#最大开度，单位弧度
float64 max_degree
#当前状态
#   0   //空模式
#   1   //配置手动模式
#   2   //配置无人模式
#   3   //无人工作模式
#   4   //手动模式
#   5   //故障模式
uint8  cur_state

#车辆信息
#当前电流
uint16 cur_current
#当前速度
int16 cur_speed
#目标速度
int16 tar_speed

#角度传感器
float64 angle
#imu当前角度
float64 imu_angle

#错误码,堵转错误后会自动切换到人工模式，每一位的作用如下
# work_mode                        : 1; // Bit0: 工作模式故障
# speed_deviation                  : 1; // Bit1: 速度偏差过大
# overheat_alarm                   : 1; // Bit2: 过温报警
# encoder_fault                    : 1; // Bit3: 编码器故障
# other_fault                      : 1; // Bit4: 其他错误
# reserved                         : 1; // Bit5: 保留
# can_communication_fault          : 1; // Bit6: CAN通讯中断
# motor_stall                      : 1; // Bit7: 堵转
# motor_disable                    : 1; // Bit8: 电机未使能
# overvoltage_alarm                : 1; // Bit9: 过压报警
# hardware_overcurrent_fault       : 1; // Bit10: 硬件过流故障
# eeprom_fault                     : 1; // Bit11: E2PROM故障
# undervoltage                     : 1; // Bit12: 欠压
# position_deviation_alarm         : 1; // Bit13: 位置偏差报警
# software_overcurrent_protection  : 1; // Bit14: 软件过流保护
# control_mode_error               : 1; // Bit15: 控制模式错误
uint16 errcode

#方向盘连接状态
uint8 is_steedwheel_connect
#角度传感器连接状态
uint8 is_angle_sensor_connect 


#方向盘转速异常可以通过此字段判断是否人工介入
#0x00 TORQUE_NORMAL               无扭矩异常 
#0x01 TORQUE_ABNORMAL_SPEED_CLEAR 扭矩异常后保持状态,速度置为0
#0x02 TORQUE_ABNORMAL_MANUAL      人工介入
#0x03 TORQUE_ABNORMAL_DISBLE      扭矩硬件超限，退出使能模式
uint8 torque_state  
#人工介入堵转电流,单位0.1A
uint32 max_block_current     
#转向堵转检测持续时间,单位0.1s
uint16 turn_block_detect_intv
#人工介入堵转检测持续时间,单位0.1s    
uint16 manual_block_detect_intv
 
 
 





