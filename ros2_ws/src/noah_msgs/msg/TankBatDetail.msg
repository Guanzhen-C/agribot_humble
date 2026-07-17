# 获取底盘电驱数据
# include "noah_msgs/msg/tank_bat_detail.hpp"
# TankBatDetail  mf_topic_tankbat_detail
std_msgs/Header header

# 工作状态  
int8        driving_mode        #控制模式

# 电机数据
float32     motor_rotate_left   #左电机转速 转/分
float32     motor_rotate_right
float32     motor_current_left  #左电机电流
float32     motor_current_right 
float32     motor_voltage       #电机电压
float32     motor_temperature_left      #左电机温度
float32     motor_temperature_right     #右电机温度

# 运动数据
float32     velocity            #车辆行使线速度 m/s 
float32     steer_speed         #角速度，转向切线速度 m/s,左转为正

float32     velocity_target     #目标速度
float32     steer_target        #目标转向角度

# 灯光状态
int8        high_beam           #前射强光灯

# 电池数据
int8        bat_soc             #电池电量
float32     bat_voltage         #电池电压
float32     bat_current         #电池电流