# 获取底盘电驱数据
std_msgs/Header header

#控制模式 0遥控 1无人
uint8        driving_mode        
uint8       gear                #档位
uint8       emergency           #急停 0开 1响应
int8        rcm_conn_state      #与遥控器连接状况 0正常 1有故障
int8        adas_conn_state     #与ADAS无人驾驶连接状况 0正常 1有故障
# 底盘整体状态 0正常；1警告 2故障
uint8       chassis_state   
uint8       chassis_fail_level  # 故障等级 0 1,2,3 (越高越严重)
int16       brake_ratio         #刹车行程比例（0-100）
int16       throttle_ratio      #油门百分比（0-100）
bool        four_wheel_enable   #四驱使能

# 剩余电量百分比（0-100）
float32     soc_ratio           
# 电压（V，仅电动底盘有效）
float32     battery_voltage
# 电流（A，仅电动底盘有效，正数为放电，负数为充电）
float32     battery_current

# 电机机转速（rpm /min）
uint32      motor_rpm
bool        motor_started       #电机是否启动
int16       motor_status        #电机引擎状态
# 驱动电机温度（℃，-40~150）
float32     motor_temperature
float32     motor_power         #行走电机功率(KW)
float32     motor_voltage       #电机电压

# 线速度 m/s 
float32     velocity
# 线性加速度
float32     acceleration        
# 角速度（rad/s，左转为正，右转为负）
float32     angular_velocity
# 转向轮角度（rad，左转为正，范围[-π/2, π/2]）
float32     steer
# 左右轮速（rad/s，正数为前进）
float32     left_wheel_speed
float32     right_wheel_speed

float32     velocity_target     #目标速度
float32     steer_target        #目标转向轮角度 rad，左转为正，范围[-π/2, π/2]）
#float32    steer_speed         #目标角速度，转向切线速度 m/s,左转为正

uint8       turn_signal         #转向灯 0停止，1 左转向，2右转向
uint8       high_beam           #前射强光灯
uint8       low_beam            #前射近光灯
uint8       horn                #鸣笛
uint8       emergency_light     #应急灯

uint8       rear_float_state    #后浮动状态 0未使能(强压) 1浮动
uint8       pto_switch_state    #pto状态值 0停 1正转 2反转
uint16      pto_speed_rpm       #pto转速（rpm/min）     
