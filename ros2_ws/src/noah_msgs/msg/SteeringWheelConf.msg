
# 方向盘状态信息

std_msgs/Header header

#命令类型.
#0x00   查询指令
#0x01   配置指令
#0x02   角度标定
#0x03   旋转角度
#0x04   进入人工模式
#0x05   进入无人模式
#0x06   赋值角度标定
#0x07   行走(差值)角度标定
#0x08   配置角度传感器选项
#0x09   速度与时间设定，无角度传感器模式下使用
#0X0A   设置堵转信息
uint16  cmd_type 


# 配置命令                  ID     描述                       单位
# CMD_ENABLE               0x0D  电机使能
# CMD_DISABLE              0x0C  电机失能
# CMD_SET_SPEED            0x00  顺时针速度设置              0.1rpm
# CMD_SET_SPEED_ANTICLOCK  0x01  逆时针速度设置              0.1rpm
# CMD_SET_POSITION         0x02    顺时针位置设置            1度
# CMD_SET_POSITION_ANTICLOCK 0x03  逆时针位置设置            1度                      
# CMD_SET_BLOCKAGE         0x06  堵转参数设置
# CMD_SET_DEVIATION        0x07  位置偏差参数设置
# CMD_SET_MODE             0x08  模式设置（8：位置 7：速度）                        
# CMD_SET_ACCEL            0x0A  加速时间设置
# CMD_SET_DECEL            0x0B  减速时间设置
# CMD_SET_TIMEOUT          0x0E  人机心跳超时时间设置
# CMD_SAVE_E2PROM          0x11  数据保存至 E2PROM
# CMD_CLEAR_ALARM          0x0F  报警清除

# 查询命令                  ID     描述                       单位
# QUERY_MOTOR_CURRENT      0x00  电机电流查询
# QUERY_MOTOR_STATUS       0x03  电机状态查询
# QUERY_POWER_VOLTAGE      0x0D  电源电压查询
# QUERY_ENCODER_COUNT      0x04  查询编码器计数值
# QUERY_PROGRAM_VERSION    0x01  查询程序版本
# QUERY_BLOCKAGE_PARAMS    0x06  查询堵转参数
# QUERY_DEVIATION_PARAMS   0x07  查询位置偏差参数
# QUERY_MODE               0x08  查询模式（8:位置 7:速度）
# QUERY_ANGLE_SENSOR_VALUE 0x09  查询角度传感器的值
# QUERY_ACCELERATION_TIME  0x0A  查询加速时间
# QUERY_DECELERATION_TIME  0x0B  查询减速时间
# QUERY_HEARTBEAT_TIMEOUT  0x0E  查询心跳超时时间
uint16 cmd_id
 
#命令参数
int32 data

#扩展参数
uint32 params 

#目标角度(角度值1度， 左负右正)
float64  target_angle   
 
 





