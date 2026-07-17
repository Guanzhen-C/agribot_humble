# 机器人自检异常情况

# values记录当下所有监控单元的异常，最多监控32类情况
# 受监控的模块反馈自身异常情况


#状态值变化索引
uint8[] indexs
uint8[] values

#定义所有的参数索引
uint8  MAX_NUM         = 32


##--------------------------------------------------##

uint8   NONE = 0        #无异常
uint8   INVALID = 255   #失能，失效

# 监控单元类型索引
uint8   LOCATION    = 0     #定位
uint8   VCU_CONN    = 1     #vcu连接 
uint8   NET_CONN    = 2     #网络连接 
uint8   REMOTE_CONN = 3     #遥控器连接 
uint8   BATTERRY    = 4     #电池异常 
uint8   MOTOR       = 5     #电机异常情况
uint8   CAT_4G      = 6     # 4G网卡
uint8   GNSS_MODULE = 7     # gnss 模组
uint8   IMU_MODULE  = 8     # imu 模组
