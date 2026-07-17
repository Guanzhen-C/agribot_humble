# 机器人所有的状态数值
# 当机器人可以在一段时间内保持运行在一个阶段时，用状态值表示该阶段
# 注意  ： 所有状态值默认值都为 0,注意与枚举对应时， 0值为初始值

#状态值变化索引
uint8[] modify_indexs
int32[] modify_values

##--------------------------------------------------##

#定义所有的参数索引
uint16  MAX_NUM         = 32

uint8   DRIVING_MODE    = 0     #控制模式，是否在无人状态
uint8   EMERGENCY_EVENT = 1     #紧急状态， 0正常,当遇到紧急事件时，需要停车
uint8   PROCEDURE_VALUE = 2     #当前执行流程
uint8   CAN             = 3     #CAN设备状态 0未连通，1连通,2传输错误, 3重启
uint8   UPGRADE         = 4     #系统升级状态 0未升级， 1升级中 2升级成功 3升级失败
uint8   WORK_ENABLE     = 5     #远程启动，控制作业使能,允许执行作业
uint8   WORK_DIRCTION   = 6     #作业方向，0,1前进，2倒车
uint8   VOLTAGE         = 7     #电压 单位0.1V,精确到小数点后1位
uint8   CURRENT         = 8     #电流值 0.1A 精确到小数点后1位
uint8   BATTERY         = 9     #剩余电量(百分比 0-100) 整数类型
uint8   IS_FOLLOWING    = 10    #是否跟随
uint8   TRACK_STATUS    = 11    #跟踪状态
uint8   NET_STATUS      = 12    #网络信号状态 0 未联网，1-5信号强度（5信号最好）
uint8   GNSS            = 13    #北斗定位状态 0无信号 1单点 2差分 3BDS/PPS模式 4RTK固定解 5 RTK浮点解 6估算
uint8   TASK_STATUS     = 14    #作业执行状态 参考枚举 ETaskStatuse 
uint8   BRAKING_VALUE   = 15    #刹车行程
uint8   MQTT_STATUS     = 16    #MQTT联通状态, 0 未联通 1连接
uint8   AVOID_OBSTACLE  = 17    # 避障碍决策



##############################################################
####################### 状态枚举值 #############################
#控制模式
uint8 ENUM_DM_MANUAL    = 0     #遥控
uint8 ENUM_DM_AUTO      = 1     #无人
uint8 ENUM_DM_EMERGENCY = 2     #故障

#紧急异常
uint8 ENUM_EMERGENCY_NONE = 0
uint8 ENUM_EMERGENCY_GNSS = 1   #GPS丢失
uint8 ENUM_EMERGENCY_CAN  = 2   #canbus异常
uint8 ENUM_EMERGENCY_OB   = 3   #障碍物，紧急避障
uint8 ENUM_EMERGENCY_MANUAL = 4 #触发急停

#执行的流程
int8 ENUM_PROCDURE_INVALID = -1
int8 ENUM_PROCDURE_IDLE   =0    #待机中，无作业任务
int8 ENUM_PROCDURE_WORK   = 1   #作业中
int8 ENUM_PROCDURE_FOLLOW = 2   #跟随
int8 ENUM_PROCDURE_NAV    = 3   #自动导航
int8 ENUM_PROCDURE_CHARGE = 4   #充电中

#CAN 连接状态
int8 ENUM_CAN_NONE      = 0     #CAN未连接
int8 ENUM_CAN_CONNECT   = 1     #CAN连通
int8 ENUM_CAN_TRANSERR  = 2     #CAN数据传输错误
int8 ENUM_CAN_RESTART   = 3     #CAN重启

#在线升级状态, 1-100表示升级进度， 0特殊表述未升级；101及以上表示升级结果
uint8 ENUM_UPGRADE_NONE  = 0     #未升级/不需要升级
uint8 ENUM_UPGRADE_FINISH    = 101  #升级成功
uint8 ENUM_UPGRADE_DOWN_FAIL = 101  #下载失败
uint8 ENUM_UPGRADE_DOWN_SUCC = 102  #下载成功
uint8 ENUM_UPGRADE_INSTALL   = 103  #安装中
uint8 ENUM_UPGRADE_INS_FAIL  = 104  #安装失败
uint8 ENUM_UPGRADE_ABORT     = 104  #升级终止

#作业使能状态
int8 ENUM_WK_DISENABLE  = 0     #自动作业未使能，需要远程启动
int8 ENUM_WK_ENABLE     = 1     #使能作业

#网络情况
int8 ENUM_NET_SS_0      = 0     #无信号
int8 ENUM_NET_SS_1      = 1     #信号强度1
int8 ENUM_NET_SS_2      = 2
int8 ENUM_NET_SS_3      = 3
int8 ENUM_NET_SS_4      = 4
int8 ENUM_NET_SS_5      = 5     #信号强度5 强
int8 ENUM_NET_NOSIM     = 6     #没有sim卡
int8 ENUM_NET_NOREG     = 7     #未能注册联网


#GNSS定位情况
int8 ENUM_GNSS_NONE     = 0     #无信号
int8 ENUM_GNSS_SINGLE   = 1     #单点     
int8 ENUM_GNSS_DIFF     = 2     #差分
int8 ENUM_GNSS_BDS      = 3
int8 ENUM_GNSS_RTK      = 4     #4RTK固定解
int8 ENUM_GNSS_FLOAT    = 5     # RTK浮点解

# 避障决策
int8 ENUM_OBSTACLE_NORMAL = 0   # 正常行驶
int8 ENUM_OBSTACLE_SLOW_DOWN = 1 # 减速
int8 ENUM_OBSTACLE_LEFT_AVOID = 2   #左避让
int8 ENUM_OBSTACLE_RIGHT_AVOID = 3
int8 ENUM_OBSTACLE_BRAKE = 4    # 紧急刹车
int8 ENUM_OBSTACLE_STOP = 5     # 停车
