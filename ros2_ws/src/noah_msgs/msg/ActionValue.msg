#机器人动作控制指令值/动作状态值
#发送单次控制指令,用于广播触发对应的操作, DoAction 话题原则上只能 Behavior节点发送

std_msgs/Header header

# 当前控制或反馈的动作存放在 action_indexs
int16[] action_indexs

# 对应动作的数据或状态数据
int32[] action_values

int16 INVALID = 32767       #默认无效值，使用无效值作初始数据

uint8 MAX_NUM   = 128       #最多支持的动作数量


#################################################################
#################################################################

# 所有的动作类型ID
# 注意：不要改变已有的动作索引值，在行为树文件中已经使用现有的索引值

# 通用类型
uint8 ACT_DRIVEMODE       = 0  # 无效
uint8 ACT_PROCEDURE       = 1  #切换工作流程：跟随，跟踪...,对应 EProcedureType
uint8 ACT_BRAKING         = 2  #刹车[0-100]刹车行程，0不刹车，100完全刹车
uint8 ACT_FOURDRIVE_ENABLE = 3  #四驱使能，默认0
uint8 ACT_EMERGENCY       = 4  #紧急状态（0解除, 其余值都要急停）
uint8 ACT_TASK_CANCEL   = 5     #作业任务取消

# 提示灯光/鸣笛等
uint8 ACT_HIGHBEAM        = 20   #前射强光开关
uint8 ACT_LOWBEAM         = 21   #前射弱光开关
uint8 ACT_TURN_SIGNAL     = 22  #转向灯 0停止 1左 2右 3双闪
uint8 ACT_WARNING_LED     = 23  #LED警示灯 使能 0关闭
uint8 ACT_HORN            = 24   #鸣笛 使能 0关闭 1开
uint8 ACT_WORK_ENABLE     = 25   #工作 使能，0停止作业，1启动作业

# 农机具控制动作
uint8 ACT_REAR_FLOAT_ENABLE     = 30    # 后浮动请求使能 0未使能 1使能
uint8 ACT_FRONT_FLOAT_ENABLE    = 31    # 前浮动请求使能
uint8 ACT_REAR_LIFT_CTRL        = 32    # 后提升控制  ENUM_LIFTCTRL_STOP 0停止 1上升 2下降
uint8 ACT_REAR_LIFT_MAX         = 33    # 后提升最大高度限制
uint8 ACT_REAR_LIFT_MIN         = 34    # 后提升最小高度限制
uint8 ACT_REAR_LIFT_VALUE       = 35    # 提升高度值

uint8 ACT_FRONT_LIFT_CTRL       = 36
uint8 ACT_FRONT_LIFT_MAX        = 37    # 前提升最大高度限制
uint8 ACT_FRONT_LIFT_MIN        = 38    # 前提升最小高度限制
uint8 ACT_FRONT_LIFT_VALUE      = 39    # 提升高度值

uint8 ACT_REAR_POWEROUT_CTRL    = 40    # 后液压输出控制
uint8 ACT_FRONT_POWEROUT_CTRL   = 41    # 前液压输出控制
uint8 ACT_REAR_POWEROUT_SWITCH  = 42    # 后油路选择 (按位取，0关闭1开，低0位表示1路，低1位表示2路..)
uint8 ACT_FRONT_POWEROUT_SWITCH = 43    # 前油路选择 (按位取，0关闭1开，低0位表示1路，低1位表示2路..)

uint8 ACT_PTO_CTRL              = 44    # pto 输出控制 0停止1正向2反向
uint8 ACT_PTO_VALUE             = 45    # pto 转速 rad/min

# 新增 T200
uint8 ACT_REAR_LIFT_MODE        = 50    # 后提升控制模式 0直控(T70) 1位控制 2力位控制(T200)
uint8 ACT_REAR_LIFT_FORCE_MODE  = 51    # 后提升力位控制模式 0:提升中位锁止；1:运输模式；2:力位控制模式
uint8 ACT_REAR_LIFT_FORCE_POS   = 52    # 力位综合调节值
uint8 ACT_REAR_LIFT_POS         = 52    # 耕深设定值

#小农机 三合一 碎枝机等
uint8 ACT_WORK_MOTOR_ENABLE     = 60    # 作业电机使能
uint8 ACT_WORK_MOTOR_GEAR       = 61    # 作业电机档位

#记录采样点
uint8 ACT_RECORDPOINT      =62  #1 存储采样点
#录制触发
uint8 ACT_RECORDPATH       =63   #1开始 0结束 2全部清除 3加载最新路径
# 相机 云台控制
uint8 ACT_CT_WORKING      = 64  #测土动作，0停止 1开始
uint8 ACT_XJ_PTZ_X        = 65  #云台左右动作，-64-64 
uint8 ACT_XJ_PTZ_Y        = 66  #云台上下动作，-64-64
uint8 ACT_XJ_CARMERA_P    = 67  #摄像头拍照 1拍照
uint8 ACT_XJ_CARMERA_V    = 68  #摄像头录像 1开始 0停止



###################################
############# 枚举值 ###############

uint8 ENUM_LIFTCTRL_STOP    = 0 #停止
uint8 ENUM_LIFTCTRL_UP      = 1 #上升
uint8 ENUM_LIFTCTRL_DOWN    = 2 #下降

uint8 ENUM_POWEROUT_STOP    = 0 #停止
uint8 ENUM_POWEROUT_UP      = 1 #升
uint8 ENUM_POWEROUT_DOWN    = 2 #下降

uint8 ENUM_PTOCTRL_STOP     = 0 #停止
uint8 ENUM_PTOCTRL_FORWARD  = 1 # 正向
uint8 ENUM_PTOCTRL_REVERSE  = 2 # 反