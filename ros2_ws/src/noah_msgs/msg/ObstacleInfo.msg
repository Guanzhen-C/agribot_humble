#障碍物信息
# 注意，之前毫米波雷达 x便是横向 y表示纵向 z表示上下位置
# 现在统一到 base_link坐标系，X轴指向正前方，负数在后方； Y轴左侧，负数在右侧；Z轴在上方，负数下方

builtin_interfaces/Time stamp
uint32  obstacle_id     # 障碍ID
uint8   type            # 障碍类型
uint8   level           # 障碍级别
uint8   direction       # 方位
     
float32 x               # 坐标位置x,基于base_link：前方正，后方负 (米)
float32 y               # 坐标位置y,基于base_link：左侧正，右侧负 (米)    
float32 z               # 坐标位置z,基于base_link：上方正，下方负 (米)

float32 width           # 障碍物宽度 单位 m
float32 height          # 高度 单位 m

float32 distance_long   # 距离传感器 纵向距离(m),用来判断障碍级别的主要数据
float32 distance_lat    # 距离传感器 横向距离(m)
float32 distance        # 距离传感器直线距离

float32 vel_lat         # 目标横向速度,m/s 接近为负数，远离为正
float32 vel_long        # 目标纵向速度 m/s 接近为负数，远离为正数

int16   frame_count     #检测到的帧数据，用于过滤跳变结果


# 枚举值定
uint8   TYPE_NOMAL      = 0 #通用，没有识别出类型，按普通障碍处理
uint8   TYPE_VEHICLE    = 1 #车辆类型
uint8   TYPE_HUMAN      = 2 #人

# 预警级别
uint8 LEVEL_NORMAL  = 1 # 普通,不预警
uint8 LEVEL_WARN    = 2 # 预警
uint8 LEVEL_CRITICAL = 3 # 紧急


# 方位
uint8 DIRECT_FRONT  = 1     #前
uint8 DIRECT_REAR   = 2     #后
uint8 DIRECT_LEFT   = 3
uint8 DIRECT_RIGHT  = 4
uint8 DIRECT_UP = 5
uint8 DIRECT_DOWN = 6