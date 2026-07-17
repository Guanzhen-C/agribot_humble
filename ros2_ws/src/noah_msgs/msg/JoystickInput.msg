# 遥控器按键操作
#include "noah_msgs/msg/joystick_input.hpp"
#定义常量数值
int8    INVALID       = -1
float32 INVALID_FLOAT = -1.0

std_msgs/Header header

# 摇杆对应的值， 最多支持 3个摇杆数据， 对应的含义：AXES_1_X...
# int8[9]  axes
int8[4]  axes

# 定长数组，当前所有按键值
int8[16] button_values



# axes 摇杆坐标值所对应的索引
int8 AXES_1_X       = 0     # 1号（左）摇杆X方向的索引值, X对应左右
int8 AXES_1_Y       = 1     # 1号（左）摇杆Y方向的索引值， Y对应上下
int8 AXES_2_X       = 2
int8 AXES_2_Y       = 3
int8 AXES_3_X       = 4
int8 AXES_3_Y       = 5

# 手柄按键对应的索引
int8 BUTTON_MODEL   = 0
int8 BUTTON_START   = 1
int8 BUTTON_SELECT  = 2
int8 BUTTON_BACK    = 3
int8  BUTTON_LEFT   = 4
int8  BUTTON_RIGHT  = 5
int8  BUTTON_UP     = 6
int8  BUTTON_DOWN   = 7
int8  BUTTON_A      = 8
int8  BUTTON_B      = 9
int8  BUTTON_X      = 10
int8  BUTTON_Y      = 11
int8  BUTTON_L_1    = 12
int8  BUTTON_L_2    = 13
int8  BUTTON_R_1    = 14
int8  BUTTON_R_2    = 15


# 航空遥控器按键对应的索引
int8 CHANNEL_0      = 0
int8 CHANNEL_1      = 1
int8 CHANNEL_2      = 2
int8 CHANNEL_3      = 3
int8 CHANNEL_4      = 4
int8 CHANNEL_5      = 5
int8 CHANNEL_6      = 6
int8 CHANNEL_7      = 7
int8 CHANNEL_8      = 8
int8 CHANNEL_9      = 9
int8 CHANNEL_10     = 10
int8 CHANNEL_11     = 11
int8 CHANNEL_12     = 12
int8 CHANNEL_13     = 13
int8 CHANNEL_14     = 14
int8 CHANNEL_15     = 15
int8 CHANNEL_16     = 16

