#各种小型农机的功能控制指令
#include "noah_msgs/msg/tank_function_control.hpp"
std_msgs/Header header      # 发送时间辍

bool        relay_on_offa       #继电器控制A，是否上电，1是0否
bool        relay_on_offb       #继电器控制B
bool        relay_on_offc       #继电器控制C

bool        work_enable         #工作使能，1有效0无效，使能土壤数据采集

bool        elecpush_up         #电推杆上升，1升0停
bool        elecpush_down       #电推杆下降，1降0停
bool        get_measure_data    #获取测试数据（两种），1是0否
bool        force_waking        #强制行走   
