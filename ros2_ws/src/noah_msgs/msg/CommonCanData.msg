# CommonCanData
# CAN 帧数据

# builtin_interfaces/Time  timestamp

std_msgs/Header header  # 发送时间辍
                        # msg.header.stamp = this->get_clock()->now();//获取当前时间
                        # msg.header.frame_id = "noah_chassis_xunjian_run_node";//赋值帧id

uint8   dev_index       #CAN设备id , 0开始
uint8   can_index       #CAN 通道id, 0开始
uint32  id              #帧ID
int8    timeflag        #是否使用时间标识
int8    sendtype        #发送帧类型，0正常发送 1单次发送(只发送一次)。多节点通信中务必设置为0
int8    remoteflag      # 是否是远程帧
int8    externflag      # 是否是扩展帧
int8    datalen
uint8[8] data
# uint8[]  data


