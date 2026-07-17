#各种小型农机的功能反馈数据
#include "noah_msgs/msg/tank_feedback_detail.hpp"

std_msgs/Header header      # 发送时间辍
                            # msg.header.stamp = this->get_clock()->now();//获取当前时间
                            # msg.header.frame_id = "noah_chassis_xunjian_run_node";//赋值帧id

uint16  alarm_status        #4bits一个对象，从低到高依次为电推杆1、电推杆2、采土状态，0为正常，1为电流或压力过大，2为伸长超时或采集失败，3为收缩超时或硬度采集失败

uint16  data_temperature        # 温度，单位0.01摄氏度，无偏移，范围-4000～8000,对应-40～80度
uint16   data_humidity           # 体积含水率，单位1%，范围0～100
uint16  data_elec_conductivity  # 电导率，单位us/cm，范围0～20000
uint16  data_hardness           # 土壤硬度，单位0.1kg/cm^2,范围-1000～1000，对应-100～100kg/cm^2


bool   state_work_model         # 工作模式，0遥控1无人
bool   state_high_beam          # 前大灯，0关1开
bool   state_relay_a            # 支流接触器A，0导通1断开

uint8  state_elec_rod_a         # a电推杆位置，0未知，1上限位，2下限位，3下降中，4上升中
uint8  state_data_get           # 数据获取的状态，0下降中，1下限位，2数据采集中，3上升中，4上限位

