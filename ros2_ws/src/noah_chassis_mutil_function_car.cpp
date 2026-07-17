#include <math.h>
#include "noah_framework/errordef.h"
#include "noah_chassis_mutil_function_car/noah_chassis_mutil_function_car.h"
#include "noah_chassis_common/chassis_utils.h"
#include "noah_framework/utility.h"
#include "noah_framework/globaldef.h"



namespace noah_chassis
{


uint32_t SEND_FRAME_ID;          //用于发送运动控制的CAN数据,帧发送ID,见CommonCanData

const int   MOTION_VEL_EXPITY_MSCS = 1000;  //底盘运动控制指令时效时长（ms）
const int   CAN_TRANS_INTERVAL  = 100;      //CAN指令发送周期ms
const uint32_t RECV_FRAME_ID1 = 0x532;      //周期报文ID1
const uint32_t RECV_FRAME_ID2 = 0x533;      //周期报文ID2
const uint32_t RECV_FRAME_ID3 = 0x534;      //周期报文ID2
const uint32_t RECV_FRAME_582 = 0x582;      //周期报文ID2
const uint8_t CAN_DATA_LEN = 8;     //can帧数据长度
const float   BACKWARD_MOTOR_RATE = 1.0f; //倒车电机速率与前向速率比
const float   HIGH_STEER_RATE_THRESHOLD = 0.75f;   //急速转向速率阈值，超此值后主动降速

int8_t DEVICE_INDEX = 0;         //用于发送CAN数据，CAN设备号,见CommonCanData
int8_t CHANNEL_INDEX = 0;        //用于发送CAN数据，CAN通道号,见CommonCanData

float MY_PI = NOAH_PI;
int32_t ERR_STATE = 0 ;             //节点状态，心跳发布

int16_t Motor_Max_Rotate;       //需要配置的参数，电机最大转速
float Reduction_Ratio ;         //需要配置的参数，减速比
float Wheel_Perimeter ;         //需要配置的参数，驱动轮周长（mm/r）

float Max_Round_Speed = 0.2f;   //原地转向速度 m/s
float Max_Straight_Speed = 1.0f;//车辆最大行使速度 m/s
float Max_Steer_Angle = 1.0f;   //最大转向角度(弧度值)

float Wheel_Spacing;        //履带间距
float Turn_Gain_Round;      //转向增益系数

//均值处理电机转速
const int BUFF_MOTOR_NUM = 5;  
float motor_buff_left[BUFF_MOTOR_NUM] = {0};
float motor_buff_right[BUFF_MOTOR_NUM] = {0};
int avarage_index = 0;

//****&&&&****&&&&//
// int8_t Can_Command_Pub_Motor[8]={0};//小农机电机的控制指令。0+1，电机1速度高+低字节；4+5，电机2速度高+低字节。

int8_t Can_State_Sub_Motor[16]={0};//小农机电机的状态数据
int8_t Can_Command_Pub_Funtion[8]={0};//小农机功能的控制指令。

//计算左电机和右点击转速的平均值，并将结果存储在输入的变量中
void Average_Filter(float inLeft, float inRight, float& outLeft, float& outRight) 
{
    motor_buff_left[avarage_index] = inLeft;
    motor_buff_right[avarage_index] = inRight;
    avarage_index += 1;
    if(avarage_index >= BUFF_MOTOR_NUM){
        avarage_index = 0;
    }

    float sum_left = 0;
    float sum_right = 0;
    for(int i = 0; i < BUFF_MOTOR_NUM; ++ i)
    {
        sum_left += motor_buff_left[i];
        sum_right += motor_buff_right[i];
    }
    outLeft = (sum_left /BUFF_MOTOR_NUM);
    outRight = (sum_right  /BUFF_MOTOR_NUM);
}

    
CChassisMutilFunctionCar::CChassisMutilFunctionCar(const rclcpp::NodeOptions& optiopns): 
    CChassisInterface("Chassis_Mutil_Function_Car_Node", optiopns)
{   
    //添加
    this->declare_parameter<int32_t>("send_frame_id",0x514);
    this->declare_parameter<int16_t>("motor_max_rotate", 3000);

    this->declare_parameter<double>("wheel_spacing", 0.94);
    this->declare_parameter<float>("reduction_ratio", 30.0);
    this->declare_parameter<float>("drivingwheel_d",200.0);
    this->declare_parameter<float>("max_straight_speed",1.0f);
    this->declare_parameter<int>("max_steer_angle", 30);   //最大转弯角度（角度值） 
    this->declare_parameter<float>("max_round_speed",0.2f);
    this->declare_parameter<double>("turn_gain_round", 1.0);
    shootWater = 0;

} 

CChassisMutilFunctionCar::~CChassisMutilFunctionCar()
{

}

noah_framework::CallbackReturn CChassisMutilFunctionCar::on_configure(const rclcpp_lifecycle::State &state)
{
    //初始化列表，用于调用基类CChassisInterface的构造函数，并传递参数
    CChassisInterface::on_configure(state);
    //修改
    SEND_FRAME_ID = this->get_parameter("send_frame_id").as_int();  
    Motor_Max_Rotate = this->get_parameter("motor_max_rotate").as_int();            //电机最大转速
    Wheel_Spacing = this->get_parameter("wheel_spacing").as_double();
    Reduction_Ratio = this->get_parameter("reduction_ratio").as_double();     

    Wheel_Perimeter = this->get_parameter("drivingwheel_d").as_double() * MY_PI;   // 周长（mm/r）
    Max_Straight_Speed = this->get_parameter("max_straight_speed").as_double();    // 最大行进速度，m/s,不包括转弯叠加的速度
    float Max_Round_Speed = this->get_parameter("max_round_speed").as_double();    // 原地转,速度最大差值，m/s
    
    int temp_steer = this->get_parameter("max_steer_angle").as_int();
    Max_Steer_Angle = temp_steer*3.14f/180.0f; 
    Turn_Gain_Round = this->get_parameter("turn_gain_round").as_double(); 
    
    return noah_framework::CallbackReturn::SUCCESS;
}


//生命周期回调函数，用来进行初始化操作
noah_framework::CallbackReturn CChassisMutilFunctionCar::on_activate(const rclcpp_lifecycle::State &state)
{
    CChassisInterface::on_activate(state);

    m_lost_timelapse = 0;
    m_command_buff.CleanUp();
    m_chassisData.CleanUp();

    return noah_framework::CallbackReturn::SUCCESS;
}


//停用前处理函数
noah_framework::CallbackReturn CChassisMutilFunctionCar::on_deactivate(const rclcpp_lifecycle::State &state)
{
    CChassisInterface::on_deactivate(state);

    return noah_framework::CallbackReturn::SUCCESS;
}


noah_framework::CallbackReturn CChassisMutilFunctionCar::on_cleanup(const rclcpp_lifecycle::State &state)
{
    CChassisInterface::on_cleanup(state);
    
    m_command_buff.CleanUp();

    m_command_buff.brakeValue = 100;    //刹车    
    //
    m_command_buff.moter_value_1 = 0;   //左电机
    m_command_buff.moter_value_2 = 0;   //右电机

    return noah_framework::CallbackReturn::SUCCESS;
}

int sync_interval = 0;

//Tick函数，用来计时
void CChassisMutilFunctionCar::OnTick(int deltaTime)
{  
    if(deltaTime < 0)
        return;

    
    m_lost_timelapse += deltaTime;

     
    if(m_lost_timelapse > MOTION_VEL_EXPITY_MSCS)    //超时1s后停车
    {
        SetLostControl(true);
        m_lost_timelapse = 0;
    }
     
    m_trans_interval += deltaTime;
     
    if(m_trans_interval >= CAN_TRANS_INTERVAL)
    {
        m_trans_interval = 0;
        
        handleActionValues();

        CalControlCmd();
        usleep(2000);
        CalTaskCtrl_582Frame();
    }
    
    sync_interval += deltaTime;
    if(sync_interval > 1000)
    {
        sync_interval = 0;
        SyncStatusValue();
    }
}


//接受控制指令回调
void CChassisMutilFunctionCar::OnRecvControlMsg(const noah_msgs::msg::ChassisControl::SharedPtr msg)
{
    auto nowTime = this->get_clock()->now();
    double time_diff = noah_framework::utility::TimestampDiff(msg->header.stamp, nowTime); //返回毫秒
    if(time_diff > MOTION_VEL_EXPITY_MSCS)
    {
        LOG_F(WARNING, "控制指令接收时间超时，检查发送过程：%s", msg->header.frame_id.c_str());
        return;
    }

    m_command_buff.velocity = msg->velocity;
    m_command_buff.steer = msg->steer_angle;
    m_command_buff.switch_light =  msg->work_or_no;
    m_command_buff.brakeValue = msg->brake;
 
    SetLostControl(false);
    m_lost_timelapse = 0;
}

/// @brief 计算当前帧控制指令
void CChassisMutilFunctionCar::CalControlCmd()
{
    try
    {
        noah_msgs::msg::CommonCanData  msg;
        
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "Chassis_Mutil_Function_Car";
        
        msg.dev_index = DEVICE_INDEX; 
        msg.can_index = CHANNEL_INDEX;
        msg.id = SEND_FRAME_ID;
        msg.timeflag = 0;      
        msg.sendtype = 0;      
        msg.remoteflag = 0;    
        msg.externflag = 0;    
        msg.datalen = CAN_DATA_LEN; 
        
        //将存储CAN指令的数组全部设置 0x00
        // msg.data.resize(CAN_DATA_LEN);
        for(int i = 0; i < CAN_DATA_LEN ; ++i)    
        {
            msg.data[i] = 0x00;
        }
        
        //检查是否失控
        if(GetLostControl())
        {   
            m_command_buff.brakeValue = 100;    //失控刹车            
        }
        float angular_velocity = m_command_buff.steer;
        float speed = m_command_buff.velocity;
        //如果刹车值大于1，则重置转向角度和速度为0
        if ( m_command_buff.brakeValue > 50)     //   0
        {
            angular_velocity = 0;
            speed = 0;
        }

        if(fabs(angular_velocity) > Max_Steer_Angle){
            angular_velocity = (angular_velocity >= 0) ? Max_Steer_Angle : (-1 * Max_Steer_Angle);
        }
        if(fabs(speed) > Max_Straight_Speed){
            speed = (speed >= 0) ? Max_Straight_Speed : (-1 * Max_Straight_Speed);
        }

        //计算两履带期望速度
        float v_left = 0;
        float v_right = 0;         
        CalMotorValue(angular_velocity, speed, v_left, v_right);
        //将速度值转为转速, 行使距离l = 转速/减速比*周长
        float moter_left = (v_left * 60.0f * 1000.0f / Wheel_Perimeter * Reduction_Ratio);
        float motor_right = (v_right * 60.0f * 1000.0f / Wheel_Perimeter * Reduction_Ratio);
    
        float ave_left = 0;
        float ave_right = 0;
        Average_Filter(moter_left, motor_right, ave_left, ave_right);//对电机值进行平均滤波
        //电机转速百分比
        int left_ratio = (int)(ave_left  / Motor_Max_Rotate * 100);
        int right_ratio = (int)(ave_right / Motor_Max_Rotate * 100);
        
        int8_t cmd_value = 0; 
        //刹车
        if (m_command_buff.brakeValue > 0){
            cmd_value = 0x03;
            // msg.data[0] = 1 + (1 <<1);
        }
        msg.data[0] = cmd_value;

        //电机1  
        cmd_value = left_ratio;  //获取左侧电机值
        msg.data[1] = cmd_value; //设置电机1 数据
        //电机2
        cmd_value = right_ratio; 
        msg.data[2] = cmd_value;
        // LOG_F(INFO,"final out left %d ,final out right %d ",left_ratio,right_ratio);
        
        //大灯控制
        cmd_value = m_command_buff.switch_light ? 1 : 0;
        msg.data[3] = cmd_value;  
        
        //发布消息
        m_canCtrl_pub->publish(std::move(msg));
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }    
}

void CChassisMutilFunctionCar::CalTaskCtrl_582Frame()
{
     try
    {
        noah_msgs::msg::CommonCanData  msg;
        
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "Chassis_Mutil_Function_Car";
        
        msg.dev_index = DEVICE_INDEX; 
        msg.can_index = CHANNEL_INDEX;
        msg.id = RECV_FRAME_582;
        msg.timeflag = 0;      
        msg.sendtype = 0;      
        msg.remoteflag = 0;    
        msg.externflag = 0;    
        msg.datalen = CAN_DATA_LEN; 
        
        //将存储CAN指令的数组全部设置 0x00
        // msg.data.resize(CAN_DATA_LEN);
        for(int i = 0; i < CAN_DATA_LEN ; ++i)    
        {
            msg.data[i] = 0x00;
        }
        msg.data[0] = shootWater;
        // LOG_F(INFO,"shootWater is %d",shootWater);
        // 3. 计算并赋值 Rollingcounter (Byte 6, 低4位, 0~15循环)
        // 使用静态变量保持上一次的计数状态。如果在类中有定义成员变量(例如m_rollCounter_582)，建议用成员变量代替。
        static uint8_t rolling_counter = 0; 
        msg.data[6] = rolling_counter & 0x0F; // 保证只取低4位
        
        rolling_counter++;
        if (rolling_counter > 15) {
            rolling_counter = 0;
        }

        // 4. 计算并赋值 Checksum (Byte 7)
        // 校验和 = Byte6 XOR Byte5 XOR Byte4 XOR Byte3 XOR Byte2 XOR Byte1 XOR Byte0
        msg.data[7] = msg.data[0] ^ msg.data[1] ^ msg.data[2] ^ 
                      msg.data[3] ^ msg.data[4] ^ msg.data[5] ^ msg.data[6];

        // 5. 将组装好的 msg 发送出去 (根据您的节点逻辑补充 publisher 发送代码)
        // can_pub_->publish(msg); 
        m_canCtrl_pub->publish(std::move(msg));

    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }


    

}

//根据转向\速度,计算驱动电机控制量
//v_left: 左驱动履带速度值 m/s
//v_right: 右驱动履带速度值
void CChassisMutilFunctionCar::CalMotorValue(float angular_velocity, float velocity, float& v_left, float& v_right)
{
    v_left = v_right = 0;
    // LOG_F(INFO,"chassis rec vel %f ,chassis rec angle %f ",velocity,angular_velocity);
    //线速度角速度都为0 返回0
    if(noah_framework::is_float_zero(velocity) &&
        noah_framework::is_float_zero(angular_velocity))
    {
        return;
    }

    bool forward = true;    //前进方向
    float max_threshold = Max_Straight_Speed;  //最大前进速度
    
    if(velocity < 0)
    {
        max_threshold = (max_threshold * BACKWARD_MOTOR_RATE);  //倒车最大转速
        forward = false;

        if(fabs(velocity) > max_threshold){
            velocity = max_threshold * -1;
        }
    }
    //根据转向计算差速比,转向 左正 右负
    bool dir_left = true;
    if(angular_velocity < 0){      //右转
        dir_left = false;
    }
    if(forward){
            //有前进速度和前进角度
        v_left = velocity - ((Wheel_Spacing * angular_velocity) * 0.5f * Turn_Gain_Round);
        v_right = velocity + ((Wheel_Spacing * angular_velocity) * 0.5f* Turn_Gain_Round);

        if(dir_left){
            // v_left = velocity - ((Wheel_Spacing * angular_velocity) * 0.5f * Turn_Gain_Round);
            v_right = velocity;

        }else
        {
            v_left = velocity;
            // v_right = velocity + ((Wheel_Spacing * angular_velocity) * 0.5f * Turn_Gain_Round);
        }
        if(v_left *v_right < 0)
        {
            v_left =  v_left <0 ? 5.0 :v_left;
            v_right = v_right <0 ? 5.0 :v_right;
            // LOG_F(INFO,"chassis out vel %f ,chassis out angle %f ",v_left,v_right);
            
        }
    }else{
        if(dir_left){
            v_right = velocity + ((Wheel_Spacing * angular_velocity) * 0.5f * Turn_Gain_Round);
            v_left = velocity ;

        }else
        {
            v_right = velocity;
            v_left = velocity - ((Wheel_Spacing * angular_velocity) * 0.5f * Turn_Gain_Round);
        }
        if(v_left *v_right <0)
        {
            v_left = v_left >0 ? 5.0 : v_left;
            v_right = v_right >0 ? 5.0 : v_right;
            // LOG_F(INFO,"chassis out vel %f ,chassis out angle %f ",v_left,v_right);
        }
    }
    

    if(noah_framework::is_float_zero(v_left) && 
        noah_framework::is_float_zero(v_right))
    {
        //速度静止不动
        return;
    }
    
    //前进速度为0,或两条履带出现正反转时，使用原地转向策略
    if(noah_framework::is_float_zero(velocity) )
    {
        float diff_vel = Max_Round_Speed * 0.5f;    //使用配置量计算差速度值
        v_left = 0 - diff_vel * (dir_left ? 1 : -1);
        v_right = 0 + diff_vel * (dir_left ? 1 : -1);
    }
    
}

//根据转向角度计算降速值，在急转时减速处理
//返回速度减速值(绝对值)
float CChassisMutilFunctionCar::CalDeceleration(float angleRate, float curSpeedRate)
{
    return 0;

    // angleRate = fabs(angleRate);
    // curSpeedRate = fabs(curSpeedRate);    //速度转为绝对值处理
    // if(angleRate < HIGH_STEER_RATE_THRESHOLD)
    // {
    //     return 0;
    // }
    // if(curSpeedRate <= ORI_TURN_MOTOR_VALUE)
    //     return 0;
    // float subSteerRate = angleRate - HIGH_STEER_RATE_THRESHOLD;
    // float decrease = curSpeedRate * subSteerRate * DECREATION_INCREASE;
    // if(decrease > ( curSpeedRate * 0.6f)){
    //     decrease = curSpeedRate * 0.6f;     //最大减速60%
    // }
    // return decrease;
}

void CChassisMutilFunctionCar::OnRecvCanbusData(
    const noah_msgs::msg::CommonCanData::SharedPtr msg)
{
    const int frameId = msg->id;

    // 注意：你要解析 1/2/3 三帧，这里必须都包含
    if (!((frameId == RECV_FRAME_ID1) ||
          (frameId == RECV_FRAME_ID2) ||
          (frameId == RECV_FRAME_ID3)))
    {
        return;
    }

    if (msg->data.size() != CAN_DATA_LEN) {
        return;
    }

    // 通用临时变量
    uint8_t  data_temp  = 0;
    int8_t   data_temp1 = 0;
    int16_t  value      = 0;
    uint16_t value1     = 0;

    switch(frameId)
    {
        /*==================== RECV_FRAME_ID1 (0x532) ====================*/
        case RECV_FRAME_ID1:
        {
            // 工作模式 bit0~1
            m_chassisData.drivingMode = msg->data[0] & 0x03;

            // 急停状态 bit2（按你表：Car_Estop_Status startbit=2）
            m_chassisData.emergencySta = (msg->data[0] >> 2) & 0x01;

            // 如果你还需要“运行状态 bit3”，建议加字段 carRunningStatus
            // uint8_t carRunningStatus = (msg->data[0] >> 3) & 0x01;

            // 遥控连接状态 bit4~5
            m_chassisData.remoteConnectionSta = (msg->data[0] >> 4) & 0x03;

            // 日行灯 byte1 bit2
            m_chassisData.dayLights = (msg->data[1] >> 2) & 0x01;

            // 电池电压 byte2~3 （你现在写的高字节在前，这里保持一致）
            value1 = (uint16_t)((msg->data[2] << 8) | msg->data[3]);
            m_chassisData.batteryVoltage = value1;

            // （如果你还要解析 VRC/AUTO/MCU/BMS 通信故障，是 byte4 bit0~3）
            // uint8_t fault = msg->data[4];
            // m_chassisData.vrcCommFlt  = (fault >> 0) & 0x01;
            // m_chassisData.autoCommFlt = (fault >> 1) & 0x01;
            // m_chassisData.mcuCommFlt  = (fault >> 2) & 0x01;
            // m_chassisData.bmsCommFlt  = (fault >> 3) & 0x01;

            break;
        }

        /*==================== RECV_FRAME_ID2 (左电机 0x533) ====================*/
        case RECV_FRAME_ID2:
        {
            const uint8_t b0 = msg->data[0];

            // Byte0 故障/保护位（按你截图：OVP/UVP/TempFault/OCP/OLP/Hall/LockedRotor/Others）
            m_chassisData.overvoltageProtection = (b0 >> 0) & 0x01; // OVP
            m_chassisData.voltageProtect        = (b0 >> 1) & 0x01; // UVP
            m_chassisData.tempFaultFlag         = (b0 >> 2) & 0x01; // TempFault
            m_chassisData.overCurrent           = (b0 >> 3) & 0x01; // OCP
            m_chassisData.overloadProtect       = (b0 >> 4) & 0x01; // OLP
            m_chassisData.motorFault            = (b0 >> 5) & 0x01; // HallFault
            m_chassisData.lockedRotorProt       = (b0 >> 6) & 0x01; // LockedRotorProt堵转保护
            m_chassisData.othersFault           = (b0 >> 7) & 0x01; // OthersFault 其他故障

            // Speed：byte1~2，小端 int16
            value = (int16_t)((uint16_t)msg->data[1] | ((uint16_t)msg->data[2] << 8));
            m_chassisData.leftSpeed = value;

            // Voltage：byte3
            m_chassisData.leftElocityFeedback = msg->data[3];

            // Running_Current：byte4，int8（你的结构是 int16_t eleCurrent，也可存）
            data_temp1 = (int8_t)msg->data[4];
            m_chassisData.eleCurrent = (int16_t)data_temp1;

            // Temperature：byte5，实际 = raw - 40
            m_chassisData.motorTemperature = (int16_t)msg->data[5] - 40;

            break;
        }

        /*==================== RECV_FRAME_ID3 (右电机 0x534) ====================*/
        case RECV_FRAME_ID3:
        {
            const uint8_t b0 = msg->data[0];

            // 右侧如果你也要保留一套独立故障位，最好结构体里加 RightXXX
            // 但你当前结构体只有一份故障字段，这里给两种方案：

            // 方案A：右帧也覆盖同一份故障（不推荐，但兼容你当前结构体）
            // m_chassisData.overvoltageProtection = (b0 >> 0) & 0x01;
            // ...

            // 方案B：只解析右侧关键量（推荐：至少速度、电压、电流、温度独立）
            // 右侧速度
            value = (int16_t)((uint16_t)msg->data[1] | ((uint16_t)msg->data[2] << 8));
            m_chassisData.rightSpeed = value;

            // 右侧电压（你结构体目前没有 rightVoltage 字段，若需要请补）
            m_chassisData.righteElocityFeedback= msg->data[3];

            // 右侧电流（同样只有一份 eleCurrent，会覆盖左侧；建议加 rightCurrent）
            data_temp1 = (int8_t)msg->data[4];
            // m_chassisData.rightCurrent = (int16_t)data_temp1;  // 建议新增
            m_chassisData.eleCurrent = (int16_t)data_temp1;       // 不新增字段时只能覆盖

            // 右侧温度（你结构体只有 motorTemperature，会覆盖左侧；建议加 rightTemperature）
            // m_chassisData.rightMotorTemperature = (int16_t)msg->data[5] - 40; // 建议新增
            m_chassisData.motorTemperature = (int16_t)msg->data[5] - 40;

            break;
        }

        default:
            break;
    }
}



//接收到控制指令
void CChassisMutilFunctionCar::OnRecvDoAction(const noah_msgs::msg::ActionValue::SharedPtr msg)
{
        //检查数据长度
    int act_size = msg->action_indexs.size();
    if(act_size <= 0 || act_size != msg->action_values.size())
    {   
        ERROR_CONTEXT("OnRecvDoAction failed, act_size:", act_size);  
        LOG_F(WARNING, "RequestDoAction msg data size error!!:%s", msg->header.frame_id.c_str());

        return;
    }
    rclcpp::Time send_stamp = msg->header.stamp;
    auto now = this->get_clock()->now();
    double timelapse = noah_framework::utility::TimestampDiff(send_stamp, now);
    // RCLCPP_INFO(get_logger(), "outime %f",timelapse);
    if(timelapse >= 5000)
    {
        //超时
        RCLCPP_WARN(get_logger(), "RequestDoAction msg timeout !!");
        return;
    }
    m_Actionslock.Lock();
    for(int i = 0; i < act_size; ++ i)
    {
        int act_index = msg->action_indexs[i];
        int act_value = msg->action_values[i];
        //剔除不关心的动作指令
        // if(Action_Map.find(act_index) == Action_Map.end())
        // {
        //     continue;
        // }   
        //跳过无效数据
        if(act_value == noah_msgs::msg::ActionValue::INVALID)
        {            
            continue;
        }
        
        m_recvActions[act_index] = act_value;
        
        LOG_F(INFO, "T50 收到动作值 :%s = %d", getActionName(act_index).c_str(), act_value);
        
    }
    
    m_Actionslock.Unlock();

}
const std::string CChassisMutilFunctionCar::getActionName(const int act_index)
{
    switch (act_index)
    {
    case noah_msgs::msg::ActionValue::ACT_PROCEDURE:
        return "切流程";
    case noah_msgs::msg::ActionValue::ACT_BRAKING:
        return "刹车";
    case noah_msgs::msg::ActionValue::ACT_EMERGENCY:
        return "急停";
    case noah_msgs::msg::ActionValue::ACT_WORK_ENABLE:
        return "作业使能";
    case noah_msgs::msg::ActionValue::ACT_REAR_FLOAT_ENABLE:  
        return "后浮动使能";
    case noah_msgs::msg::ActionValue::ACT_REAR_LIFT_CTRL: 
        return "后提升控制";
    case noah_msgs::msg::ActionValue::ACT_REAR_LIFT_MAX: 
    case noah_msgs::msg::ActionValue::ACT_REAR_LIFT_MIN: 
        return "后提高度限制";
    case noah_msgs::msg::ActionValue::ACT_REAR_LIFT_VALUE: 
        return "后提升高度值";
    case noah_msgs::msg::ActionValue::ACT_REAR_POWEROUT_CTRL: 
        return "液压输出";
    case noah_msgs::msg::ActionValue::ACT_REAR_POWEROUT_SWITCH: 
        return "液压油路";
    case noah_msgs::msg::ActionValue::ACT_PTO_CTRL:
        return "PTO控制";
    case noah_msgs::msg::ActionValue::ACT_PTO_VALUE:
        return "PTO转速";
    case noah_msgs::msg::ActionValue::ACT_REAR_LIFT_MODE:
        return "后提升模式";
    case noah_msgs::msg::ActionValue::ACT_REAR_LIFT_FORCE_MODE:
        return "力位模式";
    default:
        return "忽略动作";
    }
    return "忽略动作";
}
void CChassisMutilFunctionCar::handleActionValues()
{
    // if(!GetRecvCtrlCmd())
    // {
    //     return;
    // }
    // SetRecvCtrlCmd(false);

    int action_value = noah_msgs::msg::ActionValue::INVALID;
    int INVALID = noah_msgs::msg::ActionValue::INVALID;

    // //0 刹车请求
    // int action_index = noah_msgs::msg::ActionValue::ACT_BRAKING;
    // action_value = m_recvActions[action_index];
    // control_data_.brkRequest = (action_value != INVALID && action_value >0) ? action_value : 0;
    

    shootWater = 0;
    int action_index = noah_msgs::msg::ActionValue::ACT_REAR_FLOAT_ENABLE;
    action_value = m_recvActions[action_index];
    shootWater = (action_value != INVALID && action_value >0) ? action_value: 0;
    
}

//将状态数据同步给logic
void CChassisMutilFunctionCar::SyncStatusValue()
{
    // 初始化底盘详细信息消息
    noah_msgs::msg::ChassisDetail msg;
    msg.header.frame_id = "chassis_mutil_function_car";
    msg.header.stamp = this->get_clock()->now();

    // 驾驶模式：遥控为0，无人为1
    msg.driving_mode = m_chassisData.drivingMode;    
    // 根据电机值1判断挡位
    if(m_command_buff.moter_value_1 == 0)
    {
        msg.gear = noah_framework::GEAR_PARKING;
    }else if(m_command_buff.moter_value_1 > 0){
        msg.gear = noah_framework::GEAR_DRIVE;
    }else
    {
        msg.gear = noah_framework::GEAR_REVERSE;
    }
    
    // 刹车
    msg.brake_ratio = m_command_buff.brakeValue; 
    // 油门比例，取电机值1的绝对值
    msg.throttle_ratio = abs(m_command_buff.moter_value_1);
    // 目标速度，取命令缓冲区中的速度
    msg.velocity_target = m_command_buff.velocity;
    // 目标转向角度，取命令缓冲区中的转向角度
    msg.steer_target = m_command_buff.steer;

    //急停状态
    msg.emergency =  m_chassisData.emergencySta;

    if( m_chassisData.emergencySta == 1)
    {
        msg.chassis_state = 2;
        msg.chassis_fail_level = 3;
    }else{
        msg.chassis_state = 0;
        msg.chassis_fail_level = 0;
    }
    msg.motor_rpm =  abs(m_chassisData.leftSpeed) > abs(m_chassisData.rightSpeed) ? abs(m_chassisData.leftSpeed):abs(m_chassisData.rightSpeed);
    msg.motor_voltage = m_chassisData.righteElocityFeedback > m_chassisData.leftElocityFeedback?
                                m_chassisData.righteElocityFeedback:m_chassisData.leftElocityFeedback;


    // 发布底盘详细信息消息
    m_DetailInfo_pub->publish(std::move(msg));
}     

float CChassisMutilFunctionCar::Clip(float value, float min, float max)
{
    if(value < min)
        return min;
    if(value > max)
        return max;
    return value;
}


}