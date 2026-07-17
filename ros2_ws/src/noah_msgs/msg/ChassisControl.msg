#车辆底盘的控制指令

std_msgs/Header header

geometry_msgs/Twist twist       # 运动控制

uint8       throttle            #油门百分比 [0, 100]
uint8       brake               #刹车踏板百分比 arget brake in percentage [0, 100]
uint8       steering_rate       # [0, 100] 转向率 target non-directional steering rate
int8       steering_ratio      # [-100, 100] 目标转向角度百分比（左正右负）

bool        parking_brake       #是否驻车制动啮合， true:engaged
bool        engine_on_off       #引擎启停 true: engine on
bool        four_wheel_enable   #四驱使能控制


float32     velocity            #目标速度, in m/s 
float32     acceleration        #目标加速度 in m`s^-2
float32     steer_angle         #目标转向角度值，弧度值(正值：向左转向；负值：向右转向)

int8        driving_mode        #驾驶模式,对应 EDriverMode ，0人工， 1自动驾驶， 2只能控制转向，3只能控制速度（油门/刹车）
int8        gear_position       #档位 0空档，1前进，2后退，3驻车停泊i，4低速当
                                #巡检机器人是否上电，0下电、1上电

int8        turn_signal         #转向灯 0停止，1 左转向，2右转向
bool        high_beam           #前射强光灯
                                #碎枝机灯
                                #巡检灯 


bool        low_beam            #前射近光灯
                                #巡检无人化指示灯

bool        horn                #鸣笛
bool        emergency_light     #应急灯
bool        horn_open           #喇叭控制 0关闭，1打开
bool        work_or_no          #是否工作状态，0否，1是
                                #碎枝机电机使能
                                #测土是否使能
