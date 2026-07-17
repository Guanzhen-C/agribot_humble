builtin_interfaces/Time  timestamp

int8    driving_mode                #控制模式，遥控还是无人

float32 max_straight_speed          #最大直线行使速度 m/s
float32 max_steer_angle             #最大转弯角度（角度值）
float32 speed                       #当前速度                   

# noah_msgs/ChassisDetail chassis_detail     # 底盘数据

geometry_msgs/Pose pose             #当前位姿数据,对应frame_id:map
float64 longitude       #经纬度
float64 latitude
float64 longitude_front       #质心迁移后经纬度
float64 latitude_front 
float64 heading

int8    gps_status      #gps数据信号值状态,: 浮点解，固定，差分， -1未连接
int8    can_status      #can 状态值， -1未连接
int8    network_status  #网络连接情况
int8    task_status     #作业情况
int8    mqtt_status
int8    procedure       #当前执行流程