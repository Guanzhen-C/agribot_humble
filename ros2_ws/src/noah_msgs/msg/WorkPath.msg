
# 过时消息文件，不要使用

std_msgs/Header header
geometry_msgs/PoseStamped[] poses
int8[]  status          #路径点状态
int8    turn_mode       #转弯模式
float32 speed           #作业速度,超过车最大速度后，使用车辆自身的最大速度
float32 turn_radius     #转弯半径
int8  work_line_num
