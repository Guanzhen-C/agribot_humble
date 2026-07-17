# 激光雷达障碍检测输出
# LidarObstacles.msg 2023.07.13 ltb 
# #include "noah_msgs/msg/lidar_obstacles.hpp"

uint8     obstacles_device      #障碍检测设备ID  假定单线激光雷达为0X10
uint32    obstacles_hole_number #检测到的障碍总数
uint16[]  obstacles_number      #障碍物序号      0～
float32[] obstacles_angle       #障碍物角度 角度取整
float32[] obstacles_distance    #障碍物距离 m
float32[] obstacles_size        #障碍物尺寸 单位 度


