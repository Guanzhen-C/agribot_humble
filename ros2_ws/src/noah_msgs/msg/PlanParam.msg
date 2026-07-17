geometry_msgs/Polygon polygon  #区块坐标
int8 path_mode      #路径规划模式
float64 path_width   #耕宽
float64 car_radius #转弯半径
int8 turn_mode
int8 work_line_num


#################################################################

int8 PATHMODE_COVERAGE     = 0     # 全覆盖，鱼尾使用
int8 PATHMODE_ROTATE       = 1     # 回型    
int8 PATHMODE_XUA          = 2     # 中间回型
int8 PATHMODE_EDGECOVER    = 3     # 沿多变形轮廓覆盖(多边形内缩模式)
int8 PATHMODE_STRAIGHTLIE  = 4     # 直线
int8 PATHMODE_CAUYANG      = 5     # 采样
int8 PATHMODE_XUNJIAN      = 6     # 巡检（往返）
int8 PATHMODE_COVERAGEHOR  = 7     # 全覆盖， 直接转向，全部向、履带底盘使用     

int8 PATHMODE_TEST_EIGHT   = 101   #测试
