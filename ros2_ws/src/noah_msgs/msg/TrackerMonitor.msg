float32          x_diff_input   # 正值，车辆在目标点右侧。
float32          y_diff_input
float32          yaw_diff_input
float32          transverse_error_dis #正值，坐标系转换后的与车辆的横向差值，车辆在最近目标点右侧
int8             path_index               
float32          work_vel
float32          work_theta
int8             track_status
float32          lqr_xy_weight
float32          lqr_yaw_weight
float32          lqr_yaw_punish
float32          left_back_offset
float32          right_back_offset
float32          turn_vel_limit
float32          turn_theta_limit
float32          gain_compensation
float32          work_vel_limit
float32          work_theta_limit
float32          pre_point_dis
float32          min_vel
int8             work_line_num
float32          transverse_back_dis