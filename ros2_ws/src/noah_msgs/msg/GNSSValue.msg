std_msgs/Header header      # 发送时间辍
float64 longitude    
float64 latitude
float64 longitude_front    #质心前置后的经纬度
float64 latitude_front      #质心前置后的经纬度    
float64 heading   
float64 hdop            # 水平精度分子
float64 utc_time        #utc时间，时分秒.毫秒
int64   utc_date        # UTC日期，ddmmyy(日月年)格式   
int16   status          # 接受状态

geometry_msgs/Pose pose             #当前位姿数据,对应frame_id:map