std_msgs/Header     header                          #发送时间辍

bool                is_need_work                    #是否需要拍照，0否1是

uint8 		        numbers_of_photograph 	        #拍照总张数
int16[]	            photo_horizontal_angle          #拍照水平角度，0~359度，单位1度
int16[]	            photo_pitch_angle 		        #拍照俯仰角度，0~359度，单位1度
			
uint8 		        numbers_of_videorecord		    #录像总段数
int16[]	            video_horizontal_angle_begin	#录像起始水平角度，0~359度，单位1度
int16[]	            video_pitch_angle_begin		    #录像起始俯仰角度，0~359度，单位1度
int16[]	            video_horizontal_angle_end	    #录像结束水平角度，0~359度，单位1度
int16[]	            video_pitch_angle_end			#录像结束俯仰角度，0~359度，单位1度