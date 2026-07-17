std_msgs/Header     header                      #发送时间辍

uint8               is_work_finished            #是否完成拍照，0正在动作，1动作完成，2动作失败，FF等待
uint8 		        numbers_of_photograph	    #拍照总张数
string[]	        picture_file_name        	#照片的文件名

uint8 		        numbers_of_videorecord		#录像总段数
string[]	        videorecord_file_name		#录像的文件名
