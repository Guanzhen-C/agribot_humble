#文件上传话题

std_msgs/Header header

string  file_path    #上传文件路径(全路径)
int8    file_type    #文件类型, 图片=1 ，视频 =2，运行log=3，路径文本=4
int64   message_id   #默认 0
int64   task_id      #任务ID
int32   param_0      #参数0
int32   param_1      #参数1
float64 longitude
float64 latitude

###################

int8    FILETYPE_PIC    = 1     #文件类型图片
int8    FILETYPE_VIDEO  = 2
int8    FILETYPE_LOG    = 3
int8    FILETYPE_PATH   = 4

## 备注
# param_0 数值：上传图片时，表示对应的路径点索引值