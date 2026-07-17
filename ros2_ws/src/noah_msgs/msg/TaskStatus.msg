# 任务ID
uint64  task_id
# 任务状态,0无作业 1等待执行 2作业中 3结束 4准备作业 5等待 6取消
uint8   status
# 完成进度 0-1
float32 progress
# 目标路径点索引(索引从0开始)
int32 target_index
