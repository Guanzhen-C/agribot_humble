# 分发所有系统的激活情况，收到此消息后，获取自身的激活状态
# 通过自身激活状态来处理是否参与运算

# 使用int32数组模拟bitmap, 用来表示各个功能系统的激活状态
# 当前使用int32[3]表示 32*3-1=95个状态值：1是因为下标从1开始，因此bitmap第1位为无效位

# 使用例子: 以跟随功能为例子
# int index = GET_INDEX_IN_BITMAP( ESystemNode_Follow );
# bool enable = GET_BOOL_RESULT_IN_BITMAP(states[index], ESystemNode_Follow)


int8    MAX_SYSID_BITMAP_LEN = 3

int32[3]    states


 