# 信号信息描述

std_msgs/Header header

# 信号信息消息
uint8 signal_type

# 连接状态，如 "Connected", "Disconnected", "Searching"
uint8 conn_status

# 运营商名称，如 "China Mobile", "Verizon"
string operator_name

#CCID
string ccid     

#IMEI
string imei     

# 信号强度，取值范围一般为 -120dBm 到 0dBm
int8 signal_strength

# 信号质量，取值范围 0 - 100； 移动网络中 0（信号最弱）到 31（信号最强），99 表示无信号
uint8 signal_quality

# 信号频率，单位为 Hz
float32 frequency

# 基站ID（仅适用于移动通信信号）
uint32 base_station_id

# 信噪比，单位为 dB
float32 snr

########## 枚举值 ############
# 网络类型
uint8 SIGNAL_TYPE_GSM     = 2     # GSM：2G 网络
uint8 SIGNAL_TYPE_WCDMA   = 3     # WCDMA：3G 网络
uint8 SIGNAL_TYPE_LTE     = 4     # LTE：4G 网络
uint8 SIGNAL_TYPE_NR5G    = 5     # NR5G 5G网络
uint8 SIGNAL_TYPE_WIFI    = 10    # WiFi
uint8 SIGNAL_TYPE_BLUETOOTH = 20  # 蓝牙    

#连接状态
uint8 CONN_STATUS_NONE        = 0     #没有检测到模块
uint8 CONN_STATUS_CONNECTED   = 1     #连接正常，且数据连接正常，已附着到网络
uint8 CONN_STATUS_DISCONN     = 2
uint8 CONN_STATUS_SEARCHING   = 3
uint8 CONN_STATUS_SIM_READY   = 4     #SIM 正常
uint8 CONN_STATUS_SIM_NOT     = 5     #SIM卡未插入
uint8 CONN_STATUS_SIM_PIN     = 6     #SIM 需要输入PIN
uint8 CONN_STATUS_SIM_PUK     = 7     #需要输入 PUK 解锁
uint8 CONN_STATUS_SIM_ERROR   = 8     #SIM卡未知错误
uint8 CONN_STATUS_CREG_NOT    = 9     #未注册，未搜索到网络
uint8 CONN_STATUS_CREG_LOCAL  = 10    # 已注册到本地网络
uint8 CONN_STATUS_CREG_SEARCH = 11    # 正在搜索网络
uint8 CONN_STATUS_CREG_REFUSE = 12    # 注册被拒绝
uint8 CONN_STATUS_CREG_ROAM   = 13    # 已注册到漫游网络
uint8 CONN_STATUS_NO_NET      = 14    # 数据连接未附着网络，不能上网

