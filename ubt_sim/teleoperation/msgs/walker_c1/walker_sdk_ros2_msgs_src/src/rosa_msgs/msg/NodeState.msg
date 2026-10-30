
# frame_id 填充节点名称，stamp 填充状态时间戳
std_msgs/Header header

# 状态码
# 共计 7 位数
#   1. 状态类型标识符（1-3 表示一般状态，4-6 表示警告状态，7-9 表示错误状态）
#   2. 模块标识符
# 3-4. 子模块标识符
# 5-7. 实际状态码
uint32 state

# description 状态描述
string desc

# 自定义的状态类型
string msg_type
